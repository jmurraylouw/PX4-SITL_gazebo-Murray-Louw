#include "gazebo_forces_moments_actuators_plugin.h"

namespace gazebo {
GazeboForcesMomentsActuatorsPlugin::~GazeboForcesMomentsActuatorsPlugin() {
    update_connection_->~Connection();
}

void GazeboForcesMomentsActuatorsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Store the pointer to the model and world.
    model_ = _model;
    world_ = model_->GetWorld();

    // Get parameters.
    getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_, true);
    getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);

    getSdfParam<std::string>(_sdf, "copterConfig", copter_config_, copter_config_, true);
    getSdfParam<double>(_sdf, "motorTimeConstant", motor_time_constant_, motor_time_constant_, true);
    getSdfParam<double>(_sdf, "armLength", arm_len_, arm_len_, true);
    getSdfParam<double>(_sdf, "virtualYawMomentArm", virt_yaw_mom_arm_, virt_yaw_mom_arm_, true);

    getSdfParam<double>(_sdf, "maxMotorThrust", max_motor_thrust_, max_motor_thrust_, false);

    if (_sdf->HasElement("motor_models")) {
        sdf::ElementPtr motor_models = _sdf->GetElement("motor_models");
        sdf::ElementPtr motor = motor_models->GetElement("motor");
        int index = 0;
        while (motor) {
            if (motor->HasElement("coeffs")) {
                index = motor->Get<int>("motor_index");
                if (index < MAX_MOTORS) {
                    std::string coeff_str = motor->Get<std::string>("coeffs");

                    // Convert string to vector of doubles
                    std::stringstream ss(coeff_str); // Turn the string into a stream.
                    std::string tok;

                    char *endptr;
                    int coeff_num = 0;
                    while(getline(ss, tok, ' ')) {
                        motorModels[index][coeff_num] = strtod(tok.c_str(), &endptr);
                        coeff_num++;
                    }
                }
            } else {
                gzerr << "No coeffs, not parsing.\n";
                break;
            }
            motor = motor->GetNextElement("motor");
        }
        num_motors = index + 1;
    }

    if(num_motors <= 0 && max_motor_thrust_ <= 0) {
        gzthrow("[gazebo_forces_moments_actuators_plugin] Please provide either the maxMotorThrust or the motor 3rd order models.");
    }

    if(!IsValidConfig(copter_config_))
        gzthrow("[gazebo_forces_moments_actuators_plugin] The multicopter configuration \"" << copter_config_ << "\" is not supported.");

    // Initialize the node_handle.
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    // Pointer to the link, to be able to apply forces and torques on it.
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[gazebo_forces_moments_actuators_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboForcesMomentsActuatorsPlugin::OnUpdate, this, _1));

    // Initialize forces and moments filter
    z_filter_.reset(new FirstOrderFilter<double>(motor_time_constant_, motor_time_constant_, 0));
    dA_filter_.reset(new FirstOrderFilter<double>(motor_time_constant_, motor_time_constant_, 0));
    dE_filter_.reset(new FirstOrderFilter<double>(motor_time_constant_, motor_time_constant_, 0));
    dR_filter_.reset(new FirstOrderFilter<double>(motor_time_constant_, motor_time_constant_, 0));
}

// This gets called by the world update start event.
void GazeboForcesMomentsActuatorsPlugin::OnUpdate(const common::UpdateInfo& _info) {
    double commands[MAX_MOTORS];
    double motors[MAX_MOTORS];
    double dt;
    int i;

    // Connect to ROS
    if(!pubs_and_subs_created_) {
        CreatePubsAndSubs();
        pubs_and_subs_created_ = true;
    }

    // Wait for new motor commands
    std::unique_lock<std::mutex> lock(last_motor_message_mutex_);

    if (previous_motor_seq_ > 0) {
        while (previous_motor_seq_ == last_motor_message_.seq() && IsRunning()) {
            last_motor_message_cond_.wait_for(lock, std::chrono::microseconds(10));
        }
    }

    previous_motor_seq_ = last_motor_message_.seq();

    // Retrieve motor values
    for(i = 0 ; i < last_motor_message_.motor_throttle_size() ; i++) {
        commands[i] = last_motor_message_.motor_throttle(i);
    }
    ApplyMotorModel(commands, motors, last_motor_message_.motor_throttle_size());
    dt = last_motor_message_.dt();

    // Calculate virtual controls based on copter configuration.
    if(copter_config_ == "4x") {
        CalcControls4Cross(motors);
    } else if(copter_config_ == "4+") {
        CalcControls4Plus(motors);
    }

    // Apply first order filter.
    double z = z_filter_->updateFilter(virtual_controls[0], dt);
    double dA = dA_filter_->updateFilter(virtual_controls[1], dt);
    double dE = dE_filter_->updateFilter(virtual_controls[2], dt);
    double dR = dR_filter_->updateFilter(virtual_controls[3], dt);

    // Calculate forces and moments.
    ignition::math::Vector3d force = ignition::math::Vector3d(0, 0, z);
    ignition::math::Vector3d moment = ignition::math::Vector3d(dA, -dE, -dR); // Moments in XYZ, controls in NED
    link_->AddRelativeForce(force);
    link_->AddRelativeTorque(moment);
}

// Determine if simulation is running
bool GazeboForcesMomentsActuatorsPlugin::IsRunning() {
#if GAZEBO_MAJOR_VERSION >= 8
    return world_->Running();
#else
    return world_->GetRunning();
#endif
}

// Apply Motor Model
void GazeboForcesMomentsActuatorsPlugin::ApplyMotorModel(double commands[], double motors[], int num_motors) {
    if(max_motor_thrust_ > 0) {
        for(int i = 0 ; i < num_motors ; i++) {
            motors[i] = max_motor_thrust_ * commands[i];
        }
    } else {
        for(int i = 0 ; i < num_motors ; i++) {
            int pwm;

            // Don't do anything if commands is 0
            if(commands[i] <= 0) {
                motors[i] = 0;
                continue;
            }

            // Convert to PWM
            pwm = (commands[i] * 1000.0 + 1000.0);

            // Apply 3rd order model
            motors[i] = motorModels[i][0]*pow(pwm, 3) + motorModels[i][1]*pow(pwm, 2) + motorModels[i][2]*pwm + motorModels[i][3];
        }
    }
}

// Quadcopter X configuration (https://dev.px4.io/master/en/airframes/airframe_reference.html#quadrotor-x)
void GazeboForcesMomentsActuatorsPlugin::CalcControls4Cross(double motors[]) {
    virtual_controls[0] = (motors[0] + motors[1] + motors[2] + motors[3]);
    virtual_controls[1] = arm_len_/sqrt(2) * (-motors[0] + motors[1] + motors[2] - motors[3]);
    virtual_controls[2] = arm_len_/sqrt(2) * (motors[0] - motors[1] + motors[2] - motors[3]);
    virtual_controls[3] = virt_yaw_mom_arm_ * (motors[0] + motors[1] - motors[2] - motors[3]);
}

// Quadcopter + configuration (https://dev.px4.io/master/en/airframes/airframe_reference.html#quadrotor-)
void GazeboForcesMomentsActuatorsPlugin::CalcControls4Plus(double motors[]) {
    virtual_controls[0] = (motors[0] + motors[1] + motors[2] + motors[3]);
    virtual_controls[1] = arm_len_ * (-motors[0] + motors[1]);
    virtual_controls[2] = arm_len_ * (motors[2] - motors[3]);
    virtual_controls[3] = virt_yaw_mom_arm_ * (motors[0] + motors[1] - motors[2] - motors[3]);
}

void GazeboForcesMomentsActuatorsPlugin::MotorCommandCallback(CommandMotorThrottlePtr& motor_msg) {
    std::unique_lock<std::mutex> lock(last_motor_message_mutex_);

    // Save motor commands
    last_motor_message_ = *motor_msg;

    lock.unlock();
    last_motor_message_cond_.notify_one();
}

bool GazeboForcesMomentsActuatorsPlugin::IsValidConfig(std::string config) {
    bool valid;
    int i;

    valid = false;
    for(i = 0 ; i < sizeof(valid_copter_configs)/sizeof(valid_copter_configs[0]) ; i++) {
        if(config == valid_copter_configs[i]) {
            valid = true;
            break;
        }
    }

    return valid;
}

void GazeboForcesMomentsActuatorsPlugin::CreatePubsAndSubs() {
    // Create temporary "ConnectRosToGazeboTopic" publisher and message
    gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub = node_handle_->Advertise<std_msgs::msgs::ConnectRosToGazeboTopic>("~/" + kConnectRosToGazeboSubtopic, 1);

    // Subscribe to motor commands
    motor_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + motor_sub_topic_, &GazeboForcesMomentsActuatorsPlugin::MotorCommandCallback, this);

    // Connect to ROS
    std_msgs::msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
    connect_ros_to_gazebo_topic_msg.set_ros_topic(motor_sub_topic_);

    connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + model_->GetName() + motor_sub_topic_);
    connect_ros_to_gazebo_topic_msg.set_msgtype(std_msgs::msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_THROTTLE);
    gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg, true);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboForcesMomentsActuatorsPlugin)
} // namespace gazebo

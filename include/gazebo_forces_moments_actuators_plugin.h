#ifndef _GAZEBO_FORCES_MOMENTS_ACTUATORS_PLUGIN_H_
#define _GAZEBO_FORCES_MOMENTS_ACTUATORS_PLUGIN_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include <array>

#include <ConnectRosToGazeboTopic.pb.h>

#include <CommandMotorThrottle.pb.h>

#include "common.h"

#define MAX_MOTORS 16

namespace gazebo {

const std::string valid_copter_configs[] = {"4x", "4+"};

// Default values
static const std::string defaultNamespace = "";
static const std::string defaultLinkName = "base_link";

static const std::string defaultMotorCommandSubTopic = "/command/motor";

// Default params
static const std::string defaultCopterConfig = "4x";
static const double defaultMaxMotorThrust = -1;
static const double defaultMotorTimeConstant = 0;
static const double defaultArmLength = 0;
static const double defaultVirtualYawMomentArm = 0;

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorThrottle> CommandMotorThrottlePtr;

class GazeboForcesMomentsActuatorsPlugin : public ModelPlugin {
    public:
        GazeboForcesMomentsActuatorsPlugin() :  ModelPlugin(),
                                                namespace_(defaultNamespace),
                                                link_name_(defaultLinkName),
                                                motor_sub_topic_(defaultMotorCommandSubTopic),
                                                copter_config_(defaultCopterConfig),
                                                max_motor_thrust_(defaultMaxMotorThrust),
                                                motor_time_constant_(defaultMotorTimeConstant),
                                                arm_len_(defaultArmLength),
                                                virt_yaw_mom_arm_(defaultVirtualYawMomentArm),
                                                num_motors(0),
                                                pubs_and_subs_created_(false) {}

        virtual ~GazeboForcesMomentsActuatorsPlugin();

    protected:
        // \brief Load the plugin.
        // \param[in] _model Pointer to the model that loaded this plugin.
        // \param[in] _sdf SDF element that describes the plugin.
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        // \brief Called when the world is updated.
        // \param[in] _info Update timing information.
        void OnUpdate(const common::UpdateInfo& /*_info*/);

        // Check if value is a valid multicopter configuration
        bool IsValidConfig(std::string config);

        // Publish/Subscribe to ROS
        void CreatePubsAndSubs();

    private:
        // \brief Pointer to the update event connection.
        event::ConnectionPtr update_connection_;
        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::LinkPtr link_;

        std::string namespace_;
        std::string link_name_;
        std::string motor_sub_topic_;
        std::string copter_config_;

        double max_motor_thrust_;
        double motor_time_constant_;
        double arm_len_;
        double virt_yaw_mom_arm_;

        double motorModels[MAX_MOTORS][4];
        int num_motors;

        bool pubs_and_subs_created_;

        transport::NodePtr node_handle_;
        transport::SubscriberPtr motor_sub_;

        std::mutex last_motor_message_mutex_ {};
        std::condition_variable last_motor_message_cond_ {};
        mav_msgs::msgs::CommandMotorThrottle last_motor_message_;
        int64_t previous_motor_seq_ = 0;

        double virtual_controls[4]; // dT, dA, dE, dR
        std::unique_ptr<FirstOrderFilter<double>>  z_filter_;
        std::unique_ptr<FirstOrderFilter<double>>  dA_filter_;
        std::unique_ptr<FirstOrderFilter<double>>  dE_filter_;
        std::unique_ptr<FirstOrderFilter<double>>  dR_filter_;

        bool IsRunning();
        void ApplyMotorModel(double commnads[], double motors[], int num_motors);
        void CalcControls4Cross(double motors[]);
        void CalcControls4Plus(double motors[]);
        void MotorCommandCallback(CommandMotorThrottlePtr& motor_msg);
};
} // namespace gazebo
#endif // _GAZEBO_FORCES_MOMENTS_ACTUATORS_PLUGIN_H_
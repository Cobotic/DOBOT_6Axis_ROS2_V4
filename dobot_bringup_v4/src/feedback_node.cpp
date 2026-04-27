#include <dobot_bringup/command.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <dobot_msgs_v4/msg/robot_status.hpp>
#include <dobot_msgs_v4/msg/tool_vector_actual.hpp>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>
#include <vector>

namespace
{
static int env_int(const char *name, int default_value)
{
    const char *v = std::getenv(name);
    if (!v || !*v)
        return default_value;
    try
    {
        return std::stoi(std::string(v));
    }
    catch (...)
    {
        return default_value;
    }
}

static bool env_flag_with_fallback(const char *primary, const char *fallback, bool default_value)
{
    const char *p = std::getenv(primary);
    if (p && *p)
    {
        const int v = env_int(primary, default_value ? 1 : 0);
        return v != 0;
    }
    const int v = env_int(fallback, default_value ? 1 : 0);
    return v != 0;
}

static int env_int_with_fallback(const char *primary, const char *fallback, int default_value)
{
    const char *p = std::getenv(primary);
    if (p && *p)
        return env_int(primary, default_value);
    return env_int(fallback, default_value);
}

static bool try_enable_sched_fifo(int prio)
{
    sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = prio;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    return rc == 0;
}

static nlohmann::json realtime_to_json(const RealTimeData &rt)
{
    nlohmann::json root;

    root["len"] = rt.len;
    root["digital_input_bits"] = rt.digital_input_bits;
    root["digital_outputs"] = rt.digital_outputs;
    root["robot_mode"] = rt.robot_mode;
    root["controller_timer"] = rt.controller_timer;
    root["run_time"] = rt.run_time;
    root["test_value"] = rt.test_value;
    root["safety_mode"] = rt.safety_mode;
    root["speed_scaling"] = rt.speed_scaling;
    root["linear_momentum_norm"] = rt.linear_momentum_norm;
    root["v_main"] = rt.v_main;
    root["v_robot"] = rt.v_robot;
    root["i_robot"] = rt.i_robot;
    root["program_state"] = rt.program_state;
    root["safety_status"] = rt.safety_status;

    auto vec3 = [](const double *arr) {
        std::vector<double> out;
        out.reserve(3);
        for (int i = 0; i < 3; i++)
        {
            out.push_back(arr[i]);
        }
        return out;
    };
    auto vec6 = [](const double *arr) {
        std::vector<double> out;
        out.reserve(6);
        for (int i = 0; i < 6; i++)
        {
            out.push_back(arr[i]);
        }
        return out;
    };
    auto vec4 = [](const double *arr) {
        std::vector<double> out;
        out.reserve(4);
        for (int i = 0; i < 4; i++)
        {
            out.push_back(arr[i]);
        }
        return out;
    };

    root["tool_accelerometer_values"] = vec3(rt.tool_accelerometer_values);
    root["elbow_position"] = vec3(rt.elbow_position);
    root["elbow_velocity"] = vec3(rt.elbow_velocity);

    root["q_target"] = vec6(rt.q_target);
    root["qd_target"] = vec6(rt.qd_target);
    root["qdd_target"] = vec6(rt.qdd_target);
    root["i_target"] = vec6(rt.i_target);
    root["m_target"] = vec6(rt.m_target);
    root["q_actual"] = vec6(rt.q_actual);
    root["qd_actual"] = vec6(rt.qd_actual);
    root["i_actual"] = vec6(rt.i_actual);
    root["i_control"] = vec6(rt.i_control);
    root["tool_vector_actual"] = vec6(rt.tool_vector_actual);
    root["TCP_speed_actual"] = vec6(rt.TCP_speed_actual);
    root["TCP_force"] = vec6(rt.TCP_force);
    root["tool_vector_target"] = vec6(rt.tool_vector_target);
    root["TCP_speed_target"] = vec6(rt.TCP_speed_target);
    root["motor_temperatures"] = vec6(rt.motor_temperatures);
    root["joint_modes"] = vec6(rt.joint_modes);
    root["v_actual"] = vec6(rt.v_actual);

    root["handtype"] = {rt.handtype[0], rt.handtype[1], rt.handtype[2], rt.handtype[3]};
    root["userCoordinate"] = rt.userCoordinate;
    root["toolCoordinate"] = rt.toolCoordinate;
    root["isRunQueuedCmd"] = rt.isRunQueuedCmd;
    root["isPauseCmdFlag"] = rt.isPauseCmdFlag;
    root["velocityRatio"] = rt.velocityRatio;
    root["accelerationRatio"] = rt.accelerationRatio;
    root["jerkRatio"] = rt.jerkRatio;
    root["xyzVelocityRatio"] = rt.xyzVelocityRatio;
    root["rVelocityRatio"] = rt.rVelocityRatio;
    root["xyzAccelerationRatio"] = rt.xyzAccelerationRatio;
    root["rAccelerationRatio"] = rt.rAccelerationRatio;
    root["xyzJerkRatio"] = rt.xyzJerkRatio;
    root["rJerkRatio"] = rt.rJerkRatio;
    root["BrakeStatus"] = rt.BrakeStatus;
    root["EnableStatus"] = rt.EnableStatus;
    root["DragStatus"] = rt.DragStatus;
    root["RunningStatus"] = rt.RunningStatus;
    root["ErrorStatus"] = rt.ErrorStatus;
    root["JogStatus"] = rt.JogStatus;
    root["RobotType"] = rt.RobotType;
    root["DragButtonSignal"] = rt.DragButtonSignal;
    root["EnableButtonSignal"] = rt.EnableButtonSignal;
    root["RecordButtonSignal"] = rt.RecordButtonSignal;
    root["ReappearButtonSignal"] = rt.ReappearButtonSignal;
    root["JawButtonSignal"] = rt.JawButtonSignal;
    root["SixForceOnline"] = rt.SixForceOnline;
    root["CollisionStates"] = rt.CollisionStates;
    root["ArmApproachState"] = rt.ArmApproachState;
    root["J4ApproachState"] = rt.J4ApproachState;
    root["J5ApproachState"] = rt.J5ApproachState;
    root["J6ApproachState"] = rt.J6ApproachState;
    root["vibrationDisZ"] = rt.vibrationDisZ;
    root["currentCommandId"] = rt.currentCommandId;
    root["m_actual"] = vec6(rt.m_actual);

    root["load"] = rt.load;
    root["centerX"] = rt.centerX;
    root["centerY"] = rt.centerY;
    root["centerZ"] = rt.centerZ;

    root["user"] = vec6(rt.user);
    root["tool"] = vec6(rt.tool);

    root["TraceIndex"] = rt.TraceIndex;
    root["SixForceValue"] = vec6(rt.SixForceValue);
    root["TargetQuaternion"] = vec4(rt.TargetQuaternion);
    root["ActualQuaternion"] = vec4(rt.ActualQuaternion);
    root["AutoManualMode"] = rt.AutoManualMode;

    return root;
}
} // namespace

class CRRobotFeedbackNode : public rclcpp::Node
{
public:
    CRRobotFeedbackNode()
        : rclcpp::Node("dobot_feedback")
    {
    }

    void init()
    {
        const bool sched_fifo = env_flag_with_fallback(
            "ANYROB_FEEDBACK_SCHED_FIFO",
            "ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO",
            false);
        const int prio = env_int_with_fallback(
            "ANYROB_FEEDBACK_SCHED_FIFO_PRIORITY",
            "ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO_PRIORITY",
            99);

        if (sched_fifo)
        {
            if (try_enable_sched_fifo(prio))
            {
                RCLCPP_INFO(this->get_logger(), "SCHED_FIFO enabled (feedback node, prio=%d)", prio);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "SCHED_FIFO requested for feedback node but failed (prio=%d)", prio);
            }
        }

        this->declare_parameter("robot_ip_address", "192.168.1.6");
        this->declare_parameter("robot_number", 1);
        this->declare_parameter("robot_node_name", "dobot_bringup_ros2");
        this->declare_parameter("JointStatePublishRate", 10.0);
        this->declare_parameter("FeedInfoPublishRate", 100.0);

        this->get_parameter("robot_ip_address", robot_ip_);
        this->get_parameter("robot_number", robot_number_);
        this->get_parameter("robot_node_name", robot_node_name_);

        joint_rate_hz_ = this->get_parameter("JointStatePublishRate").as_double();
        feed_rate_hz_ = this->get_parameter("FeedInfoPublishRate").as_double();

        if (joint_rate_hz_ <= 0.0)
            joint_rate_hz_ = 10.0;
        if (feed_rate_hz_ <= 0.0)
            feed_rate_hz_ = 100.0;

        // Realtime-only commander (no dashboard port)
        commander_ = std::make_shared<CRCommanderRos2>(robot_ip_, false);
        commander_->init();

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_robot", 10);
        robot_status_pub_ = this->create_publisher<dobot_msgs_v4::msg::RobotStatus>("dobot_msgs_v4/msg/RobotStatus", 10);
        tool_vector_pub_ = this->create_publisher<dobot_msgs_v4::msg::ToolVectorActual>("dobot_msgs_v4/msg/ToolVectorActual", 10);

        // Publish FeedInfo under the instance namespace (e.g. /robot1/<robot_node_name>/msg/FeedInfo).
        // This keeps multi-robot setups from colliding on the same absolute topic.
        const std::string topic_feed_info = robot_node_name_ + "/msg/FeedInfo";
        feed_info_pub_ = this->create_publisher<std_msgs::msg::String>(topic_feed_info, 10);

        joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        joint_state_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        run_.store(true);
        thread_joint_ = std::thread(&CRRobotFeedbackNode::joint_loop, this);
        thread_feed_ = std::thread(&CRRobotFeedbackNode::feed_loop, this);

        RCLCPP_INFO(this->get_logger(), "Feedback node started (robot_ip=%s, joint_rate=%.1f Hz, feed_rate=%.1f Hz)",
                    robot_ip_.c_str(), joint_rate_hz_, feed_rate_hz_);
    }

    ~CRRobotFeedbackNode() override
    {
        run_.store(false);
        if (thread_joint_.joinable())
            thread_joint_.join();
        if (thread_feed_.joinable())
            thread_feed_.join();
    }

private:
    void joint_loop()
    {
        // Defensive: ensure the worker thread has FIFO (in case the parent didn't).
        const bool sched_fifo = env_flag_with_fallback(
            "ANYROB_FEEDBACK_SCHED_FIFO",
            "ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO",
            false);
        const int prio = env_int_with_fallback(
            "ANYROB_FEEDBACK_SCHED_FIFO_PRIORITY",
            "ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO_PRIORITY",
            99);
        if (sched_fifo)
        {
            (void)try_enable_sched_fifo(prio);
        }

        rclcpp::Rate rate(joint_rate_hz_);
        while (rclcpp::ok() && run_.load())
        {
            double joints[6] = {0, 0, 0, 0, 0, 0};
            commander_->getCurrentJointStatus(joints);

            joint_state_msg_.header.stamp = this->get_clock()->now();
            joint_state_msg_.header.frame_id = "dummy_link";
            for (int i = 0; i < 6; i++)
            {
                joint_state_msg_.position[static_cast<size_t>(i)] = joints[i];
            }
            joint_state_pub_->publish(joint_state_msg_);

            double val[6] = {0, 0, 0, 0, 0, 0};
            commander_->getToolVectorActual(val);
            dobot_msgs_v4::msg::ToolVectorActual tool_vector_actual_msg;
            tool_vector_actual_msg.x = val[0];
            tool_vector_actual_msg.y = val[1];
            tool_vector_actual_msg.z = val[2];
            tool_vector_actual_msg.rx = val[3];
            tool_vector_actual_msg.ry = val[4];
            tool_vector_actual_msg.rz = val[5];
            tool_vector_pub_->publish(tool_vector_actual_msg);

            dobot_msgs_v4::msg::RobotStatus robot_status_msg;
            robot_status_msg.is_enable = commander_->isEnable();
            robot_status_msg.is_connected = commander_->isConnected();
            robot_status_pub_->publish(robot_status_msg);

            rate.sleep();
        }
    }

    void feed_loop()
    {
        // Defensive: ensure the worker thread has FIFO (in case the parent didn't).
        const bool sched_fifo = env_flag_with_fallback(
            "ANYROB_FEEDBACK_SCHED_FIFO",
            "ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO",
            false);
        const int prio = env_int_with_fallback(
            "ANYROB_FEEDBACK_SCHED_FIFO_PRIORITY",
            "ANYROB_ACTION_MOVE_SERVER_SCHED_FIFO_PRIORITY",
            99);
        if (sched_fifo)
        {
            (void)try_enable_sched_fifo(prio);
        }

        rclcpp::Rate rate(feed_rate_hz_);
        while (rclcpp::ok() && run_.load())
        {
            const RealTimeData rt = commander_->getRealDataSnapshot();
            const std::string payload = realtime_to_json(rt).dump();

            std_msgs::msg::String msg;
            msg.data = payload;
            feed_info_pub_->publish(msg);

            rate.sleep();
        }
    }

private:
    std::string robot_ip_;
    std::string robot_node_name_{"dobot_bringup_ros2"};
    int robot_number_{1};

    double joint_rate_hz_{10.0};
    double feed_rate_hz_{100.0};

    std::shared_ptr<CRCommanderRos2> commander_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<dobot_msgs_v4::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<dobot_msgs_v4::msg::ToolVectorActual>::SharedPtr tool_vector_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feed_info_pub_;

    sensor_msgs::msg::JointState joint_state_msg_;

    std::atomic_bool run_{false};
    std::thread thread_joint_;
    std::thread thread_feed_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CRRobotFeedbackNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

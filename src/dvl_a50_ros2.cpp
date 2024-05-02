#include <string>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <cmath>
#include <map>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <marine_acoustic_msgs/msg/dvl.hpp>

#include "dvl_a50/dvl_a50.hpp"


using namespace dvl_a50;


class DvlA50Node : public rclcpp_lifecycle::LifecycleNode
{
public:
    DvlA50Node(std::string name)
    : rclcpp_lifecycle::LifecycleNode(name)
    {
        this->declare_parameter<std::string>("ip_address", "192.168.194.95");
        this->declare_parameter<std::string>("frame", "dvl_a50_link");
        this->declare_parameter<double>("rate", 30.0);
        this->declare_parameter<int>("speed_of_sound", 1500);
        this->declare_parameter<bool>("enable_on_activate", true);
        this->declare_parameter<bool>("enable_led", true);
        this->declare_parameter<int>("mountig_rotation_offset", 0);
        this->declare_parameter<std::string>("range_mode", "auto");
    }

    ~DvlA50Node()
    {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state)
    {
        // Parameters
        ip_address = this->get_parameter("ip_address").as_string();
        frame = this->get_parameter("frame").as_string();
        rate = this->get_parameter("rate").as_double();
        RCLCPP_INFO(get_logger(), "Connecting to DVL A50 at %s", ip_address.c_str());

        int success = dvl.connect(ip_address, false);
        if (success != 0)
        {
            RCLCPP_ERROR(get_logger(), "Connection failed with error code %i", success);
            return CallbackReturn::FAILURE;
        }

        // Configure
        speed_of_sound = this->get_parameter("speed_of_sound").as_int();
        enable_on_activate = this->get_parameter("enable_on_activate").as_bool();
        bool led_enabled = this->get_parameter("led_enabled").as_bool();
        int mountig_rotation_offset = this->get_parameter("mountig_rotation_offset").as_int();
        std::string range_mode = this->get_parameter("range_mode").as_string();
        
        dvl.configure(speed_of_sound, false, led_enabled, mountig_rotation_offset, range_mode);
        
         //Publishers
        velocity_pub = this->create_publisher<marine_acoustic_msgs::msg::Dvl>("dvl/velocity", 10);
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("dvl/position", 10);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& state)
    {
        LifecycleNode::on_activate(state);

        if (enable_on_activate)
        {
            dvl.set_acoustic_enabled(true);
        }

        velocity_pub->on_activate();
        odom_pub->on_activate();
        
        // Services
        enable_srv = this->create_service<std_srvs::srv::Trigger>(
            "enable", 
            bind(&DvlA50Node::srv_send_param<bool>, this, "acoustic_enabled", true, std::placeholders::_1, std::placeholders::_2));

        disable_srv = this->create_service<std_srvs::srv::Trigger>(
            "disable", 
            bind(&DvlA50Node::srv_send_param<bool>, this, "acoustic_enabled", false, std::placeholders::_1, std::placeholders::_2));

        get_config_srv = this->create_service<std_srvs::srv::Trigger>(
            "get_config", 
            bind(&DvlA50Node::srv_send_command, this, "get_config", std::placeholders::_1, std::placeholders::_2));

        calibrate_gyro_srv = this->create_service<std_srvs::srv::Trigger>(
            "calibrate_gyro", 
            bind(&DvlA50Node::srv_send_command, this, "calibrate_gyro", std::placeholders::_1, std::placeholders::_2));

        reset_dead_reckoning_srv = this->create_service<std_srvs::srv::Trigger>(
            "reset_dead_reckoning", 
            bind(&DvlA50Node::srv_send_command, this, "reset_dead_reckoning", std::placeholders::_1, std::placeholders::_2));

        trigger_ping_srv = this->create_service<std_srvs::srv::Trigger>(
            "trigger_ping", 
            bind(&DvlA50Node::srv_send_command, this, "trigger_ping", std::placeholders::_1, std::placeholders::_2));

        // Start reading data
        RCLCPP_INFO(get_logger(), "Starting to receive reports at <= %f Hz", rate);
        timer = this->create_wall_timer(
            std::chrono::duration<double>(1. / rate), 
            std::bind(&DvlA50Node::publish, this)
        );

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state)
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "Stopping report reception");

        // Stop reading data
        dvl.set_acoustic_enabled(false);
        timer->cancel();

        velocity_pub->on_deactivate();
        odom_pub->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state)
    {
        dvl.disconnect();
        timer.reset();

        velocity_pub.reset();
        odom_pub.reset();

        get_config_srv.reset();
        calibrate_gyro_srv.reset();
        reset_dead_reckoning_srv.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state)
    {
        return CallbackReturn::SUCCESS;
    }


    void publish()
    {
        DvlA50::Message res = dvl.receive();

        if (res.contains("response_to"))
        {
            // Command response
            std::string trigger = res["response_to"];

            // Always print the result
            if (res["success"])
            {
                RCLCPP_INFO(get_logger(), "%s: success", trigger.c_str());
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "%s failed: %s", trigger.c_str(), res["error_message"].dump().c_str());
            }

            if (trigger == "get_config")
            {
                RCLCPP_INFO(get_logger(), "get_config: %s", res["result"].dump().c_str());
            }

            // Check if we have a pending service call for this command and release it
            auto pending_it = pending_service_calls.find(trigger);
            if (pending_it != pending_service_calls.end())
            {
                pending_it->second.set_value(res);
                pending_service_calls.erase(pending_it);
            }
        }
        else if(res.contains("altitude"))
        {
            // Velocity report
            marine_acoustic_msgs::msg::Dvl msg;
            
            msg.header.frame_id = frame;
            msg.header.stamp = rclcpp::Time(uint64_t(res["time_of_validity"]) * 1000);

            msg.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
            msg.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;
            
            msg.velocity.x = double(res["vx"]);
            msg.velocity.y = double(res["vy"]);
            msg.velocity.z = double(res["vz"]);
            
            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    double val = double(res["covariance"][i][j]);
                    msg.velocity_covar[i*3 + j] = val;
                }
            }

            double current_altitude = double(res["altitude"]);
            if(current_altitude >= 0.0 && res["velocity_valid"])
                old_altitude = msg.altitude = current_altitude;
            else
                msg.altitude = old_altitude;

            msg.course_gnd = std::atan2(msg.velocity.y, msg.velocity.x);
            msg.speed_gnd = std::sqrt(msg.velocity.x * msg.velocity.x + msg.velocity.y * msg.velocity.y);

            msg.sound_speed = speed_of_sound;
            msg.beam_ranges_valid = true;
            msg.beam_velocities_valid = res["velocity_valid"];

            // Beam specific data
            for (size_t beam = 0; beam < 4; beam++)
            {
                msg.num_good_beams += bool(res["transducers"][beam]["beam_valid"]);
                msg.range = res["transducers"][beam]["distance"];
                //msg.range_covar
                msg.beam_quality = res["transducers"][beam]["rssi"];
                msg.beam_velocity = res["transducers"][beam]["velocity"];
                //msg.beam_velocity_covar
            }

            /*
             * Beams point 22.5° away from center, LED pointing forward
             * Transducers rotated 45° around Z
             */
            // Beam 1 (+135° from X)
            msg.beam_unit_vec[0].x = -0.6532814824381883;
            msg.beam_unit_vec[0].y =  0.6532814824381883;
            msg.beam_unit_vec[0].z =  0.38268343236508984;

            // Beam 2 (-135° from X)
            msg.beam_unit_vec[1].x = -0.6532814824381883;
            msg.beam_unit_vec[1].y = -0.6532814824381883;
            msg.beam_unit_vec[1].z =  0.38268343236508984;

            // Beam 3 (-45° from X)
            msg.beam_unit_vec[2].x =  0.6532814824381883;
            msg.beam_unit_vec[2].y = -0.6532814824381883;
            msg.beam_unit_vec[2].z =  0.38268343236508984;

            // Beam 4 (+45° from X)
            msg.beam_unit_vec[3].x =  0.6532814824381883;
            msg.beam_unit_vec[3].y =  0.6532814824381883;
            msg.beam_unit_vec[3].z =  0.38268343236508984;

            velocity_pub->publish(msg);
        }
        else if (res.contains("pitch"))
        {
            // Dead reckoning report
            nav_msgs::msg::Odometry msg;
            
            msg.header.frame_id = frame;
            msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(double(res["ts"])) * 1e9);

            msg.pose.pose.position.x = double(res["x"]);
            msg.pose.pose.position.y = double(res["y"]);
            msg.pose.pose.position.z = double(res["z"]);

            double std_dev = double(res["std"]);
            msg.pose.covariance[0] = std_dev;
            msg.pose.covariance[7] = std_dev;
            msg.pose.covariance[14] = std_dev;

            tf2::Quaternion quat;
            quat.setRPY(double(res["roll"]), double(res["pitch"]), double(res["yaw"]));
            msg.pose.pose.orientation = tf2::toMsg(quat);

            odom_pub->publish(msg);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Received unexpected DVL response: %s", res.dump().c_str());
        }
    }


    void srv_send_command(
        std::string command,
        std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        std::promise<DvlA50::Message> promise;
        std::future<DvlA50::Message> future = promise.get_future();
        pending_service_calls.insert(std::make_pair(command, std::move(promise)));
        dvl.send_command(command);

        DvlA50::Message json_data = future.get();
        res->success = json_data["success"];
        res->message = json_data["error_message"];
    }

    template<typename T>
    void srv_send_param(
        std::string param,
        const T& value,
        std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        std::promise<DvlA50::Message> promise;
        std::future<DvlA50::Message> future = promise.get_future();
        pending_service_calls.insert(std::make_pair("set_config", std::move(promise)));
        dvl.set(param, value);

        DvlA50::Message json_data = future.get();
        res->success = json_data["success"];
        res->message = json_data["error_message"];
    }


private:
    DvlA50 dvl;

    std::string ip_address;
    std::string frame;
    double rate;
    int speed_of_sound;
    bool enable_on_activate;
    double old_altitude;

    // Promises of unfulfilled service calls; assumes that no service is called twice in parallel
    std::map<std::string, std::promise<DvlA50::Message>> pending_service_calls;
    
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr velocity_pub;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_config_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_gyro_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_dead_reckoning_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_ping_srv;
};


int main(int argc, char * argv[])
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<DvlA50Node> node = std::make_shared<DvlA50Node>("dvl_a50");
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "k2_action/action/wire_feed.hpp"
#include "k2_action/msg/wire_feed_status.hpp"
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using WireFeed = k2_action::action::WireFeed;

class WireFeedSimServer : public rclcpp::Node {
public:
    WireFeedSimServer() : Node("teknic_node_sim") {
        RCLCPP_INFO(this->get_logger(), "Starting Simulated Wire Feed Action Server...");

        distance_server_ = rclcpp_action::create_server<WireFeed>(
            this,
            "/wire_feed_commands",
            std::bind(&WireFeedSimServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&WireFeedSimServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&WireFeedSimServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Action server is running! Attempting to boot the feedback publisher");

        status_pub_ = this->create_publisher<k2_action::msg::WireFeedStatus>("motion_feedback", 10);
        // create the safety status publisher timer with callback
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WireFeedSimServer::publish_status, this)
        );
        RCLCPP_INFO(this->get_logger(), "Wire feed publisher is also active. All systems go.");
    }

private:
    rclcpp_action::Server<WireFeed>::SharedPtr distance_server_;

    double torque_value_ = 0.0;
    bool torque_increasing_ = true;


    // create the status publisher and timer
	rclcpp::Publisher<k2_action::msg::WireFeedStatus>::SharedPtr status_pub_;
	rclcpp::TimerBase::SharedPtr pub_timer_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const WireFeed::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received simulated goal: axis=%d, motion_type=%s, distance=%.2f, velocity=%.2f",
                    goal->axis_number, goal->motion_type.c_str(), goal->distance, goal->velocity);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> /*goal_handle*/) {
        RCLCPP_INFO(this->get_logger(), "Simulated goal canceled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle) {
        std::thread{std::bind(&WireFeedSimServer::execute_goal, this, goal_handle)}.detach();
    }

    void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle) {
        auto result = std::make_shared<WireFeed::Result>();
        auto feedback = std::make_shared<WireFeed::Feedback>();
        auto goal = goal_handle->get_goal();

        RCLCPP_INFO(this->get_logger(), "Simulating execution...");

        if (goal->motion_type == "position") {
            double distance = goal->distance;
            double progress = 0.0;

            for (int i = 0; i <= 10; ++i) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->message = "Simulated goal canceled.";
                    goal_handle->canceled(result);
                    return;
                }

                progress = distance * i / 10.0;
                feedback->current_distance = progress;
                feedback->percent = (progress / distance) * 100.0;
                feedback->torque_feedback = 0.1;  // arbitrary value
                //goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(100ms);
            }

            result->success = true;
            result->message = "Simulated position move complete.";
            goal_handle->succeed(result);

        } else if (goal->motion_type == "velocity") {
            // For a velocity move
            feedback->current_distance = 0.0;

            while (!goal_handle->is_canceling()){
                // run a continuous velocity move until the move is cancelled
                feedback->current_distance += goal->velocity * 0.1;  // simplistic approximation
                feedback->percent = 0.0;
                feedback->torque_feedback = 0.05;
                goal_handle->publish_feedback(feedback);

                static int counter = 0;
                if (++counter % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Simulated feed at distance: %.2f", feedback->current_distance);
                }

                if (!rclcpp::ok()) {
                    result->success = false;
                    result->message = "ROS shutdown detected.";
                    goal_handle->abort(result);
                    return;
                }

                std::this_thread::sleep_for(100ms);
            }
            result->success = true;
            result->message = "Simulated velocity move canceled.";
            goal_handle->canceled(result);
            return;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown motion type: %s", goal->motion_type.c_str());
            result->success = false;
            result->message = "Unknown motion type.";
            goal_handle->abort(result);
        }
    }

    void publish_status() {
        for (size_t iPort = 0; iPort < 1; iPort++) {
            // loop through the ports
            for (unsigned iNode = 0; iNode < 4; iNode++) {
                k2_action::msg::WireFeedStatus msg;
                msg.axis_number = iNode;
                msg.position_reading = iNode * iPort;
                msg.velocity_reading = iNode * iPort;
                msg.torque_reading = torque_value_;

                status_pub_->publish(msg);
            }

            // After loop, update torque for next time
            if (torque_increasing_) {
                torque_value_ += 1.0;
                if (torque_value_ >= 100.0) {
                    torque_increasing_ = false;
                }
            } else {
                torque_value_ -= 1.0;
                if (torque_value_ <= 0.0) {
                    torque_increasing_ = true;
                }
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WireFeedSimServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

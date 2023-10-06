#include "geometry_msgs/msg/twist.hpp"
#include "mutex"
#include "nav_msgs/msg/odometry.hpp"
#include "queue"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <array>
#include <cmath>
#include <queue>
#include <string>

using namespace std::chrono_literals;
class MazeSolver : public rclcpp::Node {
public:
  MazeSolver(const std::string &name,
             const std::queue<std::array<double, 2>> &traj)
      : Node(name), trajectory(traj) {

    rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_DEBUG);

    call_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = call_group_;

    // velocity publisher
    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // odometry subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 50,
        std::bind(&MazeSolver::odom_callback, this, std::placeholders::_1),
        opt);

    Kpt = 0.5;
    Kit = 0.04;
    Kdt = 0.2;
    sum_alpha = 0.0;

    Kpf = 0.5;
    Kif = 0.01;
    Kdf = 0.22;
    sum_rho = 0.0;

    dt = 0.005;
    vel = MAX_ANG_VELOCITY;
    is_oriented = false;
    is_forward = false;

    // controller loop
    controller_loop_ = this->create_wall_timer(
        5ms, std::bind(&MazeSolver::controller_callback, this), call_group_);
  }

private:
  std::queue<std::array<double, 2>> trajectory;
  std::array<double, 6> current_waypoint = {0, 0, 0, 0, 0, 0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::CallbackGroup::SharedPtr call_group_;
  rclcpp::TimerBase::SharedPtr controller_loop_;
  std::mutex pose_mutex;
  geometry_msgs::msg::Twist vel_msg;

  double Kpt;
  double Kdt;
  double Kit;
  double Kpf;
  double Kdf;
  double Kif;
  double sum_rho;
  double sum_alpha;
  double theta;
  double vel;
  double dt;
  const double MAX_VELOCITY = 0.9;
  const double MAX_ANG_VELOCITY = 3.0;
  bool is_oriented;
  bool is_forward;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    current_waypoint[0] = msg->pose.pose.position.x;
    current_waypoint[1] = msg->pose.pose.position.y;
    current_waypoint[2] = msg->pose.pose.orientation.x;
    current_waypoint[3] = msg->pose.pose.orientation.y;
    current_waypoint[4] = msg->pose.pose.orientation.z;
    current_waypoint[5] = msg->pose.pose.orientation.w;

    tf2::Quaternion orientation(current_waypoint[2], current_waypoint[3],
                                current_waypoint[4], current_waypoint[5]);

    tf2::Matrix3x3 mat(orientation);
    double r;
    double p;
    mat.getRPY(r, p, theta);
    pose_mutex.unlock();
    RCLCPP_DEBUG(this->get_logger(), "Received current position");
  }

  void controller_callback() {
    std::lock_guard<std::mutex> lock(pose_mutex);

    if (trajectory.empty()) {
      RCLCPP_INFO(
          this->get_logger(),
          "Complete trajectory. Stopping robot. Final position: X:%f, Y:%f",
          current_waypoint[0], current_waypoint[1]);
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      vel_pub_->publish(vel_msg);
      controller_loop_->cancel();
      return;
    }

    if (is_oriented == true && is_forward == true) {
      trajectory.pop();
      is_oriented = false;
      is_forward = false;
      RCLCPP_INFO(this->get_logger(), "next goal position: %f, %f",
                  trajectory.front()[0], trajectory.front()[1]);
      // Orient goal
    } else if (is_oriented == false) {
      double alpha = update_alpha();
      double alpha_dt = -vel; // turning only, no moving forward
      RCLCPP_DEBUG(this->get_logger(), "alpha: %f", alpha);

      if (abs(alpha) <= 0.01) {
        //~~~~~~~~~~~~~~~~~~~~~~Moving the robot
        // forward~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        is_oriented = true;
        reset_ang_vel();
        vel = MAX_VELOCITY;
        RCLCPP_INFO(
            this->get_logger(),
            "Oriented to this goal position. Move forward to this goal.");
      } else {

        sum_alpha += alpha * dt;
        vel = Kpt * alpha + Kdt * alpha_dt + Kit * sum_alpha;
        vel_msg.angular.z = static_cast<float>(vel);
        vel_pub_->publish(vel_msg);
      }
      // Move forward
    } else if (is_oriented == true && is_forward == false) {
      double rho = update_rho();
      double rho_d = -vel;
      RCLCPP_DEBUG(this->get_logger(), "rho: %f", rho);
      if (rho <= 0.02) {

        is_forward = true;
        reset_lin_vel();
        vel = MAX_ANG_VELOCITY;
        RCLCPP_INFO(
            this->get_logger(),
            "Reached this goal position. Orienting to the next goal position.");
      } else {

        sum_rho += rho * dt;
        vel = Kpf * rho + Kdf * rho_d + Kif * sum_rho;
        vel_msg.linear.x = static_cast<float>(vel);
        vel_pub_->publish(vel_msg);
      }
    }

    pose_mutex.unlock();
  }

  double update_alpha() {
    // get goal
    auto goal_position = trajectory.front();
    double delta_x = goal_position[0] - current_waypoint[0];
    double delta_y = goal_position[1] - current_waypoint[1];
    double angular_error = atan2(delta_y, delta_x) - theta;
    // RCLCPP_DEBUG(this->get_logger(), "Received goal position, alpha: %f",
    //              angular_error);
    return angular_error;
  }

  double update_rho() {
    // get goal
    auto goal_position = trajectory.front();
    double delta_x = goal_position[0] - current_waypoint[0];
    double delta_y = goal_position[1] - current_waypoint[1];
    double distance_error = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    // RCLCPP_DEBUG(this->get_logger(), "Received goal position, rho: %f",
    //              distance_error);
    return distance_error;
  }

  void reset_ang_vel() {
    sum_alpha = 0.0;
    vel_msg.angular.z = 0.0;
    for (int i = 0; i < 5; i++) {
      vel_pub_->publish(vel_msg);
    }
  }

  void reset_lin_vel() {
    sum_rho = 0.0;
    vel_msg.linear.x = 0.0;
    for (int i = 0; i < 5; i++) {
      vel_pub_->publish(vel_msg);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::string node_name = "pid_maze_solver_node";

  // std::array<double, 2> sp1 = {0.0, 0.0}; // pos: x,y, orientation: x,y,z,w
  std::array<double, 2> sp2 = {0.50, -0.10};
  std::array<double, 2> sp3 = {0.50, -1.36};
  std::array<double, 2> sp4 = {1.03, -1.37};
  std::array<double, 2> sp5 = {1.04, -0.83};
  std::array<double, 2> sp6 = {1.42, -0.84};
  std::array<double, 2> sp7 = {1.43, -0.29};
  std::array<double, 2> sp8 = {1.95, -0.3};
  std::array<double, 2> sp9 = {1.95, 0.58};
  std::array<double, 2> sp10 = {1.52, 0.58};
  std::array<double, 2> sp11 = {1.48, 0.21};
  std::array<double, 2> sp12 = {0.98, 0.19};
  std::array<double, 2> sp13 = {0.69, 0.50};
  std::array<double, 2> sp14 = {0.19, 0.53};

  std::queue<std::array<double, 2>> trajectory;
  trajectory.push(sp2);
  trajectory.push(sp3);
  trajectory.push(sp4);
  trajectory.push(sp5);
  trajectory.push(sp6);
  trajectory.push(sp7);
  trajectory.push(sp8);
  trajectory.push(sp9);
  trajectory.push(sp10);
  trajectory.push(sp11);
  trajectory.push(sp12);
  trajectory.push(sp13);
  trajectory.push(sp14);

  auto pid_maze_solver_node =
      std::make_shared<MazeSolver>(node_name, trajectory);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pid_maze_solver_node);
  RCLCPP_INFO(pid_maze_solver_node->get_logger(), "Moving toward waypoints");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
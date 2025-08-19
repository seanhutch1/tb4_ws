#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/drive_arc.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TB4ArcActionServer : public rclcpp::Node
{
public:
  using Drive_Arc= irobot_create_msgs::action::DriveArc;

  explicit TB4ArcActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :Node("tb4_arc_action_server", options)
  {
    /*MILESTONE #2.2 Initialise the command velocity publisher share pointer*/
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
		using namespace std::placeholders;

    /*MILESTONE #2.3 Initialise the odometry subsriber share pointer, and bing the call back function
      "odom_callback"
    */
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(	"odom", 
																																		rclcpp::SensorDataQoS(), 
																																		std::bind(&TB4ArcActionServer::odom_callback, 
																																							this, 
																																							std::placeholders::_1)
    );

    /*MILESTONE #2.4
      Initialsie the drive arc action server with name as "drive_arc_prac2", and bind call back functions for
      handling of accepting a goal, cancelling a action, and process the accepted goal
    */
    this->action_server_ = rclcpp_action::create_server<Drive_Arc>(
      this,
      "drive_arc_prac2",
      std::bind(&TB4ArcActionServer::handle_goal, this, _1, _2),
      std::bind(&TB4ArcActionServer::handle_cancel, this, _1),
      std::bind(&TB4ArcActionServer::handle_accepted, this, _1)
    );

  RCLCPP_INFO(this->get_logger(), "TB4 Arc Action Server Initialized.");

  }
private:
  /* MILESTONE #2.1
  Define shared pointers for 
    - action server for drive arc defined in irobot_create_msgs, 
    - command velocity publisher
    - odometry subscriber
  */
 rclcpp_action::Server<Drive_Arc>::SharedPtr action_server_;
 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


  // odometry pointer
  nav_msgs::msg::Odometry::SharedPtr odom_;
  // odometry subscriber callback
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  // Callback function for handling goals
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal
  );

  // Callback function for handling cancellation:
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  // Action processing and update
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);
};


void TB4ArcActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  /* MILESTONE #3.1
    save the odom_msg to the class member variable odom_   
  */
 odom_ = odom_msg;
 RCLCPP_INFO(this->get_logger(), "Received odometry");
}

/*i MILESTONE #4.1
  complete the  call back function of "TB4ArcActionServer::handle_accepted" that handling the accepted goal 
*/
void TB4ArcActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  std::thread{std::bind(&TB4ArcActionServer::execute, this, goal_handle)}.detach();
  
}
/* MILESTONE #4.2
  complete the  call back function of "TB4ArcActionServer::handle_cancel" that cancel the goal 
  */
rclcpp_action::CancelResponse TB4ArcActionServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request.");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;

}
/* MILESTONE #4.3
  complete the  call back function of "TB4ArcActionServer::handle_goal" that accept goal, 
  you should also print the goal details in the terminal 
*/
rclcpp_action::GoalResponse TB4ArcActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal
)
{
  RCLCPP_INFO(this->get_logger(),
    "Received goal: direction=%d, angle=%.2f rad, radius=%.2f m, speed=%.2f m/s",
    goal->translate_direction, goal->angle, goal->radius, goal->max_translation_speed);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  
}
/* MILESTONE #5.1 ~ #5.3
  complete the  thread function "execute" to proccess the goal in the action request
*/
void TB4ArcActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Drive_Arc::Feedback>();
  auto result = std::make_shared<Drive_Arc::Result>();

  // Calculate angular velocity: ω = v / r
  double angular_speed = goal->max_translation_speed / goal->radius;

  // Adjust direction (clockwise/counterclockwise)
  if (goal->translate_direction < 0)
    angular_speed = -angular_speed;

  // Total angle to travel
  double angle_to_travel = goal->angle;

  // Variables to track progress
  double angle_traveled = 0.0;

  // Create Twist message for velocity commands
  geometry_msgs::msg::Twist cmd_vel_msg;

  // Set linear speed (x) and angular speed (z)
  cmd_vel_msg.linear.x = goal->max_translation_speed;
  cmd_vel_msg.angular.z = angular_speed;

  rclcpp::Rate rate(10); // 10 Hz loop rate

  auto start_time = this->now();

  while (rclcpp::ok() && angle_traveled < angle_to_travel)
  {
    // Check for cancel request
    if (goal_handle->is_canceling())
    {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);

      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // Publish velocity commands
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Calculate elapsed time and update angle traveled
    auto now = this->now();
    double elapsed_seconds = (now - start_time).seconds();

    angle_traveled = std::min(elapsed_seconds * fabs(angular_speed), angle_to_travel);

    // Publish feedback
    feedback->remaining_angle_travel = angle_to_travel - angle_traveled;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  // Stop the robot when done
  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel_msg);

  // Fill result pose with latest odometry
  if (odom_)
  {
    result->pose.header = odom_->header;
    result->pose.pose = odom_->pose.pose;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Odometry data not available!");
  }

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TB4ArcActionServer>());
	rclcpp::shutdown();
	return 0;
}

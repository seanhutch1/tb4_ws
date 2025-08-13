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
    /*TODO  TASK - MILESTONE #2.2 Initialise the command velocity publisher share pointer*/

    // Publisher of topic /cmd_vel
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::SystemDefaultsQoS());
    using namespace std::placeholders;

    /*TODO  TASK - MILESTONE #2.3 Initialise the odometry subscriber share pointer, and bing the call back function
      "odom_callback" */

    // Subscribe to the /odom topic
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&TB4ArcActionServer::odom_callback, this, std::placeholders::_1)
    );

    /*TODO  TASK - MILESTONE #2.4
      Initialsie the drive arc action server with name as "drive_arc_prac2", and bind call back functions for
      handling of accepting a goal, cancelling a action, and process the accepted goal */

    // Action server with name as "drive_arc_prac2", and bind callback functions
    this->action_server_ = rclcpp_action::create_server<Drive_Arc>(
      this,
      "drive_arc_prac2",
      std::bind(&TB4ArcActionServer::handle_goal, this, _1, _2),
      std::bind(&TB4ArcActionServer::handle_cancel, this, _1),
      std::bind(&TB4ArcActionServer::handle_accepted, this, _1));

  }
private:
  /* TODO TASK - MILESTONE #2.1
  Define shared pointers for 
    - action server for drive arc defined in irobot_create_msgs, 
    - command velocity publisher
    - odometry subscriber
  */

  /// from prac 1 action server node:
  /// creates smart pointers (memory is managed automatically) to store them as class members so they dont get lost once constructor finsihes.
   // Define Drive Arc server
  rclcpp_action::Server<Drive_Arc>::SharedPtr action_server_;

   // Define a command velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

   // Define Odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;











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
  /*TODO TASK - MILESTONE #3.1
    save the odom_msg to the class member variable odom_ */

  /// 'odom_callback' callback function runs when receiving the odometry message
  odom_ = odom_msg; /// saves the received msg into a class member for later use in execute() function

}



/*TODO TASK - MILESTONE #4.1
  complete the  call back function of "TB4ArcActionServer::handle_accepted" that handling the accepted goal 
*/
void TB4ArcActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  using namespace std::placeholders;
  // From prac 1: this needs to return quickly to avoid blocking the executor, so spin up a new thread

  /// makes a new thread and calls execute() function to run at the same time (execute(goal_handle)) goal hnadle is a param when calling execute
  std::thread{std::bind(&TB4ArcActionServer::execute, this, _1), goal_handle}.detach();
}



/* TODO TASK - MILESTONE #4.2
  complete the  call back function of "TB4ArcActionServer::handle_cancel" that cancel the goal 
  */
rclcpp_action::CancelResponse TB4ArcActionServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}



/*TODO TASK - MILESTONE #4.3
  complete the  call back function of "TB4ArcActionServer::handle_goal" that accept goal, 
  you should also print the goal details in the terminal 
*/
rclcpp_action::GoalResponse TB4ArcActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), 
    "Received goal request with: \n translate_direction = %d ,\n angle = %f ,\n radius = %f ,\n max_translation_speed = %f ", 
    goal->translate_direction,     // int8 %d
    goal->angle,                   // float
    goal->radius,                  // float
    goal->max_translation_speed);  // float
  (void)uuid;
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}







// 2:00

/* TODO TASKS - MILESTONE #5.1 ~ #5.3
  complete the  thread function "execute" to proccess the goal in the action request
*/
void TB4ArcActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{

  RCLCPP_INFO(this->get_logger(), "Executing goal"); /// print


  /// 5.1 define goal, feedback and result
  const auto goal = goal_handle->get_goal(); /// get the goal message sent from client

  int translate_direction   = goal->translate_direction;    /// Whether to arc forward or backward from robot’s current position
  float target_angle        = goal->angle;                  /// Relative angle (radians) for robot to rotate along arc from current heading. Angles greater than 2 PI will cause the robot to rotate in multiple circles
  float turning_radius      = goal->radius;                 /// Radius of arc (meters) for robot to drive along
  float max_speed           = goal->max_translation_speed;  /// Max translation speed (positive m/s), will cap negative distance to negative speed

  
  auto feedback = std::make_shared<Drive_Arc::Feedback>(); /// Feedback msg is created here
  auto & remaining_angle_travel = feedback->remaining_angle_travel; /// & creates alias, same object so i can use auto

  auto result = std::make_shared<Drive_Arc::Result>(); /// creates a result msg to send once goal finished or cancled // robots final pos
  
  




  // 5.2 - validate goal


  /// todo
  


  /// 5.3 

  const int dir; if (translate_direction >= 0) {dir = 1} else {dir = -1};


  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.set__x(goal->max_translation_speed); /// set speed from goal msg object member

  float angular_velocity = dir * (max_speed / turning_radius);

  cmd_vel.angular.set__z(angular_velocity); /// angular speed based on formula and turning direction


  int pub_freq = 100;
  rclcpp::Rate loop_rate(pub_freq); /// publishes 100 times per second

  int count = int(pub_freq*target_angle/angular_velocity);
  /// this finds the number of loop iterations needed to turn based on the set angle from the goal ms, depending on angular vel

  geometry_msgs::msg::PoseStamped pose_stamped; /// saves current pos before movement

  for (int i = 0; (i<count) && rclcpp::ok(); ++i) 
  {

    pose_stamped.header = odom_->header; /// saves current odometry each cycle of loop
    pose_stamped.pose = odom_->pose.pose; /// 

    if (goal_handle->is_canceling()) 
      {
        result->set__pose(pose_stamped);
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        // cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{}); // puiblish 0 to stop robot
        return; /// if canceled then save result then exit
      }


  
    // Publish the command velocity
    cmd_vel_publisher_->publish(cmd_vel);

    // Publish feedback
    /// angle currently travelled -  goal angle to find remaining  
    feedback->remaining_angle_travel = target_angle - angular_velocity*i/pub_freq;
    goal_handle->publish_feedback(feedback);
  
    loop_rate.sleep(); /// to keep loop frequency

  }

  // // after main movement loop, publish 0 to stop robot
  // cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});



  // Check if goal is done
  if(rclcpp::ok())
  {
    result->set__pose(pose_stamped);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }



}











int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TB4ArcActionServer>());
	rclcpp::shutdown();
	return 0;
}
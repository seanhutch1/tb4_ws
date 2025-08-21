#include <chrono>  
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358
using rcl_interfaces::msg::ParameterType;


class WallFollower : public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        /*TODO TASK - MILESTONE # 4.1
            1. Declare all parameters used for configuring the "following distance", "following angle", and all control gains. Their default values should be given as well.
            2. Get all parameter values from the constructor, and save them to private class element variables.
            3. Print all parameter values here.
            4. Set the value of "following_angle_" after initialising all parameters
        */

        //declare params
        this->declare_parameter<double>("following_angle", 0.0);
        this->declare_parameter<double>("following_distance", 1.0);
        this->declare_parameter<int64_t>("wall_side", 1.0);
        this->declare_parameter<double>("buffer_zone", 1.0);
        this->declare_parameter<double>("forward_velocity", 1.0);
        this->declare_parameter<double>("angle_control_gain_1", 1.0);
        this->declare_parameter<double>("angle_control_gain_2", 1.0);
        this->declare_parameter<double>("distance_control_gain", 0.5);

        // get parameter values
        this->get_parameter("following_distance", following_angle_);
        this->get_parameter("following_angle", following_distance_);
        this->get_parameter("wall_side", wall_side_);
        this->get_parameter("buffer_zone", buffer_zone_);
        this->get_parameter("forward_velocity", forward_velocity_);
        this->get_parameter("angle_control_gain_1", angle_control_gain_1_);
        this->get_parameter("angle_control_gain_2", angle_control_gain_2_);
        this->get_parameter("distance_control_gain", distance_control_gain_);

        // log parameter values
        RCLCPP_INFO(this->get_logger(), "following_angle: %.2f", following_angle_);
        RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
        RCLCPP_INFO(this->get_logger(), "wall_side: %d", wall_side_);
        RCLCPP_INFO(this->get_logger(), "buffer_zone: %.2f", buffer_zone_);
        RCLCPP_INFO(this->get_logger(), "forward_velocity: %.2f", forward_velocity_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_1: %.2f", angle_control_gain_1_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_2: %.2f", angle_control_gain_2_);
        RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);

        /* TODO TASK - MILESTONE #4.3
            Initialise dynamic parameter handler by the rclcpp node method "add_on_set_parameters_callback"
        */
        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(
            &WallFollower::dynamicParametersCallback,
            this, std::placeholders::_1));


        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
             "/cmd_vel",
             rclcpp::SystemDefaultsQoS());
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1)
        );
    }
private:
    std::recursive_mutex mutex_;
    // Define a command velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Define a laser scan subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    /* TODO TASK - MILESTONE #4.2
        define dynamic parameter call back handle.
    */
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    

    /** 
    * @brief Callback executed when a parameter change is detected
    * @param event ParameterEvent message
    */
    rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    double following_angle_;
    double following_distance_;
    int64_t wall_side_;
    double buffer_zone_;
    double forward_velocity_;
    double angle_control_gain_1_;
    double angle_control_gain_2_;
    double distance_control_gain_;

    // constructor for scan callback (left out of template might be a fix)
    // void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
};

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    //TODO TASKS
    //   MILESTONE # 6.1. Process the received scan_msg to get the location of the closest object in robot's environment. 
    //  NOTE: the four pillars of will be visible from the Lidar sensor, you have to remove the distance 
    //  measurements of these four pillars by ignoring any measurement less than 0.2 meter. 


    // following_angle_;
    // following_distance_;
    // wall_side_;
    // buffer_zone_;
    // forward_velocity_;
    // angle_control_gain_1_;
    // angle_control_gain_2_;
    // distance_control_gain_;


    // closest valid object with std::min_element using a comparator that ignores invalid/too-close readings
  
    const float rmin = std::max(0.2f, scan_msg->range_min);  /// clamp range_min to at least 0.2
    const float rmax = scan_msg->range_max;                  /// max range


  auto is_valid = [&](float r)
  {  /// & capture clause to use local varibles by reference /// helper lambda function
    return std::isfinite(r) && r >= rmin && r <= rmax; /// filter out invalids, <0.2,inf,>max
  };

  auto comp = [&](float a, float b) 
  {

    const bool va = is_valid(a); 
    const bool vb = is_valid(b);    

    if (va && vb) return a < b;      /// both valid, return True if a<b, new best found
    if (va && !vb) return true;      /// any valid beats an invalid
    if (!va && vb) return false;     /// no new best
    return false;                    /// no new best
  };

  auto min_distance = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end(), comp);
  /// min_element scans the range and compares currrent to best. 
  /// if custom comparator=True, current beats current best

  /// iterator now points at smallest element in scan_msg. it is a pointer not a value
  /// will point to first value if scan_msg is all invalid

  if (!is_valid(*min_distance)) { /// if min value isnt valid
    RCLCPP_INFO(this->get_logger(), "No Object is Detected - no valid lidar ranges");
    return;
  }



  // Tut3: Extracts the actual minimum value from the iterator obtained in the previous step.
  float min_value = *min_distance; /// iterator --> actual value
  
  // Tut3: Calculates the index of the minimum value in the ranges vector by finding the distance
  //        between the beginning of the vector and the iterator pointing to the minimum value.
  int min_index = std::distance(scan_msg->ranges.begin(), min_distance);
  


  // MILESTONE #3.2. You have to calculate the bearing and the range of the closest object with respect to the robot frame. You have
  // to check the LaserScan message definition, and how the Lidar sensor is mounted with respective to 
  // the robot's coordinate.

  /// From prac3 sheet: angle L = angle min + angle increment × index
  double angle_L = static_cast<double>(scan_msg->angle_min) + static_cast<double>(scan_msg->angle_increment) * static_cast<double>(min_index);

  /// angle_L is now the angle of the closest object in the Lidar sensor frame axis.


  const double bearing_offset = PI / 2.0;
  double bearing_R = angle_L + bearing_offset;

  RCLCPP_INFO(this->get_logger(), " angle_L = %.3f, angle_min = %.3f, angle_increment = %.3f.", angle_L,static_cast<double>(scan_msg->angle_min),static_cast<double>(scan_msg->angle_increment));

  /// robot is 0 facing forward, positive CCW and neg CW
  // eg. lidar angle = 0, robot should read pi/2 (90)
  /// eg. lidar angle = -90, robot should read 0
  /// eg. if lidar angle 180 (pi), robot should read 270 (normalises to -90)

  RCLCPP_INFO(this->get_logger(), " bearing_R = %.3f, (%.1f deg)", bearing_R, bearing_R * 180.0 / PI);
  
  while (bearing_R > PI)  bearing_R -= 2.0 * PI; /// wraps any values to a range of -pi to pi
  while (bearing_R < -PI) bearing_R += 2.0 * PI; /// while statement incase is wrapped multiple times over 2pi

  double range_R = static_cast<double>(min_value); /// distance is the min value calculated from 3.1

  RCLCPP_INFO(this->get_logger(), " bearing_R wrapped = %.3f (%.1f deg), range = %.2f m", bearing_R, bearing_R * 180.0 / PI, range_R);

    /// this gives use bearing and range of closest object
    /// bearing_R
    /// range_R



    //   MILESTONE # 6.2. You have to calculate the bearing and the range of the closest object with respect to the robot frame. You have         
    //   to check the LaserScan message definition, and how the Lidar sensor is mounted with respective to  the robot's coordinate.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////











    //   MILESTONE # 6.3. Write a Wall Follow Reactive Control that takes the bearing and range information of the closest object in the environment 
    //  as the input and publish a message on topic /cmd_vel to control the motion of the robot. 
    //      3.1 If the robot is far away from the wall, it should move towards its nearest wall at a constant speed until the robot 
    //      arrives at a distance of desired value + buffer zone, with respect to its closest wall. 
    //       3.2 Next, the robot enter the wall follow mode with the control lawy in in Algorithm 1
    //      3.3 The robot should deal with corner cases by only using reactive control with properly tuned control gains. 
    //
   

    geometry_msgs::msg::Twist cmd_vel_msg;

    /* 
    The magic number 12 is from the simulation setup, i.e., the range of the lidar sensor is from 0.164 to 12. 
    If min_value<12, the lidar sensor has a valid measurement. 
    */ 

    auto min_angle = bearing_R; ///  min angle??
    if(min_value<12) 
    {
        /* The robot is moving towards to the closed target at speed of forward_velocity_*/
        if(min_value>(following_distance_+buffer_zone_)){
            if(abs(min_angle)>PI/4.0){
                if(min_angle>PI/4.0)
                    cmd_vel_msg.angular.z = 1.0;
                else
                    cmd_vel_msg.angular.z = -1.0;
            }
            else{
                cmd_vel_msg.angular.z = 0;
                cmd_vel_msg.linear.x = forward_velocity_;
            }
        }
        // drive along the wall at a fixed distance
        else{ 
            if(wall_side_>0)
                cmd_vel_msg.angular.z = angle_control_gain_1_*(min_angle - following_angle_) + angle_control_gain_2_*(min_value - following_distance_);
            else
                cmd_vel_msg.angular.z = angle_control_gain_1_*(min_angle - following_angle_) - angle_control_gain_2_*(min_value - following_distance_);
                
            cmd_vel_msg.linear.x = forward_velocity_ + distance_control_gain_*(min_value - following_distance_);
        }
    }
    else // No valid measurement is available, move forward at a constant speed.
    {
        RCLCPP_INFO(this->get_logger(), "No Object is Detected");
        cmd_vel_msg.linear.x = 0.2;
    }
    //publish the command velocity
    cmd_vel_publisher_->publish(cmd_vel_msg);





























}


rcl_interfaces::msg::SetParametersResult 
WallFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    /*TODO TASK - MILESTONE #5.1 
      Check whether update of a parameter in the node is requested, if yes and save the updated
      parameter value.
    */




        // this->declare_parameter<int64_t>("wall_side", 1.0); 
        // this->declare_parameter<double>("buffer_zone", 1.0);
        // this->declare_parameter<double>("forward_velocity", 1.0);





    for (auto parameter : parameters) { /// FOR loop over parameter changes
    const auto & param_type = parameter.get_type(); /// extract type
    const auto & param_name = parameter.get_name(); /// extract name
  
    if (param_type == ParameterType::PARAMETER_DOUBLE)
    {

    if (param_name == "following_distance") {
        following_distance_ = parameter.as_double();
        if(following_distance_<0.0)
        {
          RCLCPP_WARN(this->get_logger(), "You've set following_distance to be negative,"
          " this isn't allowed, so the alpha1 will be set to be zero.");
          following_distance_ = 0.0;
        }
        continue;
    }

          
    if (param_name == "following_angle") {
        following_angle_ = parameter.as_double();
        continue;
    }

    if (param_name == "angle_control_gain_1") {
        angle_control_gain_1_ = parameter.as_double();
        if (angle_control_gain_1_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "angle_control_gain_1 cant be a negative, setting it to 0");
            angle_control_gain_1_ = 0.0;
        }
        continue;
    }

    if (param_name == "angle_control_gain_2") {
        angle_control_gain_2_ = parameter.as_double();
        if (angle_control_gain_2_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "angle_control_gain_2 cant be a negative, setting it to 0");
            angle_control_gain_2_ = 0.0;
        }
        continue;
    }

    if (param_name == "distance_control_gain") {
        distance_control_gain_ = parameter.as_double();
        if (distance_control_gain_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "distance_control_gain cant be negative, setting it to 0");
            distance_control_gain_ = 0.0;
        }
        continue;
    }

    }
  }
  result.successful = true;
  return result;

}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();
	return 0;
}
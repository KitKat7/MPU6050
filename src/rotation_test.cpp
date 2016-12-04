#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  uint8_t last_received_message_number;
  bool received_message = false;
  int data_packet_start;

  tf::Quaternion orientation;
  // tf::Quaternion zero_orientation(-0.00698190182447, 0.0570745244622, -0.799500692636, 0.597906243056);
  tf::Quaternion zero_orientation(0.604959480464, -0.795489951968, -0.0175491012633, 0.0301001854241);

  ros::init(argc, argv, "rotation_test");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
  private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
  private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
  ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
  ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

  ros::Rate r(200); // 200 hz

  sensor_msgs::Imu imu;

  sensor_msgs::Temperature temperature_msg;
  temperature_msg.variance = 0;

  std::string input;
  std::string read;

  while(ros::ok())
  {

    // double wf = 0.981054872274;
    // double xf = -0.0019547380507;
    // double yf = 0.0604222454131;
    // double zf = -0.184004992247;

    double wf = 0.99107632041;
    double xf = -0.1311288625;
    double yf = -0.0206850171089;
    double zf = -0.00191181153059;

    tf::Quaternion orientation(xf, yf, zf, wf);
    tf::Matrix3x3 R1(orientation);
    
    tf::Vector3 t1 = R1.getRow(0);
    std::cout << t1.getX() << " " << t1.getY() << " "<< t1.getZ() << std::endl;
    tf::Vector3 t2 = R1.getRow(1);
    std::cout << t2.getX() << " " << t2.getY() << " "<< t2.getZ() << std::endl;
    tf::Vector3 t3 = R1.getRow(2);
    std::cout << t3.getX() << " " << t3.getY() << " "<< t3.getZ() << std::endl;
    //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
    tf::Quaternion differential_rotation;
    differential_rotation = orientation * zero_orientation.inverse() ;

    // calculate measurement time
    ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

    // publish imu message
    imu.header.stamp = measurement_time;
    imu.header.frame_id = frame_id;
    char c;

    std::cout << "differential_rotation" << std::endl;
    quaternionTFToMsg(differential_rotation, imu.orientation);
    imu_pub.publish(imu);
    std::cin >> c;

    std::cout << "zero_orientation" << std::endl;
    quaternionTFToMsg(zero_orientation, imu.orientation);
    imu_pub.publish(imu);
    std::cin >> c;

    std::cout << "orientation" << std::endl;
    quaternionTFToMsg(orientation, imu.orientation);
    imu_pub.publish(imu);
    std::cin >> c;

    ros::spinOnce();
    r.sleep();
  }
}

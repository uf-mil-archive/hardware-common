#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <arm_bootloader/arm_bootloader.h>

#include <stm32f3discovery_imu_driver/protocol.h>

extern unsigned char firmware_bin[];
extern int firmware_bin_len;

using namespace stm32f3discovery_imu_driver;

int main(int argc, char **argv) {
  ros::init(argc, argv, "stm32f3_discovery_imu_driver");
  
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  
  boost::asio::io_service io;

  boost::asio::serial_port sp(io);
  std::string port; private_nh.getParam("port", port);
  sp.open(port);
  sp.set_option(boost::asio::serial_port::baud_rate(115200));

  std::string deststr; private_nh.getParam("dest", deststr);
  arm_bootloader::Dest dest = strtol(deststr.c_str(), NULL, 0);

  std::string frame_id; private_nh.getParam("frame_id", frame_id);

  if(argc <= 1) {
    if(!arm_bootloader::attempt_bootload(port, sp, dest, firmware_bin, firmware_bin_len)) {
      std::cout << "bootloading failed" << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 65535);
  
  arm_bootloader::Reader<Response> reader(sp, 1000);
  
  arm_bootloader::SerialPortSink sps(port, sp);
  uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink>
    packetizer(sps);
  uf_subbus_protocol::ChecksumAdder<uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink> >
    checksumadder(packetizer);
  
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::MagneticField>("/imu/mag_raw", 10);
  
  while(ros::ok()) {
    Command cmd; memset(&cmd, 0, sizeof(cmd));
    cmd.dest = dest;
    cmd.id = dis(gen);
    cmd.command = CommandID::GetIMUData;
    write_object(cmd, checksumadder);
    
    boost::optional<Response> resp = reader.read(cmd.id);
    if(!resp) {
      std::cout << "timeout receiving imu packet!" << std::endl;
      continue;
    }
    
    ros::Time now = ros::Time::now();
    
    sensor_msgs::Imu msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id;
    msg.linear_acceleration.x = resp->resp.GetIMUData.linear_acceleration[0];
    msg.linear_acceleration.y = resp->resp.GetIMUData.linear_acceleration[1];
    msg.linear_acceleration.z = resp->resp.GetIMUData.linear_acceleration[2];
    msg.angular_velocity.x = resp->resp.GetIMUData.angular_velocity[0];
    msg.angular_velocity.y = resp->resp.GetIMUData.angular_velocity[1];
    msg.angular_velocity.z = resp->resp.GetIMUData.angular_velocity[2];
    pub.publish(msg);
    
    sensor_msgs::MagneticField msg2;
    msg2.header.stamp = now;
    msg2.header.frame_id = frame_id;
    msg2.magnetic_field.x = resp->resp.GetIMUData.magnetic_field[0];
    msg2.magnetic_field.y = resp->resp.GetIMUData.magnetic_field[1];
    msg2.magnetic_field.z = resp->resp.GetIMUData.magnetic_field[2];
    pub2.publish(msg2);
  }
  
  ros::waitForShutdown();

  return EXIT_SUCCESS;
}

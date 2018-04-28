#include <unistd.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"

#define DL_HEADER1                        0x54
#define DL_HEADER2                        0x4d
#define DL_CLASS_MINIAHRS                 0x0f
#define DL_MINIAHRS_ATTITUDE_AND_SENSORS  0x01

#define DL_PAYLOAD_LENGTH                 0x31
#define DL_CHECKSUM_LENGTH                0x02

#define DL_NO_ERR                         0x00
#define DL_UNKNOW_MESSAGE                 0x01
#define DL_CHECKSUM_ERR                   0x02
#define DL_PAYLOAD_LENGTH_ERR             0x04

#define COEF_DEG_TO_RAD                   57.29578
#define g                                 9.8

using namespace std;

static unsigned short CRC16Table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

bool check_eq(serial::Serial& serial, uint8_t num){
  uint8_t buffer;
  serial.read(&buffer, 1);
  if (buffer == num){
    return true;
  }else{
    return false;
  }
}

unsigned short cal_checksum(uint8_t * payload_ptr){
  unsigned short checksum = 0;
  checksum = ( checksum >> 8 ) ^ CRC16Table[ (checksum&0xff) ^ DL_CLASS_MINIAHRS ];
  checksum = ( checksum >> 8 ) ^ CRC16Table[ (checksum&0xff) ^ DL_MINIAHRS_ATTITUDE_AND_SENSORS ];
  checksum = ( checksum >> 8 ) ^ CRC16Table[ (checksum&0xff) ^ DL_PAYLOAD_LENGTH ];
  checksum = ( checksum >> 8 ) ^ CRC16Table[ (checksum&0xff) ^ 0x00 ];
  for (int i=0; i<DL_PAYLOAD_LENGTH; i++){
    checksum = ( checksum >> 8 ) ^ CRC16Table[ (checksum&0xFF) ^ *(payload_ptr+i) ];
  }
  return checksum;
}

float convert_float(uint8_t * ptr){
  float f;
  memcpy(&f, ptr, 4);
  return f;
}

void convert_to_msg(sensor_msgs::Imu& msg, uint8_t * payload){
  msg.header.frame_id = "imu_link";
  msg.header.stamp = ros::Time::now();

  tf2::Quaternion q;
  q.setRPY(convert_float(payload+1) / COEF_DEG_TO_RAD, convert_float(payload+5) / COEF_DEG_TO_RAD, convert_float(payload+9) / COEF_DEG_TO_RAD);
  msg.orientation.x = q.getX();
  msg.orientation.y = q.getY();
  msg.orientation.z = q.getZ();
  msg.orientation.w = q.getW();
  // msg.orientation_covariance[0] = -1.0;

  msg.angular_velocity.x = (double)convert_float(payload+13) / COEF_DEG_TO_RAD;
  msg.angular_velocity.y = (double)convert_float(payload+17) / COEF_DEG_TO_RAD;
  msg.angular_velocity.z = (double)convert_float(payload+21) / COEF_DEG_TO_RAD;

  msg.linear_acceleration.x = (double)convert_float(payload+25) * g;
  msg.linear_acceleration.y = (double)convert_float(payload+29) * g;
  msg.linear_acceleration.z = (double)convert_float(payload+33) * g;
}

void fetch_payload(serial::Serial& serial, uint8_t* payload){
  unsigned char state = 0;
  if (serial.isOpen()){
    while(1){
      switch (state){
      case 0:{ // Header 1
        state = check_eq(serial, DL_HEADER1) ? 1 : 0;
        break;
      }case 1:{ // Header 2
        state = check_eq(serial, DL_HEADER2) ? 2 : 0;
        break;
      }case 2:{ // CLASS
        state = check_eq(serial, DL_CLASS_MINIAHRS) ? 3 : 0;
        break;
      }case 3:{ // ID
        state = check_eq(serial, DL_MINIAHRS_ATTITUDE_AND_SENSORS) ? 4 : 0;
        break;
      }case 4:{ // LENGTH
        state = (check_eq(serial, DL_PAYLOAD_LENGTH) && check_eq(serial, 0x00)) ? 5 : 0;
        break;
      }case 5:{ // PAYLOAD
        size_t read_payload_size = serial.read(payload, (int)DL_PAYLOAD_LENGTH);
        state = read_payload_size == DL_PAYLOAD_LENGTH ? 6 : 0;
        break;
      }case 6:{ // CHECKSUM
        unsigned short checksum = cal_checksum(payload);
        state = (check_eq(serial, static_cast<uint8_t> (checksum&0x00ff))&&check_eq(serial, static_cast<uint8_t> (checksum>>8))) ? 7 : 0;
        break;
      }case 7:{
        state = 0;
        return;
      }default:{
        state = 0;
        break;
      }
      }
    }
  }
}

int main(int argc, char **argv)
{
  // Set up ROS.
  int baud;
  string port;
  ros::init(argc, argv, "ah100b");
  ros::NodeHandle nh;
  ros::NodeHandle private_node("~");
  private_node.param<string>("port", port, "/dev/ttyUSB0");
  private_node.param<int>("baud", baud, 115200);
  serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(5000));
  ros::Publisher publisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Rate loop_rate(100);
  while (ros::ok()){
    sensor_msgs::Imu msg;
    uint8_t payload[DL_PAYLOAD_LENGTH];
    fetch_payload(serial, payload);
    convert_to_msg(msg, payload);
    publisher.publish(msg);
    loop_rate.sleep();
  }
  serial.close();
  return 0;
}

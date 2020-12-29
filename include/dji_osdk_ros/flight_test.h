
// ros
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <dji_osdk_ros/common_type.h>
// std_msgs
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
// camera
#include <dji_osdk_ros/CameraAction.h>
#include <dji_osdk_ros/Gimbal.h>
#include <geometry_msgs/Vector3Stamped.h>
// SDKControlAuthority
#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/SDKControlAuthority.h>
#include <dji_osdk_ros/Activation.h>
#include <dji_control.hpp>
// mobile sdk
#include <dji_osdk_ros/MobileData.h>
#include <dji_osdk_ros/SendMobileData.h>
//Delay
#include <unistd.h>
//waypoint
#include <dji_osdk_ros/MissionWpAction.h>
#include <dji_osdk_ros/MissionWpUpload.h>
#include <dji_osdk_ros/MissionWpAction.h>

#include <dji_osdk_ros/MissionStatus.h>


#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

using namespace dji_osdk_ros;
using namespace DJI::OSDK;

struct GimbalContainer{
  int roll = 0;
  int pitch = 0;
  int yaw = 0;
  int duration = 0;
  int isAbsolute = 0;
  bool yaw_cmd_ignore = false;
  bool pitch_cmd_ignore = false;
  bool roll_cmd_ignore = false;
  RotationAngle initialAngle;
  RotationAngle currentAngle;
  GimbalContainer( int roll = 0,
                   int pitch = 0,
                   int yaw = 0,
                   int duration = 0,
                   int isAbsolute = 0,
                   RotationAngle initialAngle = {},
                   RotationAngle currentAngle = {}):
    roll(roll), pitch(pitch), yaw(yaw),
    duration(duration),isAbsolute(isAbsolute),
    initialAngle(initialAngle), currentAngle(currentAngle){}
};

typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
    
  }
  ServiceAck()
  {
  }
} ServiceAck;



typedef struct{
	int t_num;
	float t_lat;
	float t_lng;
}Tower;





// global variables
geometry_msgs::Vector3Stamped gimbal_angle;
ros::Subscriber         gimbal_angle_subscriber;
ros::Publisher          gimbal_angle_cmd_publisher;
ros::Publisher          gimbal_speed_cmd_publisher;
ros::ServiceClient      camera_action_service;
ros::Subscriber         line_angle;

ros::Publisher          ctrlBrakePub;
ros::ServiceClient      sdk_ctrl_authority_service;
ros::ServiceClient      drone_activation_service;
ros::ServiceClient      drone_task_control_service;
ros::Subscriber         mobile_data;
ros::ServiceClient      send_to_mobile_client;


/* all function --h */
void displayResult(RotationAngle *currentAngle); 
void doSetGimbalAngle(GimbalContainer *gimbal);
void camera_gimble_act(float roll,float pitch,float yaw,float duration,float isAb); 
void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
ServiceAck obtainCtrlAuthority();
ServiceAck release_obtainCtrlAuthority();
ServiceAck land();
ServiceAck activate();
void send_to_mobile(std::string send_data);
void flight_velocity(float v_x, float v_y, float v_z, float yaw);
bool takePicture();

/* function defined --cpp*/

const void Vz_PID_control();
const void Vy_PID_control();
const void Vyaw_control();
void startMission();
void shootTowerT();
void moblie_data_callback(const dji_osdk_ros::MobileData::ConstPtr& from_mobile_data);
void oBarrier();
void check();

/* camera*/
bool
takePicture()
{
  dji_osdk_ros::CameraAction cameraAction;
  cameraAction.request.camera_action = 0;
  camera_action_service.call(cameraAction);
  return cameraAction.response.result;
}

void
displayResult(RotationAngle *currentAngle)
{
    ROS_INFO("New Gimbal rotation angle is [ %f, %f, %f ] deg",
             currentAngle->roll,
             currentAngle->pitch,
             currentAngle->yaw);
}

void
doSetGimbalAngle(GimbalContainer *gimbal)
{
    dji_osdk_ros::Gimbal gimbal_angle_data;
    gimbal_angle_data.mode |= 0;
    gimbal_angle_data.mode |= gimbal->isAbsolute;
    gimbal_angle_data.mode |= gimbal->yaw_cmd_ignore << 1;
    gimbal_angle_data.mode |= gimbal->roll_cmd_ignore << 2;
    gimbal_angle_data.mode |= gimbal->pitch_cmd_ignore << 3;
    gimbal_angle_data.ts    = gimbal->duration;
    gimbal_angle_data.roll  = DEG2RAD(gimbal->roll);
    gimbal_angle_data.pitch = DEG2RAD(gimbal->pitch);
    gimbal_angle_data.yaw   = DEG2RAD(gimbal->yaw);

    cout << "gimbal_angle_data.mode=" << gimbal_angle_data.mode << endl;

    gimbal_angle_cmd_publisher.publish(gimbal_angle_data);
    // Give time for gimbal to sync
    sleep(4);
}


void camera_gimble_act(float roll,float pitch,float yaw,float duration,float isAb){
    GimbalContainer               gimbal;
    RotationAngle                 initialAngle;
    RotationAngle                 currentAngle;
    geometry_msgs::Vector3Stamped gimbalSpeed;


    // Get Gimbal initial values
    ros::spinOnce();
    initialAngle.roll = gimbal_angle.vector.y;
    initialAngle.pitch = gimbal_angle.vector.x;
    initialAngle.yaw = gimbal_angle.vector.z;

    ROS_INFO("Initial Gimbal rotation angle: [ %f, %f, %f ] deg",
             initialAngle.roll,
             initialAngle.pitch,
             initialAngle.yaw);

    // Re-set Gimbal to initial values
    //倾斜  俯仰角 方向  持续时间  绝对的
    //roll pitch yaw duration isAbsolute
    gimbal = GimbalContainer(roll,pitch,yaw,duration,isAb,initialAngle);
    doSetGimbalAngle(&gimbal);
    ros::spinOnce();
    currentAngle.roll = gimbal_angle.vector.y;
    currentAngle.pitch = gimbal_angle.vector.x;
    currentAngle.yaw = gimbal_angle.vector.z;
    displayResult(&currentAngle);
}



void
gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    gimbal_angle = *msg;
}


ServiceAck
force_land()
{
    dji_osdk_ros::DroneTaskControl drone_task_control;
    drone_task_control.request.task = 31;
    drone_task_control_service.call(drone_task_control);
    return ServiceAck(drone_task_control.response.result, drone_task_control.response.cmd_set,
                      drone_task_control.response.cmd_id, drone_task_control.response.ack_data);
}


ServiceAck
land()
{
    dji_osdk_ros::DroneTaskControl drone_task_control;
    drone_task_control.request.task = 6;
    drone_task_control_service.call(drone_task_control);
    if (!drone_task_control.response.result)
    {
        ROS_WARN("ack.info: set = %i id = %i", drone_task_control.response.cmd_set,
                 drone_task_control.response.cmd_id);
        ROS_WARN("ack.data: %i", drone_task_control.response.ack_data);
    }
    return ServiceAck(drone_task_control.response.result, drone_task_control.response.cmd_set,
                      drone_task_control.response.cmd_id, drone_task_control.response.ack_data);
}


ServiceAck
obtainCtrlAuthority()
{
    dji_osdk_ros::SDKControlAuthority sdkAuthority;
    sdkAuthority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(sdkAuthority);
    if (!sdkAuthority.response.result)
    {
        ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                 sdkAuthority.response.cmd_id);
        ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
    }
     return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                      sdkAuthority.response.cmd_id,
                      sdkAuthority.response.ack_data);
}

ServiceAck
release_obtainCtrlAuthority()
{
    dji_osdk_ros::SDKControlAuthority sdkAuthority;
    sdkAuthority.request.control_enable = 0;
    sdk_ctrl_authority_service.call(sdkAuthority);
    if (!sdkAuthority.response.result)
    {
        ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                 sdkAuthority.response.cmd_id);
        ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
    }
     return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                      sdkAuthority.response.cmd_id,
                      sdkAuthority.response.ack_data);
}

ServiceAck
activate()
{
    dji_osdk_ros::Activation activation;
    drone_activation_service.call(activation);
    if (!activation.response.result)
    {
        ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
                 activation.response.cmd_id);
        ROS_WARN("ack.data: %i", activation.response.ack_data);
    }
    return ServiceAck(activation.response.result, activation.response.cmd_set,
                      activation.response.cmd_id, activation.response.ack_data);
}


void send_to_mobile(std::string send_data)
{
  char char_send_data[100] = {0};
  strcpy(char_send_data, send_data.c_str());
  dji_osdk_ros::SendMobileData mobile_data;
  mobile_data.request.data.resize(sizeof(char_send_data));
  memcpy(&mobile_data.request.data[0], (uint8_t *)(&char_send_data), sizeof(char_send_data));
  send_to_mobile_client.call(mobile_data);
}



void flight_velocity(float v_x, float v_y, float v_z, float yaw)
{
    sensor_msgs::Joy controlVelYawRate;
    /*
    uint8_t flag = (Control::VERTICAL_VELOCITY   |
                    Control::HORIZONTAL_VELOCITY |
                    Control::YAW_RATE            |
                    //DJI::OSDK::HORIZONTAL_GROUND |
                    Control::HORIZONTAL_BODY   |
                    Control::STABLE_ENABLE);
                    */

    uint8_t flag = (DJI::OSDK::Control::VERTICAL_VELOCITY |
		    DJI::OSDK::Control::HORIZONTAL_VELOCITY |
		    DJI::OSDK::Control::YAW_RATE |
		    DJI::OSDK::Control::HORIZONTAL_BODY |
		    DJI::OSDK::Control::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(v_x);
    controlVelYawRate.axes.push_back(v_y);
    controlVelYawRate.axes.push_back(v_z);
    controlVelYawRate.axes.push_back(yaw);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    
    //ROS_INFO("Velocity: v_x:%f, v_y:%f, v_z:%f, yaw:%f",v_x, v_y, v_z, yaw);
	     
}





/** @file advanced_sensing_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of flight control.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/SetNewHomePoint.h>
#include <dji_osdk_ros/AvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>

//CODE
using namespace dji_osdk_ros;

ros::ServiceClient task_control_client;

bool moveByPosOffset(FlightTaskControl& task, MoveOffset&& move_offset);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;
  task_control_client = nh.serviceClient<FlightTaskControl>("/flight_task_control");
  auto set_go_home_altitude_client = nh.serviceClient<SetGoHomeAltitude>("/set_go_home_altitude");
  auto set_current_point_as_home_client = nh.serviceClient<SetNewHomePoint>("/set_current_point_as_home");
  auto enable_avoid_client = nh.serviceClient<AvoidEnable>("/enable_avoid");
  auto enable_upward_avoid_client = nh.serviceClient<AvoidEnable>("/enable_upwards_avoid");
  auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");
  std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Monitored Takeoff + Landing                                |"
      << std::endl;
  std::cout
      << "| [b] Monitored Takeoff + Position Control + Landing             |"
      << std::endl;
  std::cout << "| [c] Monitored Takeoff + Position Control + Force Landing "
               "Avoid Ground  |"
            << std::endl;

  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;

  FlightTaskControl control_task;
  ObtainControlAuthority obtainCtrlAuthority;
  obtainCtrlAuthority.request.enable_obtain = true;
  obtain_ctrl_authority_client.call(obtainCtrlAuthority);

  switch (inputChar)
  {
    case 'a':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("Land request sending ...");
          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
        ROS_ERROR_STREAM("Takeoff task failed");
        break;
      }
    case 'b':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          
          ROS_INFO_STREAM("Move by position offset request sending ...");
          moveByPosOffset(control_task, MoveOffset(10.0, 0.0, 0.0, 0.0));
          ROS_INFO_STREAM("Step 1 over!");
          moveByPosOffset(control_task, MoveOffset(0.0, 10.0, 0.0, 0.0));
          ROS_INFO_STREAM("Step 2 over!");
          moveByPosOffset(control_task, MoveOffset(0.0, 0.0, 10.0, 0.0));
          ROS_INFO_STREAM("Step 3 over!");
		  moveByPosOffset(control_task, MoveOffset(0.0, 0.0, 0.0, 10.0));
          ROS_INFO_STREAM("Step 4 over!");

          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          ROS_INFO_STREAM("Landing request sending ...");
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
        break;
      }
    case 'c':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if (control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("turn on Collision-Avoidance-Enabled");
          AvoidEnable avoid_req;
          avoid_req.request.enable = true;
          enable_avoid_client.call(avoid_req);
          if(avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Avoid FAILED");
          }

          ROS_INFO_STREAM("turn on Upwards-Collision-Avoidance-Enabled");
          AvoidEnable upward_avoid_req;
          upward_avoid_req.request.enable = true;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          ROS_INFO_STREAM("Move by position offset request sending ...");
          ROS_INFO_STREAM("Move to higher altitude");
          moveByPosOffset(control_task, MoveOffset(0.0, 0.0, 30.0, 0.0));
          ROS_INFO_STREAM("Move a short distance");
          moveByPosOffset(control_task, MoveOffset(10.0, 0.0, 0.0, 0.0));

          ROS_INFO_STREAM("Set aircraft current position as new home location");
          SetNewHomePoint home_set_req;
          set_current_point_as_home_client.call(home_set_req);
          if(home_set_req.response.result == false)
          {
            ROS_ERROR_STREAM("Set current position as Home, FAILED");
            break;
          }

          ROS_INFO_STREAM("Set new go home altitude");
          SetGoHomeAltitude altitude_go_home;
          altitude_go_home.request.altitude = 50;
          set_go_home_altitude_client.call(altitude_go_home);
          if(altitude_go_home.response.result == false)
          {
            ROS_ERROR_STREAM("Set altitude for go home FAILED");
            break;
          }

          ROS_INFO_STREAM("Move to another position");
          moveByPosOffset(control_task, MoveOffset(50.0, 0.0, 0.0, 0.0));

          ROS_INFO_STREAM("Shut down Collision-Avoidance-Enabled");
          avoid_req.request.enable = false;
          enable_avoid_client.call(avoid_req);
          if(avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Disable Avoid FAILED");
          }

          ROS_INFO_STREAM("Go home...");

          control_task.request.task = FlightTaskControl::Request::TASK_GOHOME_AND_CONFIRM_LANDING;
          task_control_client.call(control_task);
          if(control_task.response.result == false)
          {
            ROS_ERROR_STREAM("GO HOME FAILED");
          }
          break;
        }
      }
    default:
      break;
  }

  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

  ros::spin();
  return 0;
}


bool moveByPosOffset(FlightTaskControl& task, MoveOffset&& move_offset)
{
  task.request.task = FlightTaskControl::Request::TASK_GO_LOCAL_POS;
  // pos_offset: A vector contains that position_x_offset, position_y_offset, position_z_offset in order
  task.request.pos_offset.clear();
  task.request.pos_offset.push_back(move_offset.x);
  task.request.pos_offset.push_back(move_offset.y);
  task.request.pos_offset.push_back(move_offset.z);
  // yaw_params: A vector contains that yaw_desired, position_threshold(Meter), yaw_threshold(Degree)
  task.request.yaw_params.clear();
  task.request.yaw_params.push_back(move_offset.yaw);
  task.request.yaw_params.push_back(move_offset.pos_threshold);
  task.request.yaw_params.push_back(move_offset.yaw_threshold);
  task_control_client.call(task);
  return task.response.result;
}

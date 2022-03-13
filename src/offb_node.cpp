/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

#include <cmath>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

geometry_msgs::PoseStamped *createArc(geometry_msgs::PoseStamped startPosition, int numberOfPoints, float radius, float height)
{
    float angle, angleIncrement, arcAngle;

    geometry_msgs::PoseStamped *pose = new geometry_msgs::PoseStamped[numberOfPoints];
    arcAngle = 360;

    angleIncrement = arcAngle / numberOfPoints;
    angle = 0;

    for (int i = 0; i < numberOfPoints; i++)
    {
        pose[i].pose.position.x = startPosition.pose.position.x + (cos(angle * M_PI / 180) * radius);
        pose[i].pose.position.y = startPosition.pose.position.y + (sin(angle * M_PI / 180) * radius);
        pose[i].pose.position.z = height;

        angle += angleIncrement;
    }

    return pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    //------------------------------------------------------
    int size_of_points = 10;
    geometry_msgs::PoseStamped points[size_of_points];
    double yaricap = 40;
    double katsayi = 360 / size_of_points;
    for (int i = 0; i < size_of_points; i++)
    {
        points[i].pose.position.x = yaricap * cos(katsayi * i * M_PI / 180);
        points[i].pose.position.y = yaricap * sin(katsayi * i * M_PI / 180);
        points[i].pose.position.z = 20;
    }

    // geometry_msgs::PoseStamped *points = createArc(current_pose,size_of_points,5,10);
    // geometry_msgs::PoseStamped points[14];

    // points[0].pose.position.y = 0;
    // points[0].pose.position.x = 0;
    // points[0].pose.position.z = 2;

    // points[1].pose.position.x = 1;
    // points[1].pose.position.y = 3;
    // points[1].pose.position.z = 2;

    // points[2].pose.position.x = 2;
    // points[2].pose.position.y = 2;
    // points[2].pose.position.z = 2;

    // points[3].pose.position.x = 3;
    // points[3].pose.position.y = 1;
    // points[3].pose.position.z = 2;

    // points[4].pose.position.x = 3;
    // points[4].pose.position.y = -1;
    // points[4].pose.position.z = 2;

    // points[5].pose.position.x = 2;
    // points[5].pose.position.y = -2;
    // points[5].pose.position.z = 2;

    // points[6].pose.position.x = 1;
    // points[6].pose.position.y = -3;
    // points[6].pose.position.z = 2;

    // points[7].pose.position.x = -1;
    // points[7].pose.position.y = -3;
    // points[7].pose.position.z = 2;

    // points[8].pose.position.x = -2;
    // points[8].pose.position.y = -2;
    // points[8].pose.position.z = 2;

    // points[9].pose.position.x = -3;
    // points[9].pose.position.y = -1;
    // points[9].pose.position.z = 2;

    // points[10].pose.position.x = -3;
    // points[10].pose.position.y = 1;
    // points[10].pose.position.z = 2;

    // points[11].pose.position.x = -2;
    // points[11].pose.position.y = 2;
    // points[11].pose.position.z = 2;

    // points[12].pose.position.x = -1;
    // points[12].pose.position.y = 3;
    // points[12].pose.position.z = 2;

    // points[13].pose.position.x = 1;
    // points[13].pose.position.y = 3;
    // points[13].pose.position.z = 2;
    //------------------------------------------------------
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        // local_pos_pub.publish(pose);
        local_pos_pub.publish(points[0]);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 30;
    takeoff_cmd.request.latitude = 0;
    takeoff_cmd.request.longitude = 0;
    takeoff_cmd.request.min_pitch = 0;
    takeoff_cmd.request.yaw = 0;

    bool takeoff_status = false;

    ros::Time last_request = ros::Time::now();
    int j = 0;
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

            if (!takeoff_status)
            {
                if (takeoff_client.call(takeoff_cmd) &&
                    takeoff_cmd.response.success)
                {
                    ROS_INFO("Vehicle takeoff");
                    takeoff_status = true;
                }
                else
                {
                    ROS_INFO("Vehicle takeoff failed");
                }
                last_request = ros::Time::now();
            }
        }

        // local_pos_pub.publish(pose);
        ROS_INFO("current waypoint = %d", j);
        if (10 > sqrt(pow(current_pose.pose.position.x - points[j].pose.position.x, 2) + pow(current_pose.pose.position.y - points[j].pose.position.y, 2)))
        {
            j = (j + 1) % size_of_points;
        }
        local_pos_pub.publish(points[j]);

        ros::spinOnce();
        rate.sleep();
    }
    // delete points;

    return 0;
}
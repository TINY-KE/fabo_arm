#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Pose target_pose1 ;

void CallBack(const geometry_msgs::PoseStamped pose)
{
    target_pose1 = pose.pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm");

    group.setGoalPositionTolerance(0.02);
    group.setGoalOrientationTolerance(0.05);

    // moveit_msgs::Constraints joint_constraints;
    // joint_constraints.joint_constraints.resize(4);
    // joint_constraints.joint_constraints[0].joint_name = "joint2";
    // joint_constraints.joint_constraints[0].position = 0.0;
    // joint_constraints.joint_constraints[0].tolerance_above = 0.01;
    // joint_constraints.joint_constraints[0].tolerance_below = 0.01;
    // joint_constraints.joint_constraints[0].weight = 1.0;

    // joint_constraints.joint_constraints[1].joint_name = "joint3";
    // joint_constraints.joint_constraints[1].position = 0.0;
    // joint_constraints.joint_constraints[1].tolerance_above = 0.01;
    // joint_constraints.joint_constraints[1].tolerance_below = 0.01;
    // joint_constraints.joint_constraints[1].weight = 1.0;

    // joint_constraints.joint_constraints[2].joint_name = "joint7";
    // joint_constraints.joint_constraints[2].position = 0.0;
    // joint_constraints.joint_constraints[2].tolerance_above = 0.01;
    // joint_constraints.joint_constraints[2].tolerance_below = 0.01;
    // joint_constraints.joint_constraints[2].weight = 1.0;

    sleep(1.0);  

    ros::WallDuration(1.0).sleep();

    target_pose1.orientation.w = 0.703184;
    target_pose1.orientation.x= 1.42057e-05;
    target_pose1.orientation.y = 0.711007;
    target_pose1.orientation.z = -1.912e-05;

    target_pose1.position.x = 0.2;
    target_pose1.position.y = 0.3;
    target_pose1.position.z = 0.5;
    group.setPoseTarget(target_pose1, "Link7");
    ros::Subscriber Pose_Cmd = nh.subscribe("/aruco_single/pose", 10, CallBack);
    //填入目标位置


    while(1)
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        ros::spinOnce();
        group.setPoseTarget(target_pose1, "Link7");
        moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"target_pose1":"FAILED");

        if(success)
        {
            group.execute(my_plan);

            ros::WallDuration(1.0).sleep();
        }
        sleep(2.0);
        
    }

    ros::waitForShutdown();

    return 0;
}
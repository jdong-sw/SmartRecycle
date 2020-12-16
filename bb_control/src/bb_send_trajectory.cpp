#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <boost/scoped_ptr.hpp>

void RunTaskTrajectory(double x, double y, double z)
{
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "bb_arm";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;
    move_group.setPoseTarget(target_pose1);

    ROS_INFO("Pose Target set.");
    ROS_INFO("Initiating planning...");

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (!success)
    {
        ROS_INFO("Planning failed.");
        return;
    }

    ROS_INFO("Executing Plan...");

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    auto result = move_group.move();

    ROS_INFO("Result: %i", result.val);

    ROS_INFO("Finished.");
}

void RunJointTrajectory(std::vector<double> joint_positions)
{
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "bb_arm";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Let's set a joint space goal and move towards it.  This will replace the
    // pose target we set above.
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Set joint position target.
    move_group.setJointValueTarget(joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // We will reuse the old goal that we had and plan to it.
    // Note that this will only work if the current state already
    // satisfies the path constraints. So, we need to set the start
    // state to a new pose.
    robot_state::RobotState start_state(*move_group.getCurrentState());
    //geometry_msgs::Pose start_pose;
    //start_pose.orientation.w = 1.0;
    //start_pose.position.x = 0.55;
    //start_pose.position.y = -0.05;
    //start_pose.position.z = 0.8;
    //start_state.setFromIK(joint_model_group, start_pose);
    move_group.setStartState(start_state);

    // Now we will plan to the earlier pose target from the new
    // start state that we have just created.
    //move_group.setPoseTarget(target_pose1);

    // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
    // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
    //move_group.setPlanningTime(10.0);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    ROS_INFO("Executing Plan...");

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    auto result = move_group.move();

    ROS_INFO("Result: %i", result.val);

    ROS_INFO("Finished.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bb_send_trajectory");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double x, y, z;
    bool gotX = node_handle.getParam("x", x);
    bool gotY = node_handle.getParam("y", y);
    bool gotZ = node_handle.getParam("z", z);


    std::vector<double> joint_positions = std::vector<double>();
    bool gotJoints = node_handle.getParam("joint", joint_positions);

    std::string config;
    bool gotConfig = node_handle.getParam("config", config);

    if (gotJoints)
    {
        ROS_INFO("Starting motion planning for joint positions:");
        for (int i = 0; i < joint_positions.size(); ++i)
        {
            ROS_INFO("    %f", joint_positions[i]);
        }
        RunJointTrajectory(joint_positions);

        node_handle.deleteParam("joint");
    }
    else if (gotX && gotY && gotZ)
    {
        ROS_INFO("Starting motion planning for EE position: (%f, %f, %f)", x, y, z);
        RunTaskTrajectory(x, y, z);

        node_handle.deleteParam("x");
        node_handle.deleteParam("y");
        node_handle.deleteParam("z");
    }
    else if(gotConfig)
    {
        std::string home = "home";
        if (config.compare(home) == 0)
        {
            ROS_INFO("Homing...");
            std::vector<double> positions = {0, 0, 0, 0, 0, 0, 0};
            RunJointTrajectory(positions);
        }
        else
        {
            ROS_INFO("'%s' not recognized.", config.c_str());
        }
    }
    else
    {
        ROS_INFO("No parameters given.");
    }
}
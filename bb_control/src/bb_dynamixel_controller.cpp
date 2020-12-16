#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class BBDynamixelController
{
protected:
    ros::NodeHandle node;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> server;
    ros::Publisher dxl_publisher;
    ros::Publisher joint_state_publisher;
    ros::Subscriber joint_state_subscriber;
    std::string action_name;

public:
    BBDynamixelController(std::string action_ns, std::string topic_ns) : action_name(action_ns),
                                                                         server(node, action_ns, boost::bind(&BBDynamixelController::MoveJoint, this, _1), false)
    {
        ROS_INFO("Starting server...");
        server.start();
        ROS_INFO("Advertising topics...");
        std::string traj_ns = topic_ns + "/joint_trajectory";
        dxl_publisher = node.advertise<trajectory_msgs::JointTrajectory>(traj_ns, 5);
        joint_state_publisher = node.advertise<sensor_msgs::JointState>("/joint_states", 100);

        std::string joint_ns = topic_ns + "/joint_states";
        joint_state_subscriber = node.subscribe(joint_ns, 100, &BBDynamixelController::PublishJointStates, this);
    }

    ~BBDynamixelController(void)
    {
    }

    void MoveJoint(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO("Joint trajectory received.");

        // Get trajectory info
        auto joints = goal->trajectory.joint_names;
        auto points = goal->trajectory.points;

        // Publish
        dxl_publisher.publish(goal->trajectory);
        ROS_INFO("Messages sent.");

        bool result = true;
        auto result_msg = new control_msgs::FollowJointTrajectoryResult();

        if (result)
        {
            result_msg->error_code = result_msg->SUCCESSFUL;
        }
        else
        {
            result_msg->error_code = result_msg->INVALID_GOAL;
        }

        server.setSucceeded(*result_msg);
        ROS_INFO("Trajectory published.");
    }

    void PublishJointStates(const sensor_msgs::JointStateConstPtr msg)
    {
        joint_state_publisher.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bb_dynamixel_controller");

    std::string name = "bb_arm_controller/follow_joint_trajectory";
    BBDynamixelController bb_arm_controller(name, argv[1]);
    ros::spin();
}
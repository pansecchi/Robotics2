#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Structure to hold goal coordinates
struct Goal {
    double x;
    double y;
    double yaw;
};

// Read goals from a CSV file
std::vector<Goal> readGoalsFromCSV(const std::string& filename) {
    std::vector<Goal> goals;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return goals;
    }

    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        Goal goal;
        std::string cell;

        std::getline(lineStream, cell, ',');
        goal.x = std::stod(cell);
        std::getline(lineStream, cell, ',');
        goal.y = std::stod(cell);
        std::getline(lineStream, cell, ',');
        goal.yaw = std::stod(cell);

        goals.push_back(goal);
    }
    return goals;
}

// Send a goal to the move_base action server
void sendGoal(MoveBaseClient& ac, const Goal& goal) {
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.frame_id = "map";
    mb_goal.target_pose.header.stamp = ros::Time::now();

    mb_goal.target_pose.pose.position.x = goal.x;
    mb_goal.target_pose.pose.position.y = goal.y;
    mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal.yaw);

    ROS_INFO("Sending goal: x=%f, y=%f, yaw=%f", goal.x, goal.y, goal.yaw);
    ac.sendGoal(mb_goal);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher_node");
    ros::NodeHandle nh;

    // Get the path to the ROS package and construct the file path
    std::string package_path = ros::package::getPath("second_project");
    std::string filename = package_path + "/waypoints.csv";

    ROS_INFO("Reading goals from file: %s", filename.c_str());
    std::vector<Goal> goals = readGoalsFromCSV(filename);

    if (goals.empty()) {
        ROS_ERROR("No goals found in the file.");
        return 1;
    }

    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for (const auto& goal : goals) {
        sendGoal(ac, goal);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to the goal!");
        } else {
            ROS_WARN("The base failed to move to the goal for some reason");
        }
    }

    return 0;
}

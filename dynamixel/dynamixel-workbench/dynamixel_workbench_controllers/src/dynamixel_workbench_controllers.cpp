/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

DynamixelController::DynamixelController()
    : node_handle_(""),
      priv_node_handle_("~"),
      is_joint_state_topic_(false),
      is_cmd_vel_topic_(false),
      use_moveit_(false),
      wheel_separation_(0.0f),
      wheel_radius_(0.0f),
      is_moving_(false)
{
    is_joint_state_topic_ = priv_node_handle_.param<bool>("use_joint_states_topic", true);
    is_cmd_vel_topic_ = priv_node_handle_.param<bool>("use_cmd_vel_topic", false);
    use_moveit_ = priv_node_handle_.param<bool>("use_moveit", false);

    read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.010f);
    write_period_ = priv_node_handle_.param<double>("dxl_write_period", 0.010f);
    pub_period_ = priv_node_handle_.param<double>("publish_period", 0.010f);

    if (is_cmd_vel_topic_ == true)
    {
        wheel_separation_ = priv_node_handle_.param<double>("mobile_robot_config/seperation_between_wheels", 0.0);
        wheel_radius_ = priv_node_handle_.param<double>("mobile_robot_config/radius_of_wheel", 0.0);
    }

    dxl_wb_ = new DynamixelWorkbench;
    jnt_tra_ = new JointTrajectory;
    jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
}

DynamixelController::~DynamixelController() {}

bool DynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
    bool result = false;
    const char *log;

    result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
    }

    return result;
}

bool DynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
    YAML::Node dynamixel;
    dynamixel = YAML::LoadFile(yaml_file.c_str());

    if (dynamixel == NULL)
        return false;

    for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
    {
        std::string name = it_file->first.as<std::string>();
        if (name.size() == 0)
        {
            continue;
        }

        YAML::Node item = dynamixel[name];
        for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
        {
            std::string item_name = it_item->first.as<std::string>();

            if (item_name == "ID")
            {
                int32_t value = it_item->second.as<int32_t>();
                dynamixel_[name] = value;
                id2name_[value] = name;
                ItemValue item_value = {item_name, value};
                std::pair<std::string, ItemValue> info(name, item_value);
                dynamixel_info_.push_back(info);
            }
            else if (item_name == "GROUP")
            {
                std::string value = it_item->second.as<std::string>();
                dynamixel_groups_[dynamixel_[name]] = value;
                joint_groups_.insert(value);
                group_id_map_[value].push_back(dynamixel_[name]);
            }
        }
    }

    return true;
}

bool DynamixelController::loadDynamixels(void)
{
    bool result = false;
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
        uint16_t model_number = 0;
        result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
        if (result == false)
        {
            ROS_ERROR("%s", log);
            ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
            return result;
        }
        else
        {
            ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
        }
    }

    return result;
}

bool DynamixelController::initDynamixels(void)
{
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
        dxl_wb_->torqueOff((uint8_t)dxl.second);

        for (auto const &info : dynamixel_info_)
        {
            if (dxl.first == info.first)
            {
                if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
                {
                    bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
                    if (result == false)
                    {
                        ROS_ERROR("%s", log);
                        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
                        return false;
                    }
                }
            }
        }

        dxl_wb_->torqueOn((uint8_t)dxl.second);
    }

    return true;
}

bool DynamixelController::initControlItems(void)
{
    bool result = false;
    const char *log = NULL;

    for (auto grp : joint_groups_)
    {
        uint8_t id = group_id_map_[grp][0];
        const ControlItem *goal_position = dxl_wb_->getItemInfo(id, "Goal_Position");
        if (goal_position == NULL)
            return false;

        const ControlItem *goal_velocity = dxl_wb_->getItemInfo(id, "Goal_Velocity");
        if (goal_velocity == NULL)
            goal_velocity = dxl_wb_->getItemInfo(id, "Moving_Speed");
        if (goal_velocity == NULL)
            return false;

        const ControlItem *present_position = dxl_wb_->getItemInfo(id, "Present_Position");
        if (present_position == NULL)
            return false;

        const ControlItem *present_velocity = dxl_wb_->getItemInfo(id, "Present_Velocity");
        if (present_velocity == NULL)
            present_velocity = dxl_wb_->getItemInfo(id, "Present_Speed");
        if (present_velocity == NULL)
            return false;

        const ControlItem *present_current = dxl_wb_->getItemInfo(id, "Present_Current");
        if (present_current == NULL)
            present_current = dxl_wb_->getItemInfo(id, "Present_Load");
        if (present_current == NULL)
            return false;

        std::map<std::string, const ControlItem *> group_control_item;
        group_control_item["Goal_Position"] = goal_position;
        group_control_item["Goal_Velocity"] = goal_velocity;

        group_control_item["Present_Position"] = present_position;
        group_control_item["Present_Velocity"] = present_velocity;
        group_control_item["Present_Current"] = present_current;

        group_control_items_[grp] = group_control_item;
    }

    auto it = dynamixel_.begin();

    const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
    if (goal_position == NULL)
        return false;

    const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
    if (goal_velocity == NULL)
        goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
    if (goal_velocity == NULL)
        return false;

    const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
    if (present_position == NULL)
        return false;

    const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
    if (present_velocity == NULL)
        present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
    if (present_velocity == NULL)
        return false;

    const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
    if (present_current == NULL)
        present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
    if (present_current == NULL)
        return false;

    control_items_["Goal_Position"] = goal_position;
    control_items_["Goal_Velocity"] = goal_velocity;

    control_items_["Present_Position"] = present_position;
    control_items_["Present_Velocity"] = present_velocity;
    control_items_["Present_Current"] = present_current;

    return true;
}

bool DynamixelController::initSDKHandlers(void)
{
    bool result = false;
    const char *log = NULL;
    uint8_t grp_cnt = 0;

    for (auto grp : joint_groups_)
    {
        ROS_INFO("Initializing group: %s", grp.c_str());
        auto group_control_item = group_control_items_[grp];
        result = dxl_wb_->addSyncWriteHandler(group_control_item["Goal_Position"]->address, group_control_item["Goal_Position"]->data_length, &log);
        if (result == false)
        {
            ROS_ERROR("%s", log);
            return result;
        }
        else
        {
            ROS_INFO("%s", log);
        }

        result = dxl_wb_->addSyncWriteHandler(group_control_item["Goal_Velocity"]->address, group_control_item["Goal_Velocity"]->data_length, &log);
        if (result == false)
        {
            ROS_ERROR("%s", log);
            return result;
        }
        else
        {
            ROS_INFO("%s", log);
        }

        if (dxl_wb_->getProtocolVersion() == 2.0f)
        {
            //uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

            /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
            // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
            //uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

            //result = dxl_wb_->addSyncReadHandler(start_address,
            //                                     read_length,
            //                                     &log);
            if (grp.compare("arm") == 0)
            {
                auto address = group_control_item["Present_Current"]->address;
                uint16_t read_length = group_control_item["Present_Current"]->data_length + group_control_item["Present_Velocity"]->data_length + group_control_item["Present_Position"]->data_length;
                ROS_INFO("Trying to add arm reader...");
                result = dxl_wb_->addSyncReadHandler(address, read_length, &log);
            }
            else
            {
                auto address = group_control_item["Present_Position"]->address;
                uint16_t read_length = group_control_item["Present_Current"]->data_length + group_control_item["Present_Velocity"]->data_length + group_control_item["Present_Position"]->data_length;
                ROS_INFO("Trying to add wrist reader...");
                result = dxl_wb_->addSyncReadHandler(address, read_length, &log);
            }
            if (result == false)
            {
                ROS_ERROR("%s", log);
                return result;
            }
        }
        grp_cnt++;
    }

    return result;
}

bool DynamixelController::getPresentPosition(std::vector<std::string> dxl_name)
{
    bool result = false;
    const char *log = NULL;

    uint8_t grp_cnt = 0;
    for (auto grp : joint_groups_)
    {
        auto id_array = group_id_map_[grp];
        auto group_control_item = group_control_items_[grp];

        int32_t get_position[id_array.size()];

        uint8_t id_cnt = 0;

        if (dxl_wb_->getProtocolVersion() == 2.0f)
        {
            result = dxl_wb_->syncRead(grp_cnt,
                                       &id_array[0],
                                       id_array.size(),
                                       &log);
            if (result == false)
            {
                ROS_ERROR("%s", log);
            }

            WayPoint wp;

            result = dxl_wb_->getSyncReadData(grp_cnt,
                                              &id_array[0],
                                              id_array.size(),
                                              group_control_item["Present_Position"]->address,
                                              group_control_item["Present_Position"]->data_length,
                                              get_position,
                                              &log);

            if (result == false)
            {
                ROS_ERROR("%s", log);
            }
            else
            {
                // Hopefully joints are loaded in order, so this should be okay
                for (uint8_t index = 0; index < id_cnt; index++)
                {
                    wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
                    pre_goal_.push_back(wp);
                }
            }
        }
        else if (dxl_wb_->getProtocolVersion() == 1.0f)
        {
            WayPoint wp;
            uint32_t read_position;
            for (auto const &dxl : dynamixel_)
            {
                result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                               control_items_["Present_Position"]->address,
                                               control_items_["Present_Position"]->data_length,
                                               &read_position,
                                               &log);
                if (result == false)
                {
                    ROS_ERROR("%s", log);
                }

                wp.position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
                pre_goal_.push_back(wp);
            }
        }
        grp_cnt++;
    }

    return result;
}

void DynamixelController::initPublisher()
{
    dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
    if (is_joint_state_topic_)
        joint_states_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber()
{
    trajectory_sub_ = priv_node_handle_.subscribe("joint_trajectory", 100, &DynamixelController::trajectoryMsgCallback, this);
    if (is_cmd_vel_topic_)
        cmd_vel_sub_ = priv_node_handle_.subscribe("cmd_vel", 10, &DynamixelController::commandVelocityCallback, this);
}

void DynamixelController::initServer()
{
    dynamixel_command_server_ = priv_node_handle_.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
}

void DynamixelController::initFollowJointTrajectoryActionServer()
{
    if (!use_moveit_)
        return;

    follow_traj_server = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(priv_node_handle_, "follow_joint_trajectory", boost::bind(&DynamixelController::moveJoints, this, _1), false);
    follow_traj_server->start();
}

void DynamixelController::readCallback(const ros::TimerEvent &)
{
#ifdef DEBUG
    static double priv_read_secs = ros::Time::now().toSec();
#endif
    bool result = false;
    const char *log = NULL;

    dynamixel_workbench_msgs::DynamixelState dynamixel_state[dynamixel_.size()]; // Message holding states for each motor
    dynamixel_state_list_.dynamixel_state.clear();
    int size = dynamixel_.size();

    uint8_t id_cnt = 0;
    for (auto const &dxl : dynamixel_) // Populate joint state message with name and id
    {
        dynamixel_state[id_cnt].name = dxl.first;
        dynamixel_state[id_cnt].id = (uint8_t)dxl.second;
    }

    uint8_t grp_cnt = 0;
    for (auto grp : joint_groups_)
    {
        auto id_array = group_id_map_[grp];
        auto group_control_item = group_control_items_[grp];

        int32_t get_current[id_array.size()];
        int32_t get_velocity[id_array.size()];
        int32_t get_position[id_array.size()];

#ifndef DEBUG
        if (is_moving_ == false)
        {
#endif
            if (dxl_wb_->getProtocolVersion() == 2.0f)
            {
                // Revelation: thing to read is actually index for syncReadHandler, which exists for each series to work with
                // Might need to call this with each syncReadHandler
                result = dxl_wb_->syncRead(grp_cnt,         // Thing to read
                                           &id_array[0],    // Array of ids 1-7
                                           id_array.size(), // 7 motors
                                           &log);
                if (result == false)
                {
                    ROS_ERROR("%s", log);
                }

                result = dxl_wb_->getSyncReadData(grp_cnt,
                                                  &id_array[0],
                                                  id_array.size(),
                                                  group_control_item["Present_Current"]->address,
                                                  group_control_item["Present_Current"]->data_length,
                                                  get_current,
                                                  &log);
                if (result == false)
                {
                    ROS_ERROR("%s", log);
                }

                result = dxl_wb_->getSyncReadData(grp_cnt,
                                                  &id_array[0],
                                                  id_array.size(),
                                                  group_control_item["Present_Velocity"]->address,
                                                  group_control_item["Present_Velocity"]->data_length,
                                                  get_velocity,
                                                  &log);
                if (result == false)
                {
                    ROS_ERROR("%s", log);
                }

                result = dxl_wb_->getSyncReadData(grp_cnt,
                                                  &id_array[0],
                                                  id_array.size(),
                                                  group_control_item["Present_Position"]->address,
                                                  group_control_item["Present_Position"]->data_length,
                                                  get_position,
                                                  &log);
                if (result == false)
                {
                    ROS_ERROR("%s", log);
                }

                uint8_t indx = 0;
                for (auto index : id_array)
                {
                    dynamixel_state[index - 1].present_current = get_current[indx];
                    dynamixel_state[index - 1].present_velocity = get_velocity[indx];
                    dynamixel_state[index - 1].present_position = get_position[indx];
                    dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index - 1]);
                    indx++;
                }
            }
            else if (dxl_wb_->getProtocolVersion() == 1.0f)
            {
                uint16_t length_of_data = control_items_["Present_Position"]->data_length +
                                          control_items_["Present_Velocity"]->data_length +
                                          control_items_["Present_Current"]->data_length;
                uint32_t get_all_data[length_of_data];
                uint8_t dxl_cnt = 0;
                for (auto const &dxl : dynamixel_)
                {
                    result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                                   control_items_["Present_Position"]->address,
                                                   length_of_data,
                                                   get_all_data,
                                                   &log);
                    if (result == false)
                    {
                        ROS_ERROR("%s", log);
                    }

                    dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
                    dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
                    dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

                    dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
                    dxl_cnt++;
                }
            }

#ifndef DEBUG
        }
#endif
        grp_cnt++;
    }

#ifdef DEBUG
    ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
    priv_read_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::publishCallback(const ros::TimerEvent &)
{
#ifdef DEBUG
    static double priv_pub_secs = ros::Time::now().toSec();
#endif
    dynamixel_state_list_pub_.publish(dynamixel_state_list_);

    if (is_joint_state_topic_)
    {
        joint_state_msg_.header.stamp = ros::Time::now();

        joint_state_msg_.name.clear();
        joint_state_msg_.position.clear();
        joint_state_msg_.velocity.clear();
        joint_state_msg_.effort.clear();

        uint8_t id_cnt = 0;
        for (auto const &dxl : dynamixel_)
        {
            double position = 0.0;
            double velocity = 0.0;
            double effort = 0.0;

            joint_state_msg_.name.push_back(dxl.first);

            if (dxl_wb_->getProtocolVersion() == 2.0f)
            {
                if (strcmp(dxl_wb_->getModelName((uint8_t)dxl.second), "XL-320") == 0)
                    effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
                else
                    effort = dxl_wb_->convertValue2Current((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
            }
            else if (dxl_wb_->getProtocolVersion() == 1.0f)
                effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);

            velocity = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity);
            position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_position);

            joint_state_msg_.effort.push_back(effort);
            joint_state_msg_.velocity.push_back(velocity);
            joint_state_msg_.position.push_back(position);

            id_cnt++;
        }

        joint_states_pub_.publish(joint_state_msg_);
    }

#ifdef DEBUG
    ROS_WARN("[publishCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
    priv_pub_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    bool result = false;
    const char *log = NULL;

    double wheel_velocity[dynamixel_.size()];
    int32_t dynamixel_velocity[dynamixel_.size()];

    const uint8_t LEFT = 0;
    const uint8_t RIGHT = 1;

    double robot_lin_vel = msg->linear.x;
    double robot_ang_vel = msg->angular.z;

    uint8_t id_array[dynamixel_.size()];
    uint8_t id_cnt = 0;

    float rpm = 0.0;
    for (auto const &dxl : dynamixel_)
    {
        const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
        rpm = modelInfo->rpm;
        id_array[id_cnt++] = (uint8_t)dxl.second;
    }

    //  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
    //       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

    double velocity_constant_value = 1 / (wheel_radius_ * rpm * 0.10472);

    wheel_velocity[LEFT] = robot_lin_vel - (robot_ang_vel * wheel_separation_ / 2);
    wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * wheel_separation_ / 2);

    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
        if (strcmp(dxl_wb_->getModelName(id_array[0]), "XL-320") == 0)
        {
            if (wheel_velocity[LEFT] == 0.0f)
                dynamixel_velocity[LEFT] = 0;
            else if (wheel_velocity[LEFT] < 0.0f)
                dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
            else if (wheel_velocity[LEFT] > 0.0f)
                dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

            if (wheel_velocity[RIGHT] == 0.0f)
                dynamixel_velocity[RIGHT] = 0;
            else if (wheel_velocity[RIGHT] < 0.0f)
                dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
            else if (wheel_velocity[RIGHT] > 0.0f)
                dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
        }
        else
        {
            dynamixel_velocity[LEFT] = wheel_velocity[LEFT] * velocity_constant_value;
            dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT] * velocity_constant_value;
        }
    }
    else if (dxl_wb_->getProtocolVersion() == 1.0f)
    {
        if (wheel_velocity[LEFT] == 0.0f)
            dynamixel_velocity[LEFT] = 0;
        else if (wheel_velocity[LEFT] < 0.0f)
            dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
        else if (wheel_velocity[LEFT] > 0.0f)
            dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

        if (wheel_velocity[RIGHT] == 0.0f)
            dynamixel_velocity[RIGHT] = 0;
        else if (wheel_velocity[RIGHT] < 0.0f)
            dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
        else if (wheel_velocity[RIGHT] > 0.0f)
            dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
    }

    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
    }
}

void DynamixelController::writeCallback(const ros::TimerEvent &)
{
#ifdef DEBUG
    static double priv_pub_secs = ros::Time::now().toSec();
#endif

    bool result = false;
    const char *log = NULL;

    int32_t dynamixel_position[dynamixel_.size()];

    static uint32_t point_cnt = 0;
    static uint32_t position_cnt = 0;

    int num_points = jnt_tra_msg_->points.size();
    if (num_points > 0)
    {
        ROS_INFO("[WRITE]: Processing point %i/%i...", point_cnt + 1, num_points);
    }

    uint8_t grp_num = 0;
    uint8_t pos_index = 0;
    for (auto grp : joint_groups_)
    {
        auto id_array = group_id_map_[grp];
        auto group_control_item = group_control_items_[grp];
        uint8_t id_cnt = id_array.size();

        if (is_moving_ == true)
        {
            for (auto index : id_array)
            {
                dynamixel_position[index - 1] = dxl_wb_->convertRadian2Value(index, jnt_tra_msg_->points[point_cnt].positions.at(index - 1));
            }

            result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION + grp_num * 2, &id_array[0], id_cnt, &dynamixel_position[pos_index], 1, &log);
            pos_index += id_cnt;
            if (result == false)
            {
                ROS_ERROR("%s", log);
            }
        }
        grp_num++;
    }

    position_cnt++;
    if (jnt_tra_msg_->points.size() == 0)
    {
        is_moving_ = false;
        point_cnt = 0;
        position_cnt = 0;
        return;
    }

    if (position_cnt >= jnt_tra_msg_->points[point_cnt].positions.size())
    {
        point_cnt++;
        position_cnt = 0;
        if (point_cnt >= jnt_tra_msg_->points.size())
        {
            is_moving_ = false;
            point_cnt = 0;
            position_cnt = 0;
            jnt_tra_msg_->points.clear();
            ROS_INFO("Trajectory complete.");
        }
    }

#ifdef DEBUG
    ROS_WARN("[writeCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
    priv_pub_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::moveJoints(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    uint8_t id_cnt = 0;
    bool result = false;
    WayPoint wp;

    if (is_moving_ == false)
    {
        jnt_tra_msg_->joint_names.clear();
        jnt_tra_msg_->points.clear();
        pre_goal_.clear();

        result = getPresentPosition(goal->trajectory.joint_names);
        if (result == false)
            ROS_ERROR("Failed to get Present Position");

        for (auto const &joint : goal->trajectory.joint_names)
        {
            ROS_INFO("'%s' is ready to move", joint.c_str());

            jnt_tra_msg_->joint_names.push_back(joint);
            id_cnt++;
        }

        if (id_cnt != 0)
        {
            uint8_t cnt = 0;
            while (cnt < goal->trajectory.points.size())
            {
                std::vector<WayPoint> dxlgoal;
                for (std::vector<int>::size_type id_num = 0; id_num < goal->trajectory.points[cnt].positions.size(); id_num++)
                {
                    wp.position = goal->trajectory.points[cnt].positions.at(id_num);

                    if (goal->trajectory.points[cnt].velocities.size() != 0)
                        wp.velocity = goal->trajectory.points[cnt].velocities.at(id_num);
                    else
                        wp.velocity = 0.0f;

                    if (goal->trajectory.points[cnt].accelerations.size() != 0)
                        wp.acceleration = goal->trajectory.points[cnt].accelerations.at(id_num);
                    else
                        wp.acceleration = 0.0f;

                    dxlgoal.push_back(wp);
                }

                jnt_tra_->setJointNum((uint8_t)goal->trajectory.points[cnt].positions.size());

                double move_time = 0.0f;
                if (cnt == 0)
                    move_time = goal->trajectory.points[cnt].time_from_start.toSec();
                else
                    move_time = goal->trajectory.points[cnt].time_from_start.toSec() - goal->trajectory.points[cnt - 1].time_from_start.toSec();

                jnt_tra_->init(move_time,
                               write_period_,
                               pre_goal_,
                               dxlgoal);

                std::vector<WayPoint> way_point;
                trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

                for (double index = 0.0; index < move_time; index = index + write_period_)
                {
                    way_point = jnt_tra_->getJointWayPoint(index);

                    for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
                    {
                        jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
                        jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
                        jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
                    }

                    jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
                    jnt_tra_point_msg.positions.clear();
                    jnt_tra_point_msg.velocities.clear();
                    jnt_tra_point_msg.accelerations.clear();
                }

                pre_goal_ = dxlgoal;
                cnt++;
            }
            ROS_INFO("Succeeded to get joint trajectory!");
            is_moving_ = true;

            auto result = new control_msgs::FollowJointTrajectoryResult();
            result->error_code = result->SUCCESSFUL;

            follow_traj_server->setSucceeded(*result);
        }
        else
        {
            ROS_WARN("Please check joint_name");

            auto result = new control_msgs::FollowJointTrajectoryResult();
            result->error_code = result->INVALID_JOINTS;

            follow_traj_server->setSucceeded(*result);
        }
    }
    else
    {
        ROS_INFO_THROTTLE(1, "Dynamixel is moving");

        auto result = new control_msgs::FollowJointTrajectoryResult();
        result->error_code = result->INVALID_GOAL;

        follow_traj_server->setSucceeded(*result);
    }
}

void DynamixelController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    ROS_INFO("[DYNACONTROL]: Joint trajectory recieved.");

    uint8_t id_cnt = 0;
    bool result = false;
    WayPoint wp;

    if (is_moving_ == false)
    {
        ROS_INFO("[DYNACONTROL]: Joints are ready to move.");
        jnt_tra_msg_->joint_names.clear();
        jnt_tra_msg_->points.clear();
        pre_goal_.clear();

        result = getPresentPosition(msg->joint_names);
        if (result == false)
            ROS_ERROR("Failed to get Present Position");

        for (auto const &dxl : dynamixel_)
        {
            auto name = dxl.first;
            ROS_INFO("'%s' is ready to move", name.c_str());

            jnt_tra_msg_->joint_names.push_back(name);
            id_cnt++;
        }

        // for (auto const &joint : msg->joint_names)
        // {
        //     ROS_INFO("'%s' is ready to move", joint.c_str());

        //     jnt_tra_msg_->joint_names.push_back(joint);
        //     id_cnt++;
        // }

        if (id_cnt != 0)
        {
            ROS_INFO("[DYNACONTROL]: Processing trajectory points...");
            uint8_t cnt = 0;
            while (cnt < msg->points.size())
            {
                std::vector<WayPoint> goal;
                for (std::vector<int>::size_type id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
                {
                    wp.position = msg->points[cnt].positions.at(id_num);

                    if (msg->points[cnt].velocities.size() != 0)
                        wp.velocity = msg->points[cnt].velocities.at(id_num);
                    else
                        wp.velocity = 0.0f;

                    if (msg->points[cnt].accelerations.size() != 0)
                        wp.acceleration = msg->points[cnt].accelerations.at(id_num);
                    else
                        wp.acceleration = 0.0f;

                    goal.push_back(wp);
                }

                ROS_INFO("[DYNACONTROL]: Goal created.");

                ROS_INFO("[DYNACONTROL]: Calculating trajectory...");
                jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());

                double move_time = 0.0f;
                if (cnt == 0)
                    move_time = msg->points[cnt].time_from_start.toSec();
                else
                    move_time = msg->points[cnt].time_from_start.toSec() - msg->points[cnt - 1].time_from_start.toSec();

                jnt_tra_->init(move_time,
                               write_period_,
                               pre_goal_,
                               goal);

                std::vector<WayPoint> way_point;
                trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

                ROS_INFO("[DYNACONTROL]: Pushing waypoints to trajectory....");

                for (double index = 0.0; index < move_time; index = index + write_period_)
                {
                    way_point = jnt_tra_->getJointWayPoint(index);

                    for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
                    {
                        // Check if joint is in control group
                        bool ingroup = false;
                        for (auto name : msg->joint_names)
                        {
                            if (name == id2name_[id_num + 1])
                            {
                                ROS_INFO("Joint%i is being controlled.", id_num);
                                ingroup = true;
                                break;
                            }
                        }

                        if (ingroup)
                        {
                            jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
                            jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
                            jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
                        }
                        else
                        {
                            jnt_tra_point_msg.positions.push_back(dynamixel_state_list_.dynamixel_state[id_num].present_position);
                            jnt_tra_point_msg.velocities.push_back(0);
                            jnt_tra_point_msg.accelerations.push_back(0);
                        }
                        // jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
                        // jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
                        // jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
                    }

                    jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
                    jnt_tra_point_msg.positions.clear();
                    jnt_tra_point_msg.velocities.clear();
                    jnt_tra_point_msg.accelerations.clear();
                }

                pre_goal_ = goal;
                cnt++;

                ROS_INFO("[DYNACONTROL]: Joint trajectory loaded.");
            }
            ROS_INFO("Succeeded to get joint trajectory!");
            is_moving_ = true;
        }
        else
        {
            ROS_WARN("Please check joint_name");
        }
    }
    else
    {
        ROS_INFO_THROTTLE(1, "Dynamixel is moving");
    }
}

bool DynamixelController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                      dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
    bool result = false;
    const char *log;

    uint8_t id = req.id;
    std::string item_name = req.addr_name;
    int32_t value = req.value;

    result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
    }

    res.comm_result = result;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_workbench_controllers");
    ros::NodeHandle node_handle("");

    std::string port_name = "/dev/ttyUSB0";
    uint32_t baud_rate = 57600;
    std::string controller_name = "arm";

    if (argc < 2)
    {
        ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
        return 0;
    }
    else
    {
        port_name = argv[1];
        baud_rate = atoi(argv[2]);
        controller_name = argv[3];
    }

    DynamixelController dynamixel_controller;

    bool result = false;

    std::string yaml_file = node_handle.param<std::string>(controller_name + "/dynamixel_info", "");

    ROS_INFO("Initializing workbench...");
    result = dynamixel_controller.initWorkbench(port_name, baud_rate);
    if (result == false)
    {
        ROS_ERROR("Please check USB port name");
        return 0;
    }

    ROS_INFO("Retrieving Dynamixel info...");
    result = dynamixel_controller.getDynamixelsInfo(yaml_file);
    if (result == false)
    {
        ROS_ERROR("Please check YAML file");
        return 0;
    }

    ROS_INFO("Loading dynamixels...");
    result = dynamixel_controller.loadDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return 0;
    }

    ROS_INFO("Initializing dynamixels...");
    result = dynamixel_controller.initDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return 0;
    }

    ROS_INFO("Initializing Control Items...");
    result = dynamixel_controller.initControlItems();
    if (result == false)
    {
        ROS_ERROR("Please check control items");
        return 0;
    }

    ROS_INFO("Initializing SDK Handlers...");
    result = dynamixel_controller.initSDKHandlers();
    if (result == false)
    {
        ROS_ERROR("Failed to set Dynamixel SDK Handler");
        return 0;
    }

    ROS_INFO("Starting publisher...");
    dynamixel_controller.initPublisher();
    ROS_INFO("Starting subscriber...");
    dynamixel_controller.initSubscriber();
    ROS_INFO("Starting server...");
    dynamixel_controller.initServer();
    ROS_INFO("Starting action server...");
    //dynamixel_controller.initFollowJointTrajectoryActionServer();

    ROS_INFO("Starting timers...");
    ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()), &DynamixelController::readCallback, &dynamixel_controller);
    ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()), &DynamixelController::writeCallback, &dynamixel_controller);
    ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), &DynamixelController::publishCallback, &dynamixel_controller);

    ROS_INFO("Spinning node...");
    ros::spin();

    return 0;
}

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>

#include "math.h"
#include "nubot_msgs/base_info.h"
#include "nubot_msgs/base_drive_cmd.h"

class state_publish
{
public:
	state_publish(int argc,char** argv,const char* name);
    //void publisher(const ros::TimerEvent& event);

	void Callback_Pumbaa(const nubot_msgs::base_info &base_info);
    void Callback_Pumbaa_Cmd(const nubot_msgs::base_drive_cmd &base_drive_cmd);
    void Callback_Vive(const geometry_msgs::PoseStamped &vive_tracker);
    void Callback_Gazebo_robot(const geometry_msgs::PoseStamped &gazebo_robot);
    void Callback_Imu(const sensor_msgs::Imu &imu_data);
	// void Callback_Cobra(const nubot_msgs::angle_position &cobra);
    //void Callback_Cobra3(const nubot_msgs::cobra3_angle_position &cobra3); //2022/0314

private:
	ros::Subscriber pumbaa_sub;
    ros::Subscriber vive_sub;
    ros::Subscriber gazebo_sub;
    ros::Subscriber imu_sub;
	// ros::Subscriber cobra_sub;

	ros::Publisher rviz_state_pub;
        //ros::Timer cmd_timer;
    
    const int model_joint_num = 4;
    float pumbaa_flipper_angle[4] = {0};
    float pumbaa_flipper_cmd[4] = {0};
    // float cobra_angle[12]={0};

    Eigen::Quaterniond q_lidar_pumbaa_sta, q_lidar_pumbaa_cur, q_vive_pumbaa_sta, q_map_pumbaa_cur;
    Eigen::Vector3d t_lidar_pumbaa_sta, t_lidar_pumbaa_cur, t_vive_pumbaa_sta, t_map_pumbaa_cur;
};

static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x()); // right-hand frame
	// double sinp = -2.0 * (q.w() * q.y() - q.z() * q.x()); // left-hand frame
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	yaw = atan2(siny_cosp, cosy_cosp);
}

void state_publish::Callback_Pumbaa(const nubot_msgs::base_info &base_info)
{
	// ###### 电机编号示意图 ######
    // # CAN-ID对应+1
    // #     |——|              |——|
    // #   2 |——|    front     |——| 4
    // #     |——|              |——|
    // #       |——————|  |——————|
    // #     0 |——————|  |——————| 1
    // #       |——————|  |——————|
    // #       |——————|  |——————|
    // #     |——|              |——|
    // #   3 |——|     back     |——| 5
    // #     |——|    /rear     |——|
    // # 速度正负：前进+ 后退- 上抬+ 下压-
    // # 摆臂角度，对应编号-2
    pumbaa_flipper_angle[0]=float(base_info.fin_angle[0])/180*M_PI;
	pumbaa_flipper_angle[1]=float(base_info.fin_angle[1])/180*M_PI*-1;
	pumbaa_flipper_angle[2]=float(base_info.fin_angle[2])/180*M_PI*-1;
	pumbaa_flipper_angle[3]=float(base_info.fin_angle[3])/180*M_PI;
}

void state_publish::Callback_Pumbaa_Cmd(const nubot_msgs::base_drive_cmd &base_drive_cmd)
{
    pumbaa_flipper_cmd[0]=float(base_drive_cmd.fin_expect[0])/180*M_PI;
	pumbaa_flipper_cmd[1]=float(base_drive_cmd.fin_expect[1])/180*M_PI*-1;
	pumbaa_flipper_cmd[2]=float(base_drive_cmd.fin_expect[2])/180*M_PI*-1;
	pumbaa_flipper_cmd[3]=float(base_drive_cmd.fin_expect[3])/180*M_PI;
}

void state_publish::Callback_Vive(const geometry_msgs::PoseStamped &vive_tracker)
{
    Eigen::Vector3d vive_pos(vive_tracker.pose.position.x, vive_tracker.pose.position.y, vive_tracker.pose.position.z);
    Eigen::Quaterniond vive_quaternion_e(vive_tracker.pose.orientation.w, vive_tracker.pose.orientation.x, vive_tracker.pose.orientation.y, vive_tracker.pose.orientation.z);
    t_map_pumbaa_cur = vive_pos + vive_quaternion_e * t_vive_pumbaa_sta;
    q_map_pumbaa_cur = vive_quaternion_e * q_vive_pumbaa_sta;
}

void state_publish::Callback_Gazebo_robot(const geometry_msgs::PoseStamped &gazebo_robot)
{
    Eigen::Vector3d robot_pos(gazebo_robot.pose.position.x, gazebo_robot.pose.position.y, gazebo_robot.pose.position.z);
    Eigen::Quaterniond robot_quaternion_e(gazebo_robot.pose.orientation.w, gazebo_robot.pose.orientation.x, gazebo_robot.pose.orientation.y, gazebo_robot.pose.orientation.z);

    t_map_pumbaa_cur = robot_pos;
    q_map_pumbaa_cur = robot_quaternion_e;
}

void state_publish::Callback_Imu(const sensor_msgs::Imu &imu_data)
{
    Eigen::Quaterniond imu_quaternion_e(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);

    // remove imu_yaw
    double roll, pitch, yaw;
    toEulerAngle(imu_quaternion_e, roll, pitch, yaw);//手动四元数转欧拉角计算
    Eigen::AngleAxisd rollAngle(roll,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0,Eigen::Vector3d::UnitZ());//eulerAngle(0)
    imu_quaternion_e = yawAngle*pitchAngle*rollAngle;
    // printf("pitch:%f\n",pitch);
    t_lidar_pumbaa_cur = imu_quaternion_e * t_lidar_pumbaa_sta;
    q_lidar_pumbaa_cur = imu_quaternion_e * q_lidar_pumbaa_sta;
}

// void state_publish::Callback_Cobra(const nubot_msgs::angle_position &cobra)
// {
//     cobra_angle[0] = -float(cobra.arm[0])/180*3.1416 + 1.5708;
//     cobra_angle[1] = float(cobra.arm[1])/180*3.1416;
//     cobra_angle[2] = -float(cobra.arm[2])/180*3.1416 + 1.5708;
//     cobra_angle[3] = -float(cobra.arm[3])/180*3.1416;
//     cobra_angle[4] = -float(cobra.arm[4])/180*3.1416;
//     cobra_angle[5] = -float(cobra.arm[5])/180*3.1416;
// 	// cobra_angle.arm[6] = float(cobra.arm[6])/180*3.1416;

//     cobra_angle[6] = -float(cobra.arm[6])/180*3.1416 + 1.5708;
//     cobra_angle[7] = float(cobra.arm[7])/180*3.1416;
//     cobra_angle[8] = -float(cobra.arm[8])/180*3.1416 + 1.5708;
//     cobra_angle[9] = -float(cobra.arm[9])/180*3.1416;
//     cobra_angle[10] = -float(cobra.arm[10])/180*3.1416;
//     cobra_angle[11] = -float(cobra.arm[11])/180*3.1416;
// }

state_publish::state_publish(int argc,char** argv,const char* name)
{
    /***ROS***/
    ros::init(argc,argv,name);
    ros::NodeHandle nh;
    
    std::string base_info_sub_name, imu_sub_name, pose_sub_name, LidarYawFrame;
    bool is_gt = false;
    int lidar_pumbaa_dir = 1, vive_pumbaa_dir = 1;
    std::vector<double> lidar_pumbaa_trans = {0,0,0}, vive_pumbaa_trans = {0,0,0};

    if(nh.getParam("base_info_sub_name", base_info_sub_name))
        ROS_INFO("base_info_sub_name is %s", base_info_sub_name.c_str());
    else
        ROS_WARN("didn't find parameter base_info_sub_name");

    if(nh.getParam("imu_sub_name", imu_sub_name))
        ROS_INFO("imu_sub_name is %s", imu_sub_name.c_str());
    else
        ROS_WARN("didn't find parameter imu_sub_name");

    if(nh.getParam("LidarYawFrame", LidarYawFrame))
        ROS_INFO("LidarYawFrame is %s", LidarYawFrame.c_str());
    else
        ROS_WARN("didn't find parameter LidarYawFrame");

    if(nh.getParam("lidar_pumbaa_trans", lidar_pumbaa_trans))
    {
        ROS_INFO("get lidar_pumbaa_trans:");
        for (int i=0; i<lidar_pumbaa_trans.size(); i++)
        {
            t_lidar_pumbaa_sta[i] = lidar_pumbaa_trans[i];
            std::cout << lidar_pumbaa_trans[i] << " ";
        }
        std::cout << std::endl;
    }
    else
        ROS_WARN("didn't find parameter lidar_pumbaa_trans");

    if(nh.getParam("lidar_pumbaa_dir", lidar_pumbaa_dir))
    {
        if(lidar_pumbaa_dir>0)
            q_lidar_pumbaa_sta = {1,0,0,0}; // w,x,y,z
        else
            q_lidar_pumbaa_sta = {0,0,0,1}; // w,x,y,z
        ROS_INFO("lidar_pumbaa_dir is %d", lidar_pumbaa_dir);
    }
    else
        ROS_WARN("didn't find parameter lidar_pumbaa_dir");

    // t_lidar_pumbaa_sta.x() = -0.36277;//平移部分赋值
    // t_lidar_pumbaa_sta.y() = 0.0;
    // t_lidar_pumbaa_sta.z() = -0.2115;
    // q_lidar_pumbaa_sta = {0,0,0,1}; // w,x,y,z

    if(nh.getParam("is_gt", is_gt))
        ROS_INFO("is_gt is %s", is_gt?"true":"false");
    else
        ROS_WARN("didn't find parameter is_gt");

    if(nh.getParam("pose_sub_name", pose_sub_name))
        ROS_INFO("pose_sub_name is %s", pose_sub_name.c_str());
    else
        ROS_WARN("didn't find parameter pose_sub_name");

    if(nh.getParam("vive_pumbaa_trans", vive_pumbaa_trans))
    {
        ROS_INFO("get vive_pumbaa_trans:");
        for (int i=0; i<vive_pumbaa_trans.size(); i++)
        {
            t_vive_pumbaa_sta[i] = vive_pumbaa_trans[i];
            std::cout << vive_pumbaa_trans[i] << " ";
        }
        std::cout << std::endl;
    }
    else
        ROS_WARN("didn't find parameter vive_pumbaa_trans");

    if(nh.getParam("vive_pumbaa_dir", vive_pumbaa_dir))
    {
        if(vive_pumbaa_dir>0)
            q_vive_pumbaa_sta = {1,0,0,0}; // w,x,y,z
        else
            q_vive_pumbaa_sta = {0,0,0,1}; // w,x,y,z
        ROS_INFO("vive_pumbaa_dir is %d", vive_pumbaa_dir);
    }
    else
        ROS_WARN("didn't find parameter vive_pumbaa_dir");

    // publisher
    rviz_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

    // subscriber
    pumbaa_sub = nh.subscribe(base_info_sub_name, 1,  &state_publish::Callback_Pumbaa, this);
    // pumbaa_sub = nh.subscribe("/nubot_drive/base_drive_cmd", 1,  &state_publish::Callback_Pumbaa_Cmd, this);
    imu_sub = nh.subscribe(imu_sub_name, 1,  &state_publish::Callback_Imu, this); 
    
    // tf lidar_yaw -> base_link
    geometry_msgs::TransformStamped tfLidarYawBase_msg;
    tfLidarYawBase_msg.header.frame_id = LidarYawFrame;
    tfLidarYawBase_msg.child_frame_id = "base_link";//因为urdf模型中base_link为parent，所以只能发布base_link的tf才能在rviz里正常显示
    tf::TransformBroadcaster tfLidarYawBase_br;
    
    // tf map -> base_link
    geometry_msgs::TransformStamped tfMapBase_msg;
    tf::TransformBroadcaster tfMapBase_br;
    if (is_gt)
    {
        tfMapBase_msg.header.frame_id = "map";
        tfMapBase_msg.child_frame_id = "base_link";//因为urdf模型中base_link为parent，所以只能发布base_link的tf才能在rviz里正常显示
        // vive_sub = nh.subscribe(pose_sub_name, 1,  &state_publish::Callback_Vive, this);
        gazebo_sub = nh.subscribe(pose_sub_name, 1,  &state_publish::Callback_Gazebo_robot, this);
    }

    // ros::Subscriber cobra_sub = n.subscribe("/timon_angle", 1,  &state_publish::Callback_Cobra, this);

    sensor_msgs::JointState current_pos_msg;
    current_pos_msg.name.resize(model_joint_num);
    current_pos_msg.position.resize(model_joint_num);
    current_pos_msg.name[0] = "front_left_flipper_joint"; 
    current_pos_msg.name[1] = "rear_left_flipper_joint";
    current_pos_msg.name[2] = "front_right_flipper_joint";
    current_pos_msg.name[3] = "rear_right_flipper_joint";

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        current_pos_msg.header.stamp =ros::Time::now();

        // "front_left_fin_joint"
        current_pos_msg.position[0] = pumbaa_flipper_angle[0];
        // "back_left_fin_joint"
        current_pos_msg.position[1] = pumbaa_flipper_angle[1];
        // "front_right_fin_joint"
        current_pos_msg.position[2] = pumbaa_flipper_angle[2];
        // "back_right_fin_joint"
        current_pos_msg.position[3] = pumbaa_flipper_angle[3];

        rviz_state_pub.publish(current_pos_msg);

        // publish the tf lidar_yaw -> base_link
        tfLidarYawBase_msg.header.stamp = ros::Time::now();
        tfLidarYawBase_msg.transform.translation.x = t_lidar_pumbaa_cur.x();
        tfLidarYawBase_msg.transform.translation.y = t_lidar_pumbaa_cur.y();
        tfLidarYawBase_msg.transform.translation.z = t_lidar_pumbaa_cur.z();
        tfLidarYawBase_msg.transform.rotation.w = q_lidar_pumbaa_cur.w();
        tfLidarYawBase_msg.transform.rotation.x = q_lidar_pumbaa_cur.x();
        tfLidarYawBase_msg.transform.rotation.y = q_lidar_pumbaa_cur.y();
        tfLidarYawBase_msg.transform.rotation.z = q_lidar_pumbaa_cur.z();
        tfLidarYawBase_br.sendTransform(tfLidarYawBase_msg);

        if (is_gt)
        {
            // publish the tf map -> base_link
            tfMapBase_msg.header.stamp = ros::Time::now();
            tfMapBase_msg.transform.translation.x = t_map_pumbaa_cur.x();
            tfMapBase_msg.transform.translation.y = t_map_pumbaa_cur.y();
            tfMapBase_msg.transform.translation.z = t_map_pumbaa_cur.z();
            tfMapBase_msg.transform.rotation.w = q_map_pumbaa_cur.w();
            tfMapBase_msg.transform.rotation.x = q_map_pumbaa_cur.x();
            tfMapBase_msg.transform.rotation.y = q_map_pumbaa_cur.y();
            tfMapBase_msg.transform.rotation.z = q_map_pumbaa_cur.z();
            tfMapBase_br.sendTransform(tfLidarYawBase_msg);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    for(int i=0; i<model_joint_num; i++)
    {
        current_pos_msg.position[i]=0;
    }

    rviz_state_pub.publish(current_pos_msg);
}


int main(int argc, char **argv)
{
	state_publish node(argc,argv,"rviz_state_publish");
    return 0;
}
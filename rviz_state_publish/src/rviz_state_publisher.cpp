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
    
    // double a2=0,a3=0,a2b=0,a3b=0,a6=0,d6=0,a6b=0,d6b=0;

private:
	ros::Subscriber pumbaa_sub;
    // ros::Subscriber vive_sub;
    ros::Subscriber gazebo_sub;
    ros::Subscriber imu_sub;
	// ros::Subscriber cobra_sub;
	ros::Publisher rviz_state_pub;
        //ros::Timer cmd_timer;
    float angle[20]={0};
    sensor_msgs::JointState current_pos;

    geometry_msgs::Quaternion base_quaternion, vive_quaternion, imu_quaternion;
    Eigen::Quaterniond base_quaternion_e, static_q_lidar_base;
    Eigen::Vector3d real_t_lidar_base, static_t_lidar_base;
    float vive_x=0, vive_y=0, vive_z=0;
    Eigen::Matrix4d vive_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d base_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d vive_base_matrix = Eigen::Matrix4d::Identity();
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
    angle[0]=float(base_info.fin_angle[0])/180*M_PI;
	angle[1]=float(base_info.fin_angle[1])/180*M_PI*-1;
	angle[2]=float(base_info.fin_angle[2])/180*M_PI*-1;
	angle[3]=float(base_info.fin_angle[3])/180*M_PI;
}

void state_publish::Callback_Pumbaa_Cmd(const nubot_msgs::base_drive_cmd &base_drive_cmd)
{
    angle[0]=float(base_drive_cmd.fin_expect[0])/180*M_PI;
	angle[1]=float(base_drive_cmd.fin_expect[1])/180*M_PI*-1;
	angle[2]=float(base_drive_cmd.fin_expect[2])/180*M_PI*-1;
	angle[3]=float(base_drive_cmd.fin_expect[3])/180*M_PI;
}

void state_publish::Callback_Vive(const geometry_msgs::PoseStamped &vive_tracker)
{
    // double roll, pitch, yaw;//定义存储r\p\y的容器
    vive_x = vive_tracker.pose.position.x;
    vive_y = vive_tracker.pose.position.y;
    vive_z = vive_tracker.pose.position.z;
    vive_quaternion = vive_tracker.pose.orientation;
    Eigen::Quaterniond vive_quaternion_e(vive_quaternion.w, vive_quaternion.x, vive_quaternion.y, vive_quaternion.z);
    vive_matrix.block<3,3>(0,0) = vive_quaternion_e.toRotationMatrix();//旋转部分赋值
    vive_matrix.block<3,1>(0,3) = Eigen::Vector3d(vive_x, vive_y, vive_z); //平移部分赋值
    base_matrix = vive_matrix * vive_base_matrix;//计算base_link位置
    base_quaternion = vive_quaternion;
}

void state_publish::Callback_Gazebo_robot(const geometry_msgs::PoseStamped &gazebo_robot)
{
    // double roll, pitch, yaw;//定义存储r\p\y的容器
    base_quaternion = gazebo_robot.pose.orientation;
    Eigen::Quaterniond base_quaternion_e(base_quaternion.w, base_quaternion.x, base_quaternion.y, base_quaternion.z);

    // remove imu_yaw
    double roll, pitch, yaw;
    toEulerAngle(base_quaternion_e, roll, pitch, yaw);//手动四元数转欧拉角计算
    Eigen::AngleAxisd rollAngle(roll,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0,Eigen::Vector3d::UnitZ());//eulerAngle(0)
    base_quaternion_e = yawAngle*pitchAngle*rollAngle;
    // printf("pitch:%f\n",pitch);

    // base_quaternion_e = base_quaternion_e * static_q_lidar_base;
    real_t_lidar_base = base_quaternion_e * static_t_lidar_base;
    base_quaternion.w = base_quaternion_e.w();
    base_quaternion.x = base_quaternion_e.x();
    base_quaternion.y = base_quaternion_e.y();
    base_quaternion.z = base_quaternion_e.z();

    // base_matrix.block<3,3>(0,0) = base_quaternion_e.toRotationMatrix();//旋转部分赋值
    // base_matrix.block<3,1>(0,3) = Eigen::Vector3d(gazebo_robot.pose.position.x, gazebo_robot.pose.position.y, gazebo_robot.pose.position.z); //平移部分赋值
}

void state_publish::Callback_Imu(const sensor_msgs::Imu &imu_data)
{
    // double roll, pitch, yaw;//定义存储r\p\y的容器
    imu_quaternion = imu_data.orientation;
    Eigen::Quaterniond imu_quaternion_e(imu_quaternion.w, imu_quaternion.x, imu_quaternion.y, imu_quaternion.z);

    // remove imu_yaw
    double roll, pitch, yaw;
    toEulerAngle(imu_quaternion_e, roll, pitch, yaw);//手动四元数转欧拉角计算
    //因为gazebo的问题，urdf中修改IMU朝向无效，IMU坐标系与机器人坐标系相同，这里手动转换，给roll和pitch加负号
    Eigen::AngleAxisd rollAngle(-roll,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-pitch,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0,Eigen::Vector3d::UnitZ());
    imu_quaternion_e = yawAngle*pitchAngle*rollAngle;
    // printf("pitch:%f\n",pitch);
    
    base_quaternion_e = imu_quaternion_e * static_q_lidar_base;
    real_t_lidar_base = base_quaternion_e * static_t_lidar_base;
    base_quaternion.w = base_quaternion_e.w();
    base_quaternion.x = base_quaternion_e.x();
    base_quaternion.y = base_quaternion_e.y();
    base_quaternion.z = base_quaternion_e.z();
}

// void state_publish::Callback_Cobra(const nubot_msgs::angle_position &cobra)
// {
//     angle[4] = -float(cobra.arm[0])/180*3.1416 + 1.5708;
// 	angle[5] = float(cobra.arm[1])/180*3.1416;
//     angle[6] = -float(cobra.arm[2])/180*3.1416 + 1.5708;
// 	angle[7] = -float(cobra.arm[3])/180*3.1416;
// 	angle[8] = -float(cobra.arm[4])/180*3.1416;
// 	angle[9] = -float(cobra.arm[5])/180*3.1416;
// 	//angle.arm[6] = float(cobra.arm[6])/180*3.1416;

//     angle[10] = -float(cobra.arm[6])/180*3.1416 + 1.5708;
//     angle[11] = float(cobra.arm[7])/180*3.1416;
//     angle[12] = -float(cobra.arm[8])/180*3.1416 + 1.5708;
//     angle[13] = -float(cobra.arm[9])/180*3.1416;
//     angle[14] = -float(cobra.arm[10])/180*3.1416;
//     angle[15] = -float(cobra.arm[11])/180*3.1416;
// }

state_publish::state_publish(int argc,char** argv,const char* name)
{
    /***ROS***/
    ros::init(argc,argv,name);
    ros::NodeHandle n;

    // angle.angle[0]=0;angle.angle[1]=0;angle.angle[2]=0;angle.angle[3]=0;
    // angle.arm={0,0,0,0,0,0,0};
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "aft_mapped_yaw";
    odom_trans.child_frame_id = "base_link";//因为urdf模型中base_link为parent，所以只能发布base_link的tf才能在rviz里正常显示
    // vive_base_matrix(0,3) = -0.16175;//平移部分赋值
    // vive_base_matrix(1,3) = 0.0;
    // vive_base_matrix(2,3) = -0.138;
    // std::cout << "vive_base_matrix:" << vive_base_matrix.block<4,4>(0,0) << std::endl;
    static_t_lidar_base.x() = 0.36277;//平移部分赋值
    static_t_lidar_base.y() = 0.0;
    static_t_lidar_base.z() = -0.2115;
    static_q_lidar_base = {0,0,0,1}; // w,x,y,z
    tf::TransformBroadcaster broadcaster;

    rviz_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 100);
    pumbaa_sub = n.subscribe("/nubot_drive/base_info", 1,  &state_publish::Callback_Pumbaa, this);
    // pumbaa_sub = n.subscribe("/nubot_drive/base_drive_cmd", 1,  &state_publish::Callback_Pumbaa_Cmd, this);
    // vive_sub = n.subscribe("/vive/LHR_6E09A2CE_pose", 1,  &state_publish::Callback_Vive, this); //LHR_50EACBF2_pose LHR_FFADBD42_pose LHR_6E09A2CE_pose
    // gazebo_sub = n.subscribe("/gazebo_state/pumbaa_pose", 1,  &state_publish::Callback_Gazebo_robot, this); //LHR_50EACBF2_pose LHR_FFADBD42_pose LHR_6E09A2CE_pose
    imu_sub = n.subscribe("/imu/data", 1,  &state_publish::Callback_Imu, this); 

    // ros::Subscriber cobra_sub = n.subscribe("/timon_angle", 1,  &state_publish::Callback_Cobra, this);

    ros::Rate loop_rate(100);
    current_pos.name.resize(4);
    current_pos.position.resize(4);

    while (ros::ok())
    {
        ros::NodeHandle n;
        // n.param<double>("cobra_a2",a2,0.483);
        // n.param<double>("cobra_a3",a3,0.575);
        // n.param<double>("cobra3_a2",a2b,0.483);
        // n.param<double>("cobra3_a3",a3b,0.575);
        // n.param<double>("cobra_a6",a6,0.00);
        // n.param<double>("cobra_d6",d6,0.00);
        // n.param<double>("cobra3_a6",a6b,0.00);
        // n.param<double>("cobra3_d6",d6b,0.00);
        // a2 -= 0.50;
        // a3 = 0.89 - a3;
        // a2b -= 0.50;
        // a3b = 0.89 - a3b;
        current_pos.header.stamp =ros::Time::now();
        current_pos.name[0] = "front_left_flipper_joint"; 
        current_pos.position[0] = angle[0];

        current_pos.name[1] = "front_right_flipper_joint";
        current_pos.position[1] = angle[2];

        current_pos.name[2] = "rear_left_flipper_joint";
        current_pos.position[2] = angle[1];

        current_pos.name[3] = "rear_right_flipper_joint";
        current_pos.position[3] = angle[3];

        // current_pos.name[4] = "joint1_a";
        // current_pos.position[4] = angle[4];

        // current_pos.name[5] = "joint2_a";
        // current_pos.position[5] = angle[5];

        // current_pos.name[6] = "joint21_a";
        // current_pos.position[6] = a2;

        // current_pos.name[7] = "joint3_a";
        // current_pos.position[7] = angle[6];

        // current_pos.name[8] = "joint31_a";
        // current_pos.position[8] = a3;

        // current_pos.name[9] = "joint4_a";
        // current_pos.position[9] = angle[7];

        // current_pos.name[10] = "joint5_a";
        // current_pos.position[10] = angle[8];

        // current_pos.name[11] = "joint6_a";
        // current_pos.position[11] = angle[9];



        // current_pos.name[12] = "joint1_b";
        // current_pos.position[12] = angle[10];

        // current_pos.name[13] = "joint2_b";
        // current_pos.position[13] = angle[11];

        // current_pos.name[14] = "joint21_b";
        // current_pos.position[14] = a2b;

        // current_pos.name[15] = "joint3_b";
        // current_pos.position[15] = angle[12];

        // current_pos.name[16] = "joint31_b";
        // current_pos.position[16] = a3b;

        // current_pos.name[17] = "joint4_b";
        // current_pos.position[17] = angle[13];

        // current_pos.name[18] = "joint5_b";
        // current_pos.position[18] = angle[14];

        // current_pos.name[19] = "joint6_b";
        // current_pos.position[19] = angle[15];

        // current_pos.name[20] = "joint7_a";
        // current_pos.position[20] = d6 - 0.112;
        // current_pos.name[21] = "joint8_a";
        // current_pos.position[21] = a6;

        // current_pos.name[22] = "joint7_b";
        // current_pos.position[22] = d6b;
        // current_pos.name[23] = "joint8_b";
        // current_pos.position[23] = a6b;

        rviz_state_pub.publish(current_pos);

        //publish the transform over tf
        odom_trans.header.stamp = ros::Time::now();
        // odom_trans.transform.translation.x = base_matrix(0,3);
        // odom_trans.transform.translation.y = base_matrix(1,3);
        // odom_trans.transform.translation.z = base_matrix(2,3);
        odom_trans.transform.translation.x = real_t_lidar_base.x();
        odom_trans.transform.translation.y = real_t_lidar_base.y();
        odom_trans.transform.translation.z = real_t_lidar_base.z();
        
        odom_trans.transform.rotation = base_quaternion;//vive与base之间没有旋转变换关系，只有平移，因此直接复制即可
        //send the transform
        broadcaster.sendTransform(odom_trans);
        ros::spinOnce();
        loop_rate.sleep();
    }
    for(int i = 0 ; i <= 3 ; i++ )
    {
        current_pos.position[i]=0;
    }
//    current_pos.position[0]=0;current_pos.position[1]=0;current_pos.position[2]=0;current_pos.position[3]=0;current_pos.position[4]=0;
//    current_pos.position[5]=0;current_pos.position[6]=0;current_pos.position[7]=0;current_pos.position[8]=0;current_pos.position[9]=0;
    rviz_state_pub.publish(current_pos);
}


int main(int argc, char **argv)
{
	state_publish node(argc,argv,"rviz_state_publish");
    return 0;
}
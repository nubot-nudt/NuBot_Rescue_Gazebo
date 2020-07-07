#!/home/nubot/Documents/Study/PythonEnv/ros-py3-tf1/bin/python3.6
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose
from nubot_pumbaa_msg.msg import PumbaaCmd


def node_init():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global cmd_pub, rate, pumbaa_cmd, pumbaa_state
    pumbaa_cmd = PumbaaCmd()
    pumbaa_state = Pose()

    rospy.init_node('Nubot_Control', anonymous=True)
    rospy.Subscriber("Nubot_Pumbaa/nubotstate/robotstate", Pose, robotstate_CB)
    cmd_pub = rospy.Publisher('Nubot_Pumbaa/nubotcontrol/pumbaacmd', PumbaaCmd, queue_size=10)
    # rate = rospy.Rate(10)  # 10hz
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


def robotstate_CB(_msg):
    pumbaa_state = _msg

    pumbaa_cmd = control_al(pumbaa_state)

    rospy.loginfo("pumbaa_cmd_track %f %f", pumbaa_cmd.vel_linear, pumbaa_cmd.vel_angular)
    rospy.loginfo("pumbaa_cmd_flip %f %f %f %f", pumbaa_cmd.front_left, pumbaa_cmd.front_right, pumbaa_cmd.rear_left,
                  pumbaa_cmd.rear_right)
    cmd_pub.publish(pumbaa_cmd)


def control_al(robot_state):
    rospy.loginfo("control_al %f %f", robot_state.position.x, robot_state.position.z)
    robot_cmd = PumbaaCmd()
    robot_cmd.vel_linear = 0.5
    robot_cmd.vel_angular = 0
    robot_cmd.front_left = 1.5708
    robot_cmd.front_right = 1.5708
    robot_cmd.rear_left = 1.5708
    robot_cmd.rear_right = 1.5708
    return robot_cmd


if __name__ == '__main__':
    try:
        node_init()
        while not rospy.is_shutdown():
            # rospy.loginfo("pumbaa_state %f %f", pumbaa_state.position.x, pumbaa_state.orientation.x)
            # rate.sleep()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

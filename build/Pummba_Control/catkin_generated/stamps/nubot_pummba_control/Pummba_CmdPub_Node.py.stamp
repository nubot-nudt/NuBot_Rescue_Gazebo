#!/home/nubot/Documents/Study/PythonEnv/ros-py3-tf1/bin/python3.6
# license removed for brevity
import rospy
from nubot_pummba_msg.msg import PummbaCmd
import Pummba_StateSub_Node
import Pummba_Control
import gol

pummbapub = None
rate = None


def pummbacmd_pub_init():
    global pummbapub
    pummbapub = rospy.Publisher('Nubot_Pummba/nubotcontrol/pummbacmd', PummbaCmd, queue_size=10)
    rospy.init_node('PummbaCmd_pub_', anonymous=True)
    global rate
    rate = rospy.Rate(10)  # 10hz


if __name__ == '__main__':
    try:
        pummbacmd_pub_init()

        pummba_cmd = PummbaCmd()
        # pummba_pose = [1, 2, 3, 4, 5, 6, 7]

        while not rospy.is_shutdown():
            # pummba_pose = Pummba_StateSub_Node.getpose()
            pummba_pose = gol.get_value()
            rospy.loginfo("Robot_Pose_Control %f", pummba_pose[1])
            pummba_cmd.vel_linear = 0.5
            pummba_cmd.vel_angular = -0.5
            pummba_cmd.front_left = 1
            pummba_cmd.front_right = 1
            pummba_cmd.rear_left = 1
            pummba_cmd.rear_right = 1
            rospy.loginfo("pummba_cmd_track %f %f", pummba_cmd.vel_linear, pummba_cmd.vel_angular)
            rospy.loginfo("pummba_cmd_flip %f %f %f %f", pummba_cmd.front_left, pummba_cmd.front_right, pummba_cmd.rear_left,
                          pummba_cmd.rear_right)
            pummbapub.publish(pummba_cmd)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
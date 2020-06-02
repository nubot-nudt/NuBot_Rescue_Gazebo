#!/home/nubot/Documents/Study/PythonEnv/ros-py3-tf1/bin/python3.6
# license removed for brevity
import rospy
import gol
from geometry_msgs.msg import Pose

pummba_state = [1, 2, 3, 4, 5, 6, 7]


def callback(_msg):
    global pummba_state
    pummba_state[0] = _msg.position.x
    pummba_state[1] = _msg.position.y
    pummba_state[2] = _msg.position.z
    pummba_state[3] = _msg.orientation.x
    pummba_state[4] = _msg.orientation.y
    pummba_state[5] = _msg.orientation.z
    pummba_state[6] = _msg.orientation.w
    rospy.loginfo(rospy.get_caller_id() + " %f", pummba_state[1])
    gol.set_value(pummba_state)


def pummbastate_sub_():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('RobotState_sub_', anonymous=True)
    rospy.Subscriber("Nubot_Pummba/nubotstate/robotstate", Pose, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def getpose():
    global pummba_state
    rospy.loginfo("getpose %f", pummba_state[1])
    return pummba_state


if __name__ == '__main__':
    pummbastate_sub_()
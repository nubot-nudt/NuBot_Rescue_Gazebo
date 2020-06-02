#!/home/nubot/Documents/Study/PythonEnv/ros-py3-tf1/bin/python3.6
# license removed for brevity
import rospy


class GlobalVar:
    demo_value = None


def set_value(value):
    GlobalVar.demo_value= value
    rospy.loginfo("set_value %f", GlobalVar.demo_value[1])


def get_value():
    rospy.loginfo("get_value %f", GlobalVar.demo_value[1])
    return GlobalVar.demo_value
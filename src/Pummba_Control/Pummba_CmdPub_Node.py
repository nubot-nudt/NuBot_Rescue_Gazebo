#!/Home/nubot/Documents/Study/PythonEnv/ros-py3-tf1/bin/python3.6
# license removed for brevity
import rospy
from nubot_pummba_msg.msg import PummbaCmd

pummbapub = None
rate = None

def cmd_publisher_init():
    global pummbapub
    pummbapub = rospy.Publisher('Nubot_Pummba/nubotcontrol/pummbacmd', PummbaCmd, queue_size=10)
    rospy.init_node('TrackCmd_Pub_', anonymous=True)
    global rate
    rate = rospy.Rate(10)  # 10hz

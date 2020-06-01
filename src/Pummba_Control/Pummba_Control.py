#!/Home/nubot/Documents/Study/PythonEnv/ros-py3-tf1/bin/python3.6
# license removed for brevity
import rospy
import Pummba_CmdPub_Node


if __name__ == '__main__':
    try:
        Pummba_CmdPub_Node.cmd_publisher_init()
        pummba_cmd = Pummba_CmdPub_Node.PummbaCmd()
        while not rospy.is_shutdown():
            pummba_cmd.vel_linear = 0.5
            pummba_cmd.vel_angular = 0.5
            pummba_cmd.front_left = 1.5708
            pummba_cmd.front_right = 1.5708
            pummba_cmd.rear_left = 1.5708
            pummba_cmd.rear_right = 1.5708
            rospy.loginfo("pummba_cmd_track %f %f", pummba_cmd.vel_linear, pummba_cmd.vel_angular)
            rospy.loginfo("pummba_cmd_flip %f %f %f %f", pummba_cmd.front_left, pummba_cmd.front_right, pummba_cmd.rear_left,
                          pummba_cmd.rear_right)
            Pummba_CmdPub_Node.pummbapub.publish(pummba_cmd)
            Pummba_CmdPub_Node.rate.sleep()

    except rospy.ROSInterruptException:
        pass
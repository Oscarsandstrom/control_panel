#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy


speed_reference = 0
rospy.loginfo(' I heard %s', 'hello')


def callback(data):
    speed_reference = (data.axes[5] + data.axes[2])
    rospy.loginfo(' I heard %s', speed_reference)
def joyInputs():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joyInputs', anonymous=True)

    rospy.Subscriber('joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo(' I heard %s', 'hello')
    joyInputs()

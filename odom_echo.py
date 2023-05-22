
import rospy
from nav_msgs.msg import Odometry
import time

def echo(msg):
    print(f'x: {msg.pose.pose.position.x} y: {msg.pose.pose.position.y} timestamp: {time.time()}')
    


rospy.init_node('odom_echo')
sub = rospy.Subscriber('/odom',Odometry , echo)
rospy.spin()
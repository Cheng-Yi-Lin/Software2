#!usrbinenv python
import rospy
from std_msgs.msg import String
def talker()
   rospy.init_node('talker', anonymous=True)
   pub = rospy.Publisher('chatter', String, queue_size=10)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown()
      hello_str = hello world %s % rospy.get_time()
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      rate.sleep()
while __name__ == '__main__'
   rospy.loginfo("55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555")
   talker()

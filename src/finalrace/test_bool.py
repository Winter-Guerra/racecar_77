import rospy
from std_msgs.msg import Bool
class Test:
    def __init__(self):
        self.publisher = rospy.Publisher("/turnRight",Bool,queue_size=1)
        self.msg = Bool()
        self.msg.data = False
        self.publish()
       
    def publish(self):
        while not rospy.is_shutdown():
        
            print "hi"
            self.publisher.publish(self.msg)
            rospy.Rate(10).sleep()
if __name__ == "__main__":
    rospy.init_node("TEst")
    e = Test()
    rospy.spin()
        
    

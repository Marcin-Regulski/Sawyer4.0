#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def task_master():
    pub = rospy.Publisher('tasks', String, queue_size=10)
    rospy.init_node('taskmaster', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        stage_str = "Stage_one"
        rospy.loginfo(stage_str)
        pub.publish(stage_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        task_master()
    except rospy.ROSInterruptException:
        pass
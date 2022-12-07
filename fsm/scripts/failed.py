#!/usr/bin/env python3

import smach
import rospy
from server_msgs.msg import RobotFault


class Failed(smach.State):
    """
    @brief Failed State class configuration.
    @param smach.State: Class Inheritance from smach.State class. 
    @return None
    """
    def __init__(self):
        rospy.loginfo('Failed is initialized')
        
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['error_message_in'])
        
        # Define ROS Publisher, Subscriber, Service or Action
        self.error_message_publisher = rospy.Publisher('/robot_fault', RobotFault, queue_size=10)
        
        # Define ROS variables        
        self.duration = rospy.Duration(1)
        self.error_message = RobotFault()
  
        # Define class variables
        self.package_name = " [ Medeniyet.FailedState ] "


    def execute(self, ud):
        rospy.sleep(self.duration)
        
        self.error = ud.error_message_in
        self.error_message.error_message = str(self.error)
        self.error_message_publisher.publish(self.error_message)
        
        return 'succeeded'
        
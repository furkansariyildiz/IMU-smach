#!/usr/bin/env python3

import smach
import rospy



class Start(smach.State):
    """
    @brief Start State class configuration.
    @param smach.State: Class Inheritance from smach.State class. 
    @return None
    """
    def __init__(self):
        rospy.loginfo('Start is initialized')
        
        smach.State.__init__(self, outcomes=['succeeded'])  

        # Define ROS variables
        self.duration = rospy.Duration(1)
        
        # Define class variables
        self.package_name = " [ PatikaRobotics.FailedState ] "
        
        

    def execute(self, ud):
        rospy.sleep(self.duration)
        return 'succeeded'
        
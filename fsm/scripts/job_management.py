#!/usr/bin/env python3

import smach
import rospy



class JobManagement(smach.State):
    """
    @brief JobManagement State class configuration.
    @param smach.State: Class Inheritance from smach.State class. 
    @return None
    """
    def __init__(self):
        rospy.loginfo('JobManagement is initialized')
        
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['error_message_out'])  
        

        # Define ROS variables
        self.duration = rospy.Duration(1)


        # Define class variables
        self.package_name = " [ Medeniyet.JobManagement ] "
        

    def execute(self, ud):
        rospy.sleep(self.duration)

        try:
            return 'succeeded'
        except Exception as e:
            ud.error_message_out = self.package_name + str(e)
            return 'failed'
#!/usr/bin/env python3

import smach
import rospy

# Messages
from server_msgs.msg import Job

class JobManagement(smach.State):
    """
    @brief JobManagement State class configuration.
    @param smach.State: Class Inheritance from smach.State class. 
    @return None
    """
    def __init__(self):
        rospy.loginfo('JobManagement is initialized')
        
        smach.State.__init__(self, outcomes=['succeeded', 'job_management', 'failed'], input_keys=['job_in'], output_keys=['error_message_out'])  
        

        # Define ROS variables
        self.duration = rospy.Duration(1)


        # Define class variables
        self.package_name = " [ Medeniyet.JobManagement ] "
        

    def execute(self, ud):
        rospy.sleep(self.duration)
        current_job = ud.job_in
        try:
            if current_job == Job():
                return 'job_management'

            return 'succeeded'
        except Exception as e:
            ud.error_message_out = self.package_name + str(e)
            return 'failed'
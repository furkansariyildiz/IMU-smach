#!/usr/bin/env python3

import smach
import rospy

# Messages

# Services
from server_msgs.srv import RobotToBackend, RobotToBackendRequest, RobotToBackendResponse



class UpdateRobotStatus(smach.State):
    """
    @brief UpdateRobotStatus State class configuration.
    @param smach.State: Class Inheritance from smach.State class. 
    @return None
    """
    def __init__(self):
        rospy.loginfo('UpdateRobotStatus is initialized')
        
        smach.State.__init__(self, outcomes=['succeeded', 'response_failed', 'failed'], input_keys=['robot_in', 'activity_in'], output_keys=['robot_out', 'error_message_out'])  


        # Define input keys variables

        # Define ROS variables
        self.duration = rospy.Duration(1)



        # Define class variables
        self.package_name = " [ Medeniyet.UpdateRobotStatus ] "
        



    def execute(self, ud):
        # Sleeping for a duration.
        rospy.sleep(self.duration)

        robot = ud.robot_in 
        robot.current_activity = ud.activity_in
        rospy.wait_for_service('robot_to_backend')

        try:
            robot_to_backend_proxy = rospy.ServiceProxy('robot_to_backend', RobotToBackend)
            
            request = RobotToBackendRequest()
            request.robot_submit = robot
            response = robot_to_backend_proxy(request)

            if response.response == 'OK':
                ud.robot_out = robot
                return 'succeeded'

            return 'response_failed'

        except Exception as e:
            ud.error_message_out = self.package_name + str(e)
            return 'failed'

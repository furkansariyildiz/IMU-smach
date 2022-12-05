#!/usr/bin/env python3

import rospy
import smach
import smach_ros

# Messages
from server_msgs.msg import RobotSubmit
from server_msgs.msg import Job

# Services
from server_msgs.srv import RobotCurrentActivityToBackend, RobotCurrentActivityToBackendRequest, RobotCurrentActivityToBackendResponse
from server_msgs.srv import JobToRobot, JobToRobotRequest, JobToRobotResponse

# States
from start import Start
from update_robot_status import UpdateRobotStatus
from job_management import JobManagement
from failed import Failed


class FSM:
    def __init__(self):
        """
        @brief Smach-State Machine class configuration
        @param None
        @source http://wiki.ros.org/smach/Tutorials/Getting%20Started - PatikaRobotics
        @return None
        """
        self._sm = smach.StateMachine(outcomes=['Finished']) # Final State
        self._sm.set_initial_state(['Start']) # Initial State

        # Create and start the introspection server
        self._sis = smach_ros.IntrospectionServer('fsm', self._sm, '/SM_ROOT') 

        
        # Defining userdata variables
        self._sm.userdata.robot = RobotSubmit()
        self._sm.userdata.current_job = Job()

        # Defining initial robot's activity
        self._sm.userdata.robot.current_activity = 'UNSUPPORTED'

        # Defining error message
        self._sm.userdata.error_message = ''

        # Defining Robot Activities
        self._sm.userdata.UNSUPPORTED = 'UNSUPPORTED'
        self._sm.userdata.IDLE = 'IDLE'
        self._sm.userdata.IN_PROGRESS = 'IN_PROGRESS'

        # Creating-Advertising Services
        self._robot_current_activity_to_backend = rospy.Service('robot_current_activity_to_backend', RobotCurrentActivityToBackend, self.advertiseRobotCurrentActivityToBackend)
        self._job_to_robot = rospy.Service('job_to_robot', JobToRobot, self.advertiseJobToRobotService)

        self.createStates()
        self._sis.start()

    def advertiseRobotCurrentActivityToBackend(self, request:RobotCurrentActivityToBackendRequest):
        """
        @brief Returns robot's current activity within this service. 
        @param request: Request of ROS Service | Type: RobotCurrentActivityToBackendRequest()
        @return response: Response of ROS Service | Type: RobotCurrentActivityToBackendResponse()
        """
        robot = self._sm.userdata.robot
        response = RobotCurrentActivityToBackendResponse()
        
        response.current_activity = robot.current_activity
        
        return response

    
    def advertiseJobToRobotService(self, request:JobToRobotRequest):
        """
        @brief Sending a job from server to robot.
        @param request: Request of ROS Service | Type: JobToRobotRequest()
        @return response: Response of ROS Service | Type: JobToRobotResponse()
        """
        robot = self._sm.userdata.robot
        response = JobToRobotResponse()

        if not robot.current_activity == 'IDLE':
            response.response = 'NOT_IDLE'
        
        else:
            self._sm.userdata.current_job.external_reference_id = request.external_reference_id
            self._sm.userdata.current_job.tasks = request.tasks
            response.response = 'OK'

        return response
        

    def execute(self):
        self._sm.execute()


    def createStates(self):
        # Open the container.
        with self._sm:
            # Add states to the container.
            smach.StateMachine.add('Start', Start(),
                                   transitions={'succeeded': 'UpdateRobotStatusIdle'})

            smach.StateMachine.add('UpdateRobotStatusIdle', UpdateRobotStatus(),
                                    transitions={'succeeded': 'JobManagement',
                                                 'response_failed': 'UpdateRobotStatusIdle',
                                                'failed': 'Failed'},
                                    remapping={'robot_in': 'robot',
                                               'activity_in': 'IDLE',
                                               'robot_ud': 'robot',
                                               'error_message_out': 'error_message'})

            smach.StateMachine.add('JobManagement', JobManagement(),
                                    transitions={'succeeded': 'JobManagement',
                                                'failed': 'Failed'},
                                    remapping={'error_message_out': 'error_message'})

            smach.StateMachine.add('Failed', Failed(),
                                   transitions={'succeeded': 'Failed'},
                                   remapping={'error_message_in': 'error_message'})



if __name__ == '__main__':
    rospy.init_node("fsm_node", anonymous=False, disable_signals=True)
    fsm = FSM()
    fsm.execute()
    fsm._sis.stop()
    
    
    
        
            

        
            
        

    
        




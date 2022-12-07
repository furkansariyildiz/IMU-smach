#!/usr/bin/env python3

import smach
import rospy

# Messages
from server_msgs.msg import JobSubmit
from server_msgs.msg import Job

# Services
from server_msgs.srv import JobToBackend, JobToBackendRequest, JobToBackendResponse


class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Navigation is initialized')

        # Initialize State 
        smach.State.__init__(self, outcomes=['response_failed', 'in_progress', 'completed', 'failed'], input_keys=['job_in'], output_keys=['job_out', 'error_message_out'])

        # Define ROS Variables
        self.duration = rospy.Duration(1)

        # Define class variables
        self.package_name = " [ Medeniyet.Navigation ] "


    def execute(self, ud):
        current_job = ud.job_in
        rospy.loginfo("Current Job: " + str(current_job))
        rospy.wait_for_service('job_to_backend')
        rospy.sleep(10)
        try:
            job_to_backend = rospy.ServiceProxy('job_to_backend', JobToBackend)
            request = JobSubmit()

            if current_job.last_completed_task.action_name == "":
                request.job_status = "IN_PROGRESS"
                request.external_reference_id = current_job.external_reference_id
                request.error_code = ''
                request.message = ''
                request.last_completed_task = current_job.last_completed_task
                response = job_to_backend(request)

                if not response.mission_status == 'OK':
                    return 'response_failed'

                else:
                    # ud.job_out.last_completed_task = current_job.tasks[0]
                    current_job.last_completed_task = current_job.tasks[0]
                    ud.job_out = current_job
                    return 'in_progress'

            else:
                request.job_status = "COMPLETED"
                request.external_reference_id = current_job.external_reference_id
                request.error_code = ''
                request.message = ''
                request.last_completed_task = current_job.last_completed_task
                response = job_to_backend(request)
                
                if not response.mission_status == 'OK':
                    return 'response_failed'

                else:
                    # ud.job_out.last_completed_task = current_job.tasks[1]
                    current_job = Job()
                    ud.job_out = current_job
                    return 'completed'


        except Exception as e:
            rospy.loginfo(self.package_name + str(e))
            ud.error_message_out = self.package_name + str(e)
            return 'failed'      
                
            
            
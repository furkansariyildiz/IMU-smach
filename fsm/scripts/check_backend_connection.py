#!/usr/bin/env python3

import smach
import rospy
import socketio
import subprocess

# 
from rosbridge_msgs.msg import ConnectedClients

sio = socketio.Client()

class BackendConnection(smach.State):
    def __init__(self):
        rospy.loginfo("CheckBackendConnection is initialized.")

        smach.State.__init__(self, outcomes=['succeeded', 'check_backend_connection', 'failed'], output_keys=['error_message_out'])

        rospy.Subscriber("/connected_clients", ConnectedClients, self.connectedClientsCallback)

        # Server IP Adress
        self.server_ip_adress = "192.168.1.32"
        
        # Rosbridge IP Adress with Rosbridge Port
        self.robot_url = "ws://192.168.1.32:9090"

        # Rospy duration for a sec.
        self.duration = rospy.Duration(1)

        # Ros Message Variables
        self.connected_clients = ConnectedClients()

        self.package_name = " [ Medeniyet.BackendConnection ] "


    def connectedClientsCallback(self, message):
        self.connected_clients = message



    def createIOConnection(self):
        try:
            sio.connect(self.server_ip_adress)
        except Exception as e:
            rospy.sleep(self.duration)
            return self.createIOConnection()


    @sio.on('connect')
    def resetRobot(self):
        sio.emit('RobotReset', {'robot_url': self.robot_url})


    def execute(self, ud):
        rospy.sleep(self.duration)
        try:
            if len(self.connected_clients.clients) == 0:
                ping_response = subprocess.call(['ping', '-c', '1', self.server_ip_adress])

                return 'check_backend_connection'

            return 'succeeded'

        except Exception as e:
            ud.error_message_out = self.package_name + str(e)
            return 'failed'

            
            


#!/usr/bin/env python

import rospy
import actionlib
import xmlrpc.client
from control_msgs.msg import GripperCommandAction, GripperCommandActionFeedback, GripperCommandActionResult
from sensor_msgs.msg import JointState
import json


class ur_onrobot(object):
    _rpc_server = None
    _result = GripperCommandActionResult()
    _feedback = GripperCommandActionFeedback()

    def __init__(self, name):
        
        self._action_name = name
        try:
            self._robot_ip = rospy.get_param('/robot_ip')

            self._rpc_server = xmlrpc.client.ServerProxy("http://" + str(self._robot_ip) + ":41414/")
            if(self._rpc_server.system.listMethods()):
                rospy.logdebug(self._rpc_server.system.listMethods())
                

            self._server = actionlib.SimpleActionServer(
                'onrobot_gripper_action', GripperCommandAction, execute_cb=self.execute_cb, auto_start=False)
            self._server.start()
            
            gripper = json.loads(self._rpc_server.get_discovery())["devices"][0]["deviceName"]
            if gripper != '2FG7':
                rospy.logerr('Gripper not yet supported')
                rospy.shutdown() 
            rospy.loginfo("Connected to Gripper "+ gripper + ' - READY')

            rospy.spin()

        except KeyError as e:
            rospy.logerr(str(e) + " not set")
            rospy.shutdown()
            return

        except Exception as ex:
            rospy.logdebug("Error is: " + str(ex))
            rospy.logerr("Connection to Gripper Failed")
            rospy.shutdown()

    def execute_cb(self, goal):
        # r = rospy.Rate(1)
        rospy.loginfo('Gripper controller action goal recieved:')
        rospy.loginfo('    Position Goal: '+ str(goal.command.position))
        rospy.loginfo('    Max Effort Goal: '+ str(goal.command.max_effort))
        if not (20.0 <= goal.command.max_effort <= 140.0):
            self._server.set_aborted()
            rospy.logerr("Max Effort outside limits (20.0-140.0)")
            return
        if not (34.0 <= goal.command.position <= 70.0):
            self._server.set_aborted()
            rospy.logerr("Position outside limits (34.0-70.0)")
            return

        rospy.loginfo("Moving gripper")
        
        self._server.set_succeeded(self._result)
        success = True
        

if __name__ == '__main__':
    try:
        rospy.init_node('onrobot_ur', log_level=rospy.INFO)
        gripper = ur_onrobot(rospy.get_name())
        rospy.spin()
        # while not rospy.is_shutdown():
            
    except rospy.ROSInterruptException:
        pass

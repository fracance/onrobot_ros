#!/usr/bin/env python

import rospy
import actionlib
import xmlrpc.client
from control_msgs.msg import GripperCommandAction, GripperCommandActionFeedback, GripperCommandActionResult
from sensor_msgs.msg import JointState
import json
from std_msgs.msg import Bool

import traceback

class ur_onrobot(object):
    _rpc_server = None
    _result = GripperCommandActionResult()
    _feedback = GripperCommandActionFeedback()
    _variables = None

    def __init__(self, name):

        self._action_name = name
        try:
            self._robot_ip = rospy.get_param('/robot_ip')
            
            self._rpc_server = xmlrpc.client.ServerProxy(
                "http://" + str(self._robot_ip) + ":41414/")
            if(self._rpc_server.system.listMethods()):
                rospy.logdebug(self._rpc_server.system.listMethods())

            self._server = actionlib.SimpleActionServer(
                'onrobot_gripper_action', GripperCommandAction, execute_cb=self.execute_cb, auto_start=False)
            self._server.start()

            gripper = json.loads(self._rpc_server.get_discovery())[
                "devices"][0]["deviceName"]
            if gripper != '2FG7':
                rospy.logerr('Gripper not yet supported')
                rospy.signal_shutdown()
            rospy.loginfo("Connected to Gripper " + gripper + ' - READY')

            self._rpc_server.twofg_set_fingertip_offset(0, 0.0)
            self._variables = self._rpc_server.twofg_get_all_variables(0)

            self.update_variables()
            # rospy.spin()

        except KeyError as e:
            rospy.logerr(str(e) + " not set")
            rospy.signal_shutdown(str(e) + " not set")
            return

        except Exception as ex:
            print(traceback.format_exc(ex))
            rospy.logdebug("Error is: " + str(ex))
            rospy.logerr("Connection to Gripper Failed")
            rospy.signal_shutdown("Connection to Gripper Failed")

    def execute_cb(self, goal):
        # r = rospy.Rate(1)
        rospy.loginfo('Gripper controller action goal recieved:')
        rospy.loginfo('    Position Goal: ' + str(goal.command.position))
        rospy.loginfo('    Max Effort Goal: ' + str(goal.command.max_effort))
        if not (20.0 <= goal.command.max_effort <= 140.0):
            self._server.set_aborted()
            rospy.logerr("Max Effort outside limits (20.0-140.0)")
            return
        if not (self._variables['min_internal_width'] <= goal.command.position <= self._variables['max_internal_width']):
            self._server.set_aborted()
            rospy.logerr("Position outside limits (%f-%f)",
                         self._variables['min_internal_width'], self._variables['max_internal_width'])
            return

        rospy.loginfo("Moving gripper")

        self._server.set_succeeded(self._result)
        success = True

    def update_variables(self):
        
        self._pubState=rospy.Publisher('gripper_status', JointState, queue_size=10)
        self._pubGrip=rospy.Publisher('gripper_gripped', Bool, queue_size=10)
        rate = rospy.Rate(2)
        jointMsg = JointState()
        jointMsg.name = [str(json.loads(self._rpc_server.get_discovery())[
                "devices"][0]["deviceName"])]
        rospy.loginfo("Publishing Gripper Status")
        jointMsg.header.frame_id = jointMsg.name[0]
        while not rospy.is_shutdown():
            variables = self._rpc_server.twofg_get_all_variables(0)
            jointMsg.position = [variables['internal_width']]
            jointMsg.effort = [variables['force']]
            self._pubState.publish(jointMsg)
            self._pubGrip.publish(variables['grip_detected'])
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('onrobot_ur', log_level=rospy.DEBUG)
        gripper = ur_onrobot(rospy.get_name())
        rospy.spin()
        # while not rospy.is_shutdown():
        
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import actionlib
import xmlrpc.client


class UR_Onrobot(object):

    _rpc_server = None

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)
        try:
            self._robot_ip = rospy.get_param('/robot_ip')
            self._gripper = rospy.get_param('/onrobot_gripper')

            self._rpc_server = xmlrpc.client.ServerProxy(
                "http://" + str(self._robot_ip) + ":41414/")
            if(self._rpc_server.system.listMethods()):
                rospy.logerr("Connected to Gripper")

        except KeyError as e:
            rospy.logerr(str(e) + " not set")
            return

        except Exception as ex:
            rospy.logerr("Connection to Gripper Failed")

    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True


if __name__ == '__main__':
    try:
        rospy.init_node('onrobot_ur', log_level=rospy.DEBUG)
        gripper = UR_Onrobot(rospy.get_name())
    except rospy.ROSInterruptException:
        pass

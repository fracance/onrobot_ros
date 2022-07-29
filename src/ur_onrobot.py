#!/usr/bin/env python

import rospy
import actionlib
import xmlrpc.client





class UR_Onrobot():

    _rpc_server = None

    def __init__(self):
        try:
            self._robot_ip = rospy.get_param('/robot_ip')
            self._gripper = rospy.get_param('/onrobot_gripper')
            
            self._rpc_server = xmlrpc.client.ServerProxy("http://" + str(self._robot_ip) + ":41414/")
            if(self._rpc_server.system.listMethods()):
                rospy.logerr("Connected to Gripper")

        except KeyError as e:
            rospy.logerr(str(e) + " not set")
            return
        
        except Exception as ex:
            rospy.logerr("Connection to Gripper Failed")




if __name__ == '__main__':
    try:
        rospy.init_node('onrobot_ur', log_level=rospy.DEBUG)
        gripper = UR_Onrobot()
    except rospy.ROSInterruptException:
        pass

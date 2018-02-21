#!/usr/bin/env python

import roslib
roslib.load_manifest('simulation_control')
import rospy
import actionlib
import mavros_state
import time

from simulation_control.msg import center_on_objectAction, center_on_objectGoal, descend_on_objectAction, \
    descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal

if __name__ == '__main__':
    rospy.init_node('action_controller')
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()
    goto_position_goal.destination.pose.position.z = 3
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Takeoff succeded")


    goto_position_goal.destination.pose.position.x = 34
    goto_position_goal.destination.pose.position.y = -25
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Is at position (x,y,z)=(34.4, -25.6, 10)")


    rospy.loginfo("Detecting apriltag...")
    detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
    detect_object_client.wait_for_server()
    detect_object_goal = detect_objectGoal()
    detect_object_client.send_goal(detect_object_goal)
    detect_object_client.wait_for_result()
    print(detect_object_client.get_result())

    rospy.loginfo("Going to detected position")
    goto_position_goal.destination.pose.position.x = detect_object_client.get_result().detected_position.pose.position.x
    goto_position_goal.destination.pose.position.y = detect_object_client.get_result().detected_position.pose.position.y
    goto_position_goal.destination.pose.position.z = detect_object_client.get_result().detected_position.pose.position.z
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Is at position (x,y,z)=({}, {}, {})".format(detect_object_client.get_result().detected_position.pose.position.x,
                                                               detect_object_client.get_result().detected_position.pose.position.y,
                                                               detect_object_client.get_result().detected_position.pose.position.z))
    #time.sleep(3)
    rospy.loginfo("Descending on object")
    descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
    descend_on_object_client.wait_for_server()
    rospy.loginfo("Descending server started")
    descend_on_object_goal = descend_on_objectGoal()
    descend_on_objectGoal = 2.0
    descend_on_object_client.send_goal(descend_on_object_goal)
    descend_on_object_client.wait_for_result()
    if descend_on_object_client.get_result().position_reached.data:
        print("landing")
        mv_state.arm(False)
        #mv_state.land(0.9)
    else:
        rospy.loginfo("Couldnt land exiting")
    time.sleep(4)
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Going Home")

    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    mv_state.land(0.0)



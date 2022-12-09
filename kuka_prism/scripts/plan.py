#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
 
group_arm = moveit_commander.MoveGroupCommander("arm_group")
end_effector = moveit_commander.MoveGroupCommander("end_effector")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

end_effector.set_named_target("open")
plan1 = end_effector.plan()
rospy.sleep(1)
end_effector.go(wait=True)

group_arm.set_named_target("pick_ready")
plan2 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

group_arm.set_named_target("pick")
plan3 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

end_effector.set_named_target("close")
plan4 = end_effector.plan()
rospy.sleep(1)
end_effector.go(wait=True)

group_arm.set_named_target("pick_ready")
plan5 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

group_arm.set_named_target("ready_drop")
plan6 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

group_arm.set_named_target("drop")
plan7 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

end_effector.set_named_target("open")
plan8 = end_effector.plan()
rospy.sleep(1)
end_effector.go(wait=True)

group_arm.set_named_target("ready_drop")
plan1 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

group_arm.set_named_target("rest")
plan1 = group_arm.plan()
rospy.sleep(5)
group_arm.go(wait=True)

moveit_commander.roscpp_shutdown()

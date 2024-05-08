#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e=e):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if (
            notification.action_event == Base_pb2.ACTION_END
            or notification.action_event == Base_pb2.ACTION_ABORT
        ):
            e.set()

    return check


def joint_movement(base, joints):

    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = joints[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e), Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def joint_movement_wrist(base, base_cyclic, wrist):

    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    feedback = base_cyclic.RefreshFeedback()
    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = feedback.actuators[joint_id].position
    
    joint_angle.value = wrist

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e), Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def cartesian_action_movement(base, base_cyclic, pose):

    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = pose[0]
    cartesian_pose.y = pose[1]
    cartesian_pose.z = pose[2]
    cartesian_pose.theta_x = pose[3]
    cartesian_pose.theta_y = pose[4]
    cartesian_pose.theta_z = pose[5]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e), Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def gripper_command(base, position):
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.value = position
    finger.finger_identifier = 1
    base.SendGripperCommand(gripper_command)
    gripper_request = Base_pb2.GripperRequest()

    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len(gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            if abs(gripper_measure.finger[0].value - position) < 0.01:
                break
        else:  # Else, no finger present in answer, end loop
            break

    return True
def populateAngularPose(jointPose,durationFactor):
    waypoint = Base_pb2.AngularWaypoint()
    waypoint.angles.extend(jointPose)
    waypoint.duration = durationFactor*0.8
    
    return waypoint
 

def example_trajectory(base, base_cyclic, jointPoses):

    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # jointPoses = tuple(tuple())
    product = base.GetProductConfiguration()

    # if(   product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L53 
    # or product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
    #     if(product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
    #         jointPoses = (  (0.0,  344.0, 75.0,  360.0, 300.0, 0.0),
    #                         (0.0,  21.0,  145.0, 272.0, 32.0,  273.0),
    #                         (42.0, 334.0, 79.0,  241.0, 305.0, 56.0))
    #     else:
    #         # Binded to degrees of movement and each degrees correspond to one degree of liberty
    #         degreesOfFreedom = base.GetActuatorCount();

    #         if(degreesOfFreedom.count == 6):
    #             jointPoses = (  ( 360.0, 35.6, 281.8, 0.8,  23.8, 88.9 ),
    #                             ( 359.6, 49.1, 272.1, 0.3,  47.0, 89.1 ),
    #                             ( 320.5, 76.5, 335.5, 293.4, 46.1, 165.6 ),
    #                             ( 335.6, 38.8, 266.1, 323.9, 49.7, 117.3 ),
    #                             ( 320.4, 76.5, 335.5, 293.4, 46.1, 165.6 ),
    #                             ( 28.8,  36.7, 273.2, 40.8,  39.5, 59.8 ),
    #                             ( 360.0, 45.6, 251.9, 352.2, 54.3, 101.0 ))
    #         else:
    #             jointPoses = (  ( 360.0, 35.6, 180.7, 281.8, 0.8,   23.8, 88.9  ),
    #                             ( 359.6, 49.1, 181.0, 272.1, 0.3,   47.0, 89.1  ),
    #                             ( 320.5, 76.5, 166.5, 335.5, 293.4, 46.1, 165.6 ),
    #                             ( 335.6, 38.8, 177.0, 266.1, 323.9, 49.7, 117.3 ),
    #                             ( 320.4, 76.5, 166.5, 335.5, 293.4, 46.1, 165.6 ),
    #                             ( 28.8,  36.7, 174.7, 273.2, 40.8,  39.5, 59.8  ),
    #                             ( 360.0, 45.6, 171.0, 251.9, 352.2, 54.3, 101.0 ))
            
    # else:
    #     print("Product is not compatible to run this example please contact support with KIN number bellow")
    #     print("Product KIN is : " + product.kin())


    waypoints = Base_pb2.WaypointList()    
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = True
    
    index = 0
    for jointPose in jointPoses:
        waypoint = waypoints.waypoints.add()
        waypoint.name = "waypoint_" + str(index)
        durationFactor = 1
        # Joints/motors 5 and 7 are slower and need more time
        if(index == 1):
            durationFactor = 1.1
        if(index == 4 or index == 6):
            durationFactor = 6 # Min 30 seconds
        
        waypoint.angular_waypoint.CopyFrom(populateAngularPose(jointPose, durationFactor))
        index = index + 1 
    
    
   # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints);
    if(len(result.trajectory_error_report.trajectory_error_elements) == 0):

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Reaching angular pose trajectory...")
        
        
        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished
    else:
        print("Error found in trajectory") 
        print(result.trajectory_error_report)
        finished = True
        return finished

def main():

    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        pick_pose = [0.561, -0.008, 29, 0, 180, 45]
        drop_pose = [341.7, 40.32, 9.41, 109.9, 253.0, 340.0, 87]
        twist_pose = [0.776, -0.011, 0.166, 91.9, 173.7, 62.5]
        regrip_pre_pose = [0.77, -0.057, 0.127, -169.77, 47.29, 166]
        regrip_pose = [00.8, -0.073, 0.103, -172.0, 47.9, 168.5]
        dunk_pose = [0.318, -0.233, 0.2, 180, 0, 168]
        feed_pose = [0.315, -0.44, 0.3, 90, -180, 0]
        # Example core
        success = True
        # success &= gripper_command(base, 0.35)
        # success &= joint_movement(base, [5, 35, 348, 64, 5, 80, 37])
        # success &= cartesian_action_movement(base, base_cyclic, pick_pose)
        # input("Enter to start")
        # # #Pick
        # pick_pose[2] = 0.033
        # success &= cartesian_action_movement(base, base_cyclic, pick_pose)
        # success &= gripper_command(base, 0.485)
        # #Move to twist
        # pick_pose[2] = 0.15
        # success &= cartesian_action_movement(base, base_cyclic, pick_pose)
        # success &= joint_movement(base, [332.0, 70.2, 11.1, 86.8, 299.1, 288.6, 95.2])
        # input("Enter to continue")
        # # time.sleep(1)
        # success &= joint_movement_wrist(base, base_cyclic, 60)
        # success &= cartesian_action_movement(base, base_cyclic, [0.688, 0.093, 0.148, 91.58, 125.7, 61.741])
        # # #Move to drop
        # success &= joint_movement(base, drop_pose)
        # success &= gripper_command(base, 0.3)
        # #regrip
        # success &= cartesian_action_movement(base, base_cyclic, regrip_pre_pose)
        # success &= cartesian_action_movement(base, base_cyclic, regrip_pose)
        # success &= gripper_command(base, 0.9)
        # input("Enter to continue")
        # # #Dunk
        # dunk_pose[2] = 0.25
        # success &= cartesian_action_movement(base, base_cyclic, dunk_pose)
        # for _ in range(2):
        #     dunk_pose[2] = 0.15
        #     success &= cartesian_action_movement(base, base_cyclic, dunk_pose)
        #     dunk_pose[2] = 0.18
        #     success &= cartesian_action_movement(base, base_cyclic, dunk_pose)
        # dunk_pose[2] = 0.22
        # success &= cartesian_action_movement(base, base_cyclic, dunk_pose)
        # success &= cartesian_action_movement(base, base_cyclic, feed_pose)
        # success &= gripper_command(base, 0.85)
        # input()
        # success &= gripper_command(base, 0.9)
        # feed_pose[1] = -0.7
        # success &= cartesian_action_movement(base, base_cyclic, feed_pose)
        # success &= gripper_command(base, 0.85)
        # time.sleep(1)
        # success &= gripper_command(base, 0.4)
        # success &= joint_movement(base, [])
        success &= joint_movement(base, [54, 51, 356, 91, 313, 295, 118])
        success &= gripper_command(base, 0.85)
        input()
        success &= gripper_command(base, 0.92)
        input()
        success &= example_trajectory(base, base_cyclic, [ [54, 58, 359, 70, 338, 313, 109], [58, 52, 2.6, 77, 305, 309, 130]])
        # success &= joint_movement(base, [54, 58, 359, 70, 338, 313, 109])
        # success &= joint_movement(base, [58, 52, 2.6, 77, 305, 309, 130])
        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1


if __name__ == "__main__":
    exit(main())

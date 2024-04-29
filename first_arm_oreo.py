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
        pick_pose = [0.55, 0.0, 0.23, 0, 180, 45]
        drop_pose = [341.7, 40.32, 9.41, 109.9, 253.0, 340.0, 87]
        twist_pose = [0.776, -0.011, 0.166, 91.9, 173.7, 62.5]
        regrip_pose = []
        dunk_pose = []
        # Example core
        success = True
        success &= gripper_command(base, 0.35)
        success &= joint_movement(base, [7.0, 37.3, 348.7, 71.8, 5.0, 70.33, 37.0])
        success &= cartesian_action_movement(base, base_cyclic, pick_pose)
        input("Enter to start")
        #Pick
        pick_pose[2] = 0.078
        success &= cartesian_action_movement(base, base_cyclic, pick_pose)
        success &= gripper_command(base, 0.485)
        #Move to twist
        pick_pose[2] = 0.15
        success &= cartesian_action_movement(base, base_cyclic, pick_pose)
        success &= joint_movement(base, [332.0, 70.2, 11.1, 86.8, 299.1, 288.6, 95.2])
        input("Enter to continue")
        # time.sleep(1)
        success &= joint_movement_wrist(base, base_cyclic, 50)
        success &= cartesian_action_movement(base, base_cyclic, [0.688, 0.093, 0.148, 91.58, 125.7, 61.741])
        input("Enter to continue")
        #Move to drop
        success &= joint_movement(base, drop_pose)
        success &= gripper_command(base, 0.3)
        return 0
        #regrip
        success &= cartesian_action_movement(base, base_cyclic, regrip_pose)
        regrip_pose[2] = 0.3
        success &= cartesian_action_movement(base, base_cyclic, regrip_pose)
        success &= gripper_command(base, 0.8)
        #Dunk
        success &= cartesian_action_movement(base, base_cyclic, dunk_pose)
        success &= cartesian_action_movement(base, base_cyclic, dunk_pose)
        # time.sleep(1)
        # success &= gripper_command(base, 0.4)
        # success &= joint_movement(base, [])

        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1


if __name__ == "__main__":
    exit(main())

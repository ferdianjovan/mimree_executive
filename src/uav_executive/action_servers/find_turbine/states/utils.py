import time

import numpy as np


def wait_for_action_completion(timeout, target_pose, pos_err, or_err,
                               pose_stream):
    """
    :param timeout: allowed time to reach target pose
    :param target_pose: the pose to reach GeoPoseStamped
    :param pos_err: acceptable tolerance to assume pose has been reached
    :param or_err: acceptable tolerance to assume pose has been reached
    :param pose_stream: the constantly updated pose value: GeoPoseStamped
    :return:
    """
    attributes = ['altitude', 'longitude', 'latitude', 'x', 'y', 'z', 'w']
    time_elapsed = time.time()
    t_pos, t_or = target_pose.pose.position, target_pose.pose.orientation
    t_pos_ranges = map(lambda a: [t_pos.__getattribute__(a) - pos_err, t_pos.__getattribute__(a) + pos_err],
                       attributes[:3])
    t_or_ranges = map(lambda a: [t_or.__getattribute__(a) - or_err, t_or.__getattribute__(a) + or_err],
                      attributes[3:])
    t_ranges = t_pos_ranges + t_or_ranges
    prev_c_ranges = None
    while timeout > 0:
        timeout -= time.time() - time_elapsed
        time_elapsed = time.time()
        # both positional and orientational values must be within target+-error to be accepted
        c_pos = pose_stream.pose.position
        c_or = pose_stream.pose.orientation
        c_pos_ranges = map(lambda a: c_pos.__getattribute__(a), attributes[:3])
        c_or_ranges = map(lambda a: c_or.__getattribute__(a), attributes[3:])
        c_ranges = c_pos_ranges + c_or_ranges
        if prev_c_ranges == c_ranges:
            print("awaiting update")
            time.sleep(0.2)
            continue
        prev_c_ranges = c_ranges
        if np.all(map((lambda a, b: b[0] < a < b[1]), c_ranges, t_ranges)):
            return (True, "POSE_REACHED")
    return (False, "OUT_OF_TIME")


def wait_for_stop(timeout, vel_stream, ang_thresh, lin_thresh):
    """

    :param timeout:
    :param vel_stream: Twist message
    :return:
    """
    print("WAITING FOR STOP: ")
    time_elapsed = time.time()
    attributes = ['x', 'y', 'z']
    while timeout > 0:
        timeout -= time.time() - time_elapsed
        time_elapsed = time.time()
        l_xyz = map(lambda a: abs(vel_stream.twist.linear.__getattribute__(a)), attributes)
        a_xyz = map(lambda a: abs(vel_stream.twist.angular.__getattribute__(a)), attributes)
        print(l_xyz, a_xyz)
        if np.all(map(lambda a, l: a < ang_thresh and l < lin_thresh, a_xyz, l_xyz)):
            print('ARRIVED')
            break
        else:
            time.sleep(0.1)
    print("TIMEOUT REACHED")

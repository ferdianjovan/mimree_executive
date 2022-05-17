import time

import matplotlib.pyplot as plt
import numpy as np
# Though the following import is not directly being used, it is required
# for 3D projection to work
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import MeanShift


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


from scipy.stats import gaussian_kde


def find_wt_pole(np_array):
    """
    Finds all points that associate with pole
    :param np_array:
    :return:
    """
    # if points are collapsed on a 2d plane (i.e. ignore z)
    # then the area where there are the most points will be the pole as it is a vertical wide structure.
    # collapse the points onto 2d and plot them
    arr_2d = np_array[:, :2]
    x, y = arr_2d[:, 0], np_array[:, 1]
    xy = np.vstack([x, y])
    z = gaussian_kde(xy)(xy)
    pole_coordinates = np.where(z == np.amax(z))
    pole = arr_2d[[pole_coordinates]][0]
    pole_mean = np.mean(pole, axis=0)
    print(pole_mean)
    print(z)
    print("POLE (MAX GAUSS)", pole)

    fig, ax = plt.subplots()

    ax.scatter(x, y, c=z, s=100, edgecolor='')

    x_vals = np_array[:, 0]
    y_vals = np_array[:, 1]
    max_x, min_x = np.max(x_vals), np.min(x_vals)
    max_y, min_y = np.max(y_vals), np.min(y_vals)
    range_x = max_x - min_x
    range_y = max_y - min_y
    chosen = y_vals
    if range_x > range_y:
        chosen = x_vals
    # find index of both extremes
    result_max = np.where(chosen == np.amax(chosen))
    result_min = np.where(chosen == np.amin(chosen))
    min_vals = arr_2d[[result_min]][0]
    max_vals = arr_2d[[result_max]][0]
    print(min_vals, max_vals)
    print("concatenating: ", [np.mean(min_vals, axis=0)])
    to_plot = np.concatenate(([np.mean(min_vals, axis=0)], [np.mean(max_vals, axis=0)]), axis=0)
    print(to_plot)
    plt.plot(to_plot[:, 0], to_plot[:, 1], 'r')
    normal = (to_plot[1] - to_plot[0]) / (np.linalg.norm(to_plot[1] - to_plot[0]))
    print("normal: ", normal)
    p_v = pole_mean - to_plot[0]
    print("from left to pole", p_v)
    t = np.dot(p_v, normal)
    print("dot product with normal: ", t)
    p_online = to_plot[0] + t * normal
    x = [pole_mean[0], p_online[0]]
    y = [pole_mean[1], p_online[1]]

    plt.plot(x, y, 'r')
    plt.show()


def get_k_means_clusters(pc, k):
    """
    :param k: number of clusters
    :param pc: pointcloud
    :return: each cluster.
    """
    find_wt_pole(pc)
    print(pc[0])
    print(pc.shape)
    fig = plt.figure(figsize=(20, 20))

    ax = Axes3D(fig)

    ax.set_xlim3d(-100, 100)
    ax.set_ylim3d(-50, 150)
    ax.set_zlim3d(-50, 150)
    # est2 = SpectralClustering(n_clusters=2)
    # est3=DBSCAN()
    est1 = MeanShift()
    est = est1
    X = pc
    est.fit(X[:, :2])
    labels = est.labels_
    # 0,1,2
    # 0,2,1
    # 1,0,2
    x = 0
    y = 1
    z = 2
    print(X)
    for p in X:
        print(p)
    ax.scatter(X[:, x], X[:, y], X[:, z],
               c=labels.astype(np.float), edgecolor='k')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title('k_means where k = ' + str(k))
    fig.show()
    time.sleep(500)

    pass

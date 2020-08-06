import numpy as np
import quaternion
import rospy
import sensor_msgs
import std_msgs
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

bridge = CvBridge()


def is_moving_towards(vector_to_point, velocity):
    return np.dot(vector_to_point, velocity) > 0


def unit_vector(vector):
    """
    Returns the unit vector of the vector.
    """
    # print(vector)
    divisor = np.linalg.norm(vector)
    if divisor == 0:
        res = np.zeros(vector.shape)
    else:
        res = vector / divisor
    return res


def angle_deg(v1, v2):
    """
    Returns the angle in degrees between vectors 'v1' and 'v2'::
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.rad2deg(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))


def angle_rad(v1, v2):
    """
    Returns the angle in radians between vectors 'v1' and 'v2'::
    """
    # print(unit_vector(v1))
    # print(unit_vector(v2))
    dot = np.dot(v1, v2)
    mag_a = np.linalg.norm(v1)
    mag_b = np.linalg.norm(v2)
    cos_theta = dot / (mag_a * mag_b)
    return cos_theta


def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return (ang1 - ang2) % (2 * np.pi)


def quaternion_product(q, r):
    """
    https://www.mathworks.com/help/aerotbx/ug/quatmultiply.html?w.mathworks.com
    https://www.mathworks.com/help/fusion/ref/quaternion.rotatepoint.html
    Output quaternion product quatprod has the form of
    n=qxr=n0+in1+jn2+kn3
    where
    n0=(r0q0-r1q1-r2q2-r3q3)
    n1=(r0q1+r1q0-r2q3+r3q2)
    n2=(r0q2+r1q3+r2q0-r3q1)
    n3=(r0q3-r1q2+r2q1+r3q0)

    :param q:
    :param r:
    :return:
    """
    return [r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
            r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2],
            r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1],
            r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0]]


def rotate_point_by_quaternion(point: np.array, q):
    """
    https://www.mathworks.com/help/fusion/ref/quaternion.rotatepoint.html
    Converts point [x,y,z] to a quaternion:

    uq=0+xi+yj+zk

    Normalizes the quaternion, q:

    qn=qGa2+b2+c2+d2

    Applies the rotation:re-iterate

    vq=quqq*

    Converts the quaternion output, vq, back to R3
    :param point:
    :param q:
    :return:
    """
    # add the imaginary w variable to the point to convert it to a quaternion
    # the w value is 0 when the point is real.
    r = [0] + point.tolist()
    # calculate the conjugate
    q_conj = q * [1, -1, -1, -1]
    # quaternion is normalized already so no need to reiterate. only take the last 3 variables (ignore w)
    return quaternion_product(quaternion_product(q, r), q_conj)[1:]


def get_twist(linear_xyz=(0, 0, 0), angular_xyz=(0, 0, 0)):
    twist = Twist()
    set_xyz_class(twist.linear, linear_xyz)
    set_xyz_class(twist.angular, angular_xyz)
    return twist


def set_twist(twist: Twist, linear_xyz=None, angular_xyz=None):
    if linear_xyz is not None:
        set_xyz_class(twist.linear, linear_xyz)
    if angular_xyz is not None:
        set_xyz_class(twist.angular, angular_xyz)


def get_pose(position_xyz=(0, 0, 0), orientation_wxyz=(1, 0, 0, 0)):
    pose = Pose()
    set_xyz_class(pose.position, position_xyz)
    set_wxyz_class(pose.orientation, orientation_wxyz)
    return pose


def get_odom(position_xyz=(0, 0, 0), orientation_wxyz=(1, 0, 0, 0)):
    odom = Odometry()
    set_xyz_class(odom.pose.pose.position, position_xyz)
    set_wxyz_class(odom.pose.pose.orientation, orientation_wxyz)
    return odom


def set_xyz_class(xyz_class, xyz_arr):
    xyz_class.x = xyz_arr[0]
    xyz_class.y = xyz_arr[1]
    xyz_class.z = xyz_arr[2]


def set_wxyz_class(wxyz_class, wxyz_arr):
    wxyz_class.w = wxyz_arr[0]
    wxyz_class.x = wxyz_arr[1]
    wxyz_class.y = wxyz_arr[2]
    wxyz_class.z = wxyz_arr[3]


def xyz_class_to_np_array(xyz_class):
    return np.array([xyz_class.x, xyz_class.y, xyz_class.z])


def wxyz_class_to_np_array(xyz_class):
    return np.array([xyz_class.w, xyz_class.x, xyz_class.y, xyz_class.z])


def wxyz_class_to_np_quaternion(xyz_class):
    return np.quaternion(xyz_class.w, xyz_class.x, xyz_class.y, xyz_class.z)


def get_vector_to_point_quaternion_odom(point_from: Odometry, point_to: Odometry):
    p_from = xyz_class_to_np_array(point_from.pose.pose.position)
    q_from = wxyz_class_to_np_quaternion(point_from.pose.pose.orientation)
    q_to = wxyz_class_to_np_quaternion(point_to.pose.pose.orientation)
    p_to = xyz_class_to_np_array(point_to.pose.pose.position)

    q_abs=np.abs(q_from)
    q = (q_from.conj() * q_abs) * q_to
    # q=q_to
    q.w=-q.w

    new_point=quaternion.rotate_vectors(q,np.array([p_from-p_to]))

    return new_point[0]


def get_vector_to_point_quaternion(point_from: np.array, point_to: np.array, quaternion_reference: np.array):
    """
    vector to point from the zero frame of reference (0,0,0,0) to another frame of reference (w,x,y,z)
    :param point_from:
    :param point_to:
    :param quaternion_reference:
    :return:
    """
    vector_to_point = point_from-point_to
    quaternion2 = quaternion_reference * np.array([-1, 1, 1, 1],dtype=float)
    vector_to_point = rotate_point_by_quaternion(vector_to_point, quaternion2)
    return np.array(vector_to_point)


def points_to_point_cloud(points, parent_frame):
    """
    Taken from:
    https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    for converting an array to a point cloud.
    Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """

    ros_dtype = sensor_msgs.msg.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.msg.PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]

    header = std_msgs.msg.Header(frame_id=parent_frame, stamp=rospy.Time.now())
    return sensor_msgs.msg.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )


def arr_to_img(arr):
    im = np.flip(arr, axis=0).astype(np.uint8)
    return bridge.cv2_to_compressed_imgmsg(im)


def arr_to_rgb(arr, values, values_rgb):
    """
    converts map to rgb array, for publishing
    :return:
    """
    r = np.zeros((arr.shape[0], arr.shape[1], 3))
    for i, v in enumerate(values):
        rgb_val = np.array(values_rgb[i])
        r[arr == v] = rgb_val
    return r


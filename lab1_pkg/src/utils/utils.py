#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa
"""
import numpy as np
from math import sin, cos, atan2
import itertools

try:
    import rospy
    import tf
    from geometry_msgs.msg._Point import Point
    import tf.transformations as tfs
    from geometry_msgs.msg import Pose, PoseStamped
    ros_enabled = True
except:
    ros_enabled = False



def length(vec):
    """
    Returns the length of a 1 dimensional numpy vector

    Parameters
    ----------
    vec : nx1 :obj:`numpy.ndarray`

    Returns
    -------
    float
        ||vec||_2^2
    """
    return np.linalg.norm(vec)

def normalize(vec):
    """
    Returns a normalized version of a numpy vector

    Parameters
    ----------
    vec : nx' :obj:`numpy.ndarray

    Returns
    -------
    nx' :obj:`numpy.ndarray`
    """
    return vec / length(vec)

def joint_array_to_dict(vel_torque_array, limb):
    """
    the baxter interface requires you to send the joint velocities / torques
    as a dictionary, this turns and array of velocities and torques into a 
    dictionary with joint names.

    Parameters
    ----------
    vel_torque_array : 7x' :obj:`numpy.ndarray`
        numpy array of velocities or torques to be sent to the baxter
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    :obj:`dict` of string->float
        mapping of joint names to joint velocities / torques
    """

    return dict(itertools.izip(limb.joint_names(), vel_torque_array))

def get_joint_positions(limb):
    """
    Returns the baxter joint positions IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint positions
    """
    return np.array([limb.joint_angles()[joint_name] for joint_name in limb.joint_names()])

def get_joint_velocities(limb):
    """
    Returns the baxter joint velocities IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`baxter_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint velocities
    """
    return np.array([limb.joint_velocities()[joint_name] for joint_name in limb.joint_names()])

def vec(*args):
    """
    all purpose function to get a numpy array of random things.  you can pass
    in a list, tuple, ROS Point message.  you can also pass in:
    vec(1,2,3,4,5,6) which will return a numpy array of each of the elements 
    passed in: np.array([1,2,3,4,5,6])
    """
    if len(args) == 1:
        if type(args[0]) == tuple:
            return np.array(args[0])
        elif ros_enabled and type(args[0]) == Point:
            return np.array((args[0].x, args[0].y, args[0].z))
        else:
            return np.array(args)
    else:
        return np.array(args)

def hat(v):
    """
    See https://en.wikipedia.org/wiki/Hat_operator or the MLS book

    Parameters
    ----------
    v : :obj:`numpy.ndarrray`
        vector form of shape 3x1, 3x, 6x1, or 6x

    Returns
    -------
    3x3 or 6x6 :obj:`numpy.ndarray`
        hat version of the vector v
    """
    if v.shape == (3, 1) or v.shape == (3,):
        return np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    elif v.shape == (6, 1) or v.shape == (6,):
        return np.array([
                [0, -v[5], v[4], v[0]],
                [v[5], 0, -v[3], v[1]],
                [-v[4], v[3], 0, v[2]],
                [0, 0, 0, 0]
            ])
    else:
        raise ValueError

def adj(g):
    """
    Adjoint of a rotation matrix.  See the MLS book

    Parameters
    ----------
    g : 4x4 :obj:`numpy.ndarray`
        Rotation matrix

    Returns
    -------
    6x6 :obj:`numpy.ndarray` 
    """
    if g.shape != (4, 4):
        raise ValueError

    R = g[0:3,0:3]
    p = g[0:3,3]
    result = np.zeros((6, 6))
    result[0:3,0:3] = R
    result[0:3,3:6] = hat(p) * R
    result[3:6,3:6] = R
    return result

def twist_from_tf(g):
    """
    Returns the twist version of a 2D rotation matrix
    Parameters
    ----------
    g : 3x3 :obj:`numpy.ndarray`
        2D rotation matrix

    Returns
    -------
    3x' :obj:`numpy.ndarray`
    """
    return vec(g[0,2], g[1,2], atan2(g[1,0], g[0,0]))

def rotation2d(theta):
    """
    2D rotation matrix from a single theta around the origin

    Parameters
    ----------
    theta : float

    Returns
    -------
    2x2 :obj:`numpy.ndarray`
    """
    return np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]
        ])

def rotation_3d(omega, theta): # added by ryan using 106A code
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    wn = np.linalg.norm(omega)
    rot = np.eye(3)+hat(omega)/np.linalg.norm(omega)*sin(wn*theta)+np.dot(hat(omega),hat(omega))/np.linalg.norm(omega)**2*(1-cos(wn*theta))
    print(rot)
    test1 = np.dot(hat(omega),hat(omega))/np.linalg.norm(omega)**2
    print(test1)


    return rot


def rigid(twist):
    """
    Returns a 3x3 Rotation Matrix version of a 2D twist

    Parameters
    ----------
    twist : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    3x3 :obj:`numpy.ndarray`
    """
    return np.array([
            [cos(twist[2]), -sin(twist[2]), twist[0]],
            [sin(twist[2]), cos(twist[2]), twist[1]],
            [0, 0, 1]
        ])

def look_at_general(origin, direction):
    """
    Creates a 3D Rotation Matrix at the origin such that the z axis is the same
    as the direction specified.  There are infinitely many of such matrices, 
    but we choose the one where the x axis is as vertical as possible.  

    Parameters
    ----------
    origin : 3x1 :obj:`numpy.ndarray`
    direction : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    4x4 :obj:`numpy.ndarray`
    """
    up = vec(0,0,1)
    z = normalize(direction)
    x = normalize(np.cross(up, z))
    y = np.cross(z, x) 

    result = np.eye(4)
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z
    result[0:3,3] = origin
    return result

def create_pose_from_rigid_transform(g):
    """
    takes a rotation matrix and turns it into a ROS Pose

    Parameters
    ----------
    g : 4x4 : :obj:`numpy.ndarray`

    Returns
    -------
    :obj:`geometry_msgs.msg.Pose`
    """
    position = tfs.translation_from_matrix(g)
    quaternion = tfs.quaternion_from_matrix(g)
    wpose = Pose()
    wpose.position.x = position[0]
    wpose.position.y = position[1]
    wpose.position.z = position[2]
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    return wpose

def create_pose_stamped_from_pos_quat(pos, quat, frame_id):
    """
    takes a position and quaternion and turns it into a ROS PoseStamped

    Parameters
    ----------
    pos : 3x1 : :obj:`numpy.ndarray`
    quat : 4x1 : :obj:`numpy.ndarray`


    Returns
    -------
    :obj:`geometry_msgs.msg.PoseStamped`
    """
    print(pos.shape)
    print(pos)
    wpose = PoseStamped()
    wpose.header.frame_id = frame_id
    wpose.pose.position.x = pos[0]
    wpose.pose.position.y = pos[1]
    wpose.pose.position.z = pos[2]
    wpose.pose.orientation.x = quat[0]
    wpose.pose.orientation.y = quat[1]
    wpose.pose.orientation.z = quat[2]
    wpose.pose.orientation.w = quat[3]
    return wpose

"""
def homog_3d(xi, theta): # added by ryan using 106A code

    # Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    # joint displacement.
    
    # Args:
    # xi - (6,) ndarray: the 3D twist
    # theta: the joint displacement

    # Returns:
    # g - (4,4) ndarary: the resulting homogeneous transformation matrix

    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    omega = np.array([xi[3],xi[4],xi[5]])
    v = np.array([xi[0],xi[1],xi[2]])
    r = rotation_3d(omega,theta)
    p = (np.eye(3)-r)*np.dot(hat(omega),v)+omega*omega.T*v*theta
    bot = np.array([0,0,0,1])
    test1 = omega*omega.T*v*theta
    print(v)
    #print(p)
    #print(bot)
    g = np.vstack((np.hstack((r,p)),bot))

    return g
"""
def get_end_effector_position(limb):
    
    # outputs the currect end effector position
    joints = get_joint_positions(limb) # array length 7

    raise NotImplementedError

def array_func_test(func_name, args, ret_desired): # taken from 106a code for testing
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')

if __name__ == "__main__":
    print('Testing...')


    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    #arg1 = np.array([0, 0, 1])
    #arg2 = np.pi/4
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)


    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)


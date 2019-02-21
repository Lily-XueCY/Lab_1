#!/usr/bin/env python
"""
Starter script for lab1. 
Author: Chris Correa
"""
import copy
import sys
import argparse
import time
import numpy as np
import signal

from paths.paths import LinearPath, CircularPath, MultiplePaths
from controllers.controllers import (
    PDWorkspaceVelocityController, 
    PDJointVelocityController, 
    PDJointTorqueController, 
    FeedforwardJointVelocityController
)
from utils.utils import *
from path_planner import PathPlanner

try:
    import rospy
    import tf
    import baxter_interface
    import moveit_commander
    from moveit_msgs.msg import DisplayTrajectory, RobotState
    from baxter_pykdl import baxter_kinematics
except:
    print 'Couldn\'t import ROS, I assume you\'re working on just the paths on your own computer'

#added for ar tag
    from geometry_msgs.msg import Point


def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)

    r = rospy.Rate(200)
    while (
        not listener.frameExists(from_frame) or not listener.frameExists(to_frame)
    ) and (
        not rospy.is_shutdown()
    ):
        print 'Cannot find AR marker {}, retrying'.format(tag_number)
        r.sleep()

    t = listener.getLatestCommonTime(from_frame, to_frame)
    tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
    return vec(tag_pos)

def get_trajectory(task, tag_pos, num_way, controller_name):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    if task == 'line':
        #worked start_pos
        # start_pos = np.array([0.752, 0.207, 0.200])

        #test for sqare
        start_pos = np.array([0.717, 0.124, 0.261])

        #for ar folloing
        #start_pos = 


        path =  LinearPath(5, kin, limb, start_pos, tag_pos)# call the paths here form paths.py
    elif task == 'circle':
        path = CircularPath(5, kin, limb, tag_pos)
    elif task == 'square':

        #for fixed points
        # z = 0.261
        # first = np.array([0.567, 0.162, z])
        # second = np.array([0.547, 0.314, z])
        # third= np.array([0.676, 0.314, z])
        # fourth = np.array([0.717, 0.124, z])

        # first = np.array([0.6, 0.1, 0.362])
        # second = np.array([0.7, 0.1, 0.362])
        # third= np.array([0.7, 0.2, 0.362])
        # fourth = np.array([0.6, 0.2, 0.362])

        # path = MultiplePaths(25, kin, limb, [first, second, third, fourth])


        #for ar tag
        path = MultiplePaths(20, kin, limb, [tag_pos_1, tag_pos_2, tag_pos_3, tag_pos_4])
    else:
        raise ValueError('task {} not recognized'.format(task))
    return path.to_robot_trajectory(num_way, controller_name!='workspace')

def get_trajectory2(task, tag_pos, num_way, controller_name, start_pos):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    if task == 'line':
        path =  LinearPath(5, kin, limb, start_pos, tag_pos)# call the paths here form paths.py
    
    else:
        raise ValueError('task {} not recognized'.format(task))
    return path.to_robot_trajectory(num_way, controller_name!='workspace')

def get_controller(controller_name):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'workspace':
        # YOUR CODE HERE
        # limb and kin get set in call to main
        kp_ind = .1
        kv_ind = 0
        Kp = np.array([kp_ind,kp_ind,kp_ind,kp_ind,kp_ind,kp_ind])#1
        Kv = np.array([kv_ind,kv_ind,kv_ind,kv_ind,kv_ind,kv_ind])
        controller = PDWorkspaceVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'jointspace':
        # YOUR CODE HERE
        Kp = np.array([1,1,1,1,1,1,1])
        Kv = np.array([0.1,0.1,0.1,0.1,0.1,0.1,0.1])
        controller = PDJointVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'torque':
        # YOUR CODE HERE
        kp_ind = 15
        Kp = np.array([30,30,kp_ind,kp_ind,kp_ind,10,10])
        Kv = np.array([3,1,1,1,1,1,1])
        controller = PDJointTorqueController(limb, kin, Kp, Kv)
    elif controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

if __name__ == "__main__":
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t 1 -ar 1 -c workspace -a left --log
    python scripts/main.py -t 2 -ar 2 -c velocity -a left --log
    python scripts/main.py -t 3 -ar 3 -c torque -a right --log
    python scripts/main.py -t 1 -ar 4 5 --path_only --log

    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, square.  Default: line'
    )
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='workspace', 
        help='Options: workspace, jointspace, or torque.  Default: workspace'
    )
    parser.add_argument('-arm', '-a', type=str, default='left', help=
        'Options: left, right.  Default: left'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=300, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--moveit', action='store_true', help=
        """If you set this flag, moveit will take the path you plan and execute it on 
        the real robot"""
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()




    rospy.init_node('moveit_node')
    # this is used for sending commands (velocity, torque, etc) to the robot 
    limb = baxter_interface.Limb(args.arm)
    # this is used to get the dynamics (inertia matrix, manipulator jacobian, etc) from the robot
    # in the current position, UNLESS you specify other joint angles.  see the source code
    # https://github.com/valmik/baxter_pykdl/blob/master/src/baxter_pykdl/baxter_pykdl.py
    # for info on how to use each method
    kin = baxter_kinematics(args.arm)

    # ADD COMMENT EHRE what?




    #A dummy position in order to make the square motion to work
    tag_pos = np.array([0.567, 0.162, 0.261])


    # for square
    tag_pos_1 = lookup_tag(4)[0]
    tag_pos_1 = np.array([-tag_pos_1[0], -tag_pos_1[1], tag_pos_1[2]])
    print("############ tag_pos_1 ############")
    print(tag_pos_1[0])

    tag_pos_2 = lookup_tag(5)[0]
    #for some reason, the x and y coordinateds of the extracted ar tag position by the "look_up tag" function is inverted
    #Thus, we need to invert it back by the following line of code.
    # tag_pos_2 = np.array([-tag_pos_2[0], -tag_pos_2[1], tag_pos_2[2]])

    print("############3 tag_pos_1 ############")
    print(tag_pos_2[0])

    tag_pos_3 = lookup_tag(9)[0]
    # tag_pos_3 = np.array([-tag_pos_3[0], -tag_pos_3[1], tag_pos_3[2]])
    print("############3 tag_pos_1 ############")
    print(tag_pos_3[0])


    tag_pos_4 = lookup_tag(12)[0]
    print("################before#################")
    print(tag_pos_4[0])
    # tag_pos_4 = np.array([-tag_pos_4[0], -tag_pos_4[1], tag_pos_4[2]])

    # print("############3 tag_pos_1 ############")
    # print(tag_pos_4[0])



    # #for follow ar track, uncomment the following block
    # tag_pos = lookup_tag(5)[0]
    # # tag_pos = np.array([abs(tag_pos[0]), abs(tag_pos[1]), abs(tag_pos[2])])
    # if abs(tag_pos[0]) > abs(tag_pos[1]):
    #     tag_pos = np.array([abs(tag_pos[0]), abs(tag_pos[1]), abs(tag_pos[2])])
    # else:
    #     tag_pos = np.array([abs(tag_pos[1]), abs(tag_pos[0]), abs(tag_pos[2])])

     





    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.
    robot_trajectory = get_trajectory(args.task, tag_pos, args.num_way, args.controller_name)
    

    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('{}_arm'.format(args.arm))
    if args.controller_name == "workspace":
        pose = create_pose_stamped_from_pos_quat(
            robot_trajectory.joint_trajectory.points[0].positions,
            [0, 1, 0, 0],
            'base'
        )
        plan = planner.plan_to_pose(pose)
    else:
        plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan)
    #print('part 1 done')

    if args.moveit:
        # LAB 1 PART A
        # by publishing the trajectory to the move_group/display_planned_path topic, you should 
        # be able to view it in RViz.  You will have to click the "loop animation" setting in 
        # the planned path section of MoveIt! in the menu on the left side of the screen.
        pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        disp_traj = DisplayTrajectory()
        disp_traj.trajectory.append(robot_trajectory)
        # disp_traj.trajectory_start = planner._group.get_current_joint_values()
        disp_traj.trajectory_start = RobotState()
        pub.publish(disp_traj)



        try:
            raw_input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # uses MoveIt! to execute the trajectory.  make sure to view it in RViz before running this.
        # the lines above will display the trajectory in RViz


        planner.execute_plan(robot_trajectory)
    else:
        # LAB 1 PART B
        controller = get_controller(args.controller_name)
        try:
            raw_input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()

        #for followinging ar tag, uncomment below
        # done = controller.follow_ar_tag(
        #     get_trajectory2_func=get_trajectory2,
        #     tag=5,
        #     controller_name=args.controller_name,
        #     rate=args.rate,
        #     timeout=args.timeout,
        #     log=args.log
        # )


        done = controller.execute_path( 
            robot_trajectory,
            rate=args.rate, 
            timeout=args.timeout, 
            log=args.log
        )
        if not done:
            print 'Failed to move to position'
            sys.exit(0)


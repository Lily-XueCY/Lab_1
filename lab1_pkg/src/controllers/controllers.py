#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa, Valmik Prabhu
"""

# Python imports
import sys
import numpy as np
import itertools
import matplotlib.pyplot as plt

# Lab imports
from utils.utils import *
from listener import listener

# ROS imports
try:
    import tf
    import rospy
    import baxter_interface
    import intera_interface
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotTrajectory
except:
    pass
from paths.paths import *

NUM_JOINTS = 7

class Controller:

    def __init__(self, limb, kin):
        """
        Constructor for the superclass. All subclasses should call the superconstructor

        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        """

        # Run the shutdown function when the ros node is shutdown
        rospy.on_shutdown(self.shutdown)
        self._limb = limb
        self._kin = kin

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly.  

        Parameters
        ----------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        """
        pass

    def interpolate_path(self, path, t, current_index = 0):
        """
        interpolates over a :obj:`moveit_msgs.msg.RobotTrajectory` to produce desired
        positions, velocities, and accelerations at a specified time

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        t : float
            the time from start
        current_index : int
            waypoint index from which to start search

        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        current_index : int
            waypoint index at which search was terminated 
        """

        # a very small number (should be much smaller than rate)
        epsilon = 0.0001

        max_index = len(path.joint_trajectory.points)-1

        # If the time at current index is greater than the current time,
        # start looking from the beginning
        if (path.joint_trajectory.points[current_index].time_from_start.to_sec() > t):
            current_index = 0

        # Iterate forwards so that you're using the latest time
        while (
            not rospy.is_shutdown() and \
            current_index < max_index and \
            path.joint_trajectory.points[current_index+1].time_from_start.to_sec() < t+epsilon
        ):
            current_index = current_index+1

        # Perform the interpolation
        if current_index < max_index:
            time_low = path.joint_trajectory.points[current_index].time_from_start.to_sec()
            time_high = path.joint_trajectory.points[current_index+1].time_from_start.to_sec()

            target_position_low = np.array(
                path.joint_trajectory.points[current_index].positions
            )
            target_velocity_low = np.array(
                path.joint_trajectory.points[current_index].velocities
            )
            target_acceleration_low = np.array(
                path.joint_trajectory.points[current_index].accelerations
            )

            target_position_high = np.array(
                path.joint_trajectory.points[current_index+1].positions
            )
            target_velocity_high = np.array(
                path.joint_trajectory.points[current_index+1].velocities
            )
            target_acceleration_high = np.array(
                path.joint_trajectory.points[current_index+1].accelerations
            )

            target_position = target_position_low + \
                (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + \
                (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)
            target_acceleration = target_acceleration_low + \
                (t - time_low)/(time_high - time_low)*(target_acceleration_high - target_acceleration_low)

        # If you're at the last waypoint, no interpolation is needed
        else:
            target_position = np.array(path.joint_trajectory.points[current_index].positions)
            target_velocity = np.array(path.joint_trajectory.points[current_index].velocities)
            target_acceleration = np.array(path.joint_trajectory.points[current_index].velocities)

        return (target_position, target_velocity, target_acceleration, current_index)


    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.stop_moving()
        rospy.sleep(0.1)

    def stop_moving(self):
        """
        Set robot joint velocities to zero
        """
        zero_vel_dict = joint_array_to_dict(np.zeros(NUM_JOINTS), self._limb)
        self._limb.set_joint_velocities(zero_vel_dict)



    def plot_results(
        self,
        times,
        actual_positions, 
        actual_velocities, 
        target_positions, 
        target_velocities
    ):
        """
        Plots results.
        If the path is in joint space, it will plot both workspace and jointspace plots.
        Otherwise it'll plot only workspace

        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx7 or nx6 :obj:`numpy.ndarray`
            actual joint positions for each time in times
        actual_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            actual joint velocities for each time in times
        target_positions: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace positions for each time in times
        target_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)

        # Find the actual workspace positions and velocities
        actual_workspace_positions = np.zeros((len(times), 3))
        actual_workspace_velocities = np.zeros((len(times), 3))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(actual_positions[i], self._limb)
            actual_workspace_positions[i] = \
                self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
            actual_workspace_velocities[i] = \
                self._kin.jacobian(joint_values=positions_dict)[:3].dot(actual_velocities[i])
        # check if joint space
        if target_positions.shape[1] > 3:
            # it's joint space

            target_workspace_positions = np.zeros((len(times), 3))
            target_workspace_velocities = np.zeros((len(times), 3))

            for i in range(len(times)):
                positions_dict = joint_array_to_dict(target_positions[i], self._limb)
                target_workspace_positions[i] = \
                    self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
                target_workspace_velocities[i] = \
                    self._kin.jacobian(joint_values=positions_dict)[:3].dot(target_velocities[i])

            # Plot joint space
            plt.figure()
            # print len(times), actual_positions.shape()
            joint_num = len(self._limb.joint_names())
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Position Error")

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Velocity Error")

            print "Close the plot window to continue"
            plt.show()

        else:
            # it's workspace
            target_workspace_positions = target_positions
            target_workspace_velocities = target_velocities

        plt.figure()
        workspace_joints = ('X', 'Y', 'Z')
        joint_num = len(workspace_joints)
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_workspace_positions[:,joint], label='Actual')
            plt.plot(times, target_workspace_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Position Error")

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_workspace_velocities[:,joint], label='Actual') #actual_workspace_velocities?
            plt.plot(times, target_workspace_velocities[:,joint], label='Desired')#ditto
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Velocity Error")

        print "Close the plot window to continue"
        plt.show()




    def execute_path(self, path, rate=200, timeout=None, log=False):
        """
        takes in a path and moves the baxter in order to follow the path.  

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(path, t, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break

        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True


    def lookup_tag2(self, tag_number):
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



    def follow_ar_tag(self, get_trajectory2_func, tag, controller_name, rate=200, timeout=None, log=False):
        """
        takes in an AR tag number and follows it with the baxter's arm.  You 
        should look at execute_path() for inspiration on how to write this. 

        Parameters
        ----------
        tag : int
            which AR tag to use
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """


        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        t=0
        count = 0
        while timeout is None or count < 5:
            print("!!!!!!!!!!!!!!In first while loop!!!!!!!!!!!!!!")



            cur_pos_quat = self._kin.forward_position_kinematics()
            angle_pos = get_joint_positions(self._limb)
            cur_pos = np.array([cur_pos_quat[0],cur_pos_quat[1],cur_pos_quat[2]])
            tag_pos = self.lookup_tag2(tag)[0]
            print("Time:", t)
            print("########first tag_pos in controller#########")
            print(tag_pos)

            #for some reason, the x y coordinates of the ar tag position are all negative and sometimes inverted.
            #Thus, we added the following block to make sure the ar tag position is passed in correctly.
            if abs(tag_pos[0]) > abs(tag_pos[1]):
                m_tag_pos = np.array([abs(tag_pos[0]), abs(tag_pos[1]), abs(tag_pos[2])])
            else:
                m_tag_pos = np.array([abs(tag_pos[1]), abs(tag_pos[0]), abs(tag_pos[2])])
            
            #m_tag_pos = np.array([abs(tag_pos[1]), abs(tag_pos[0]), abs(tag_pos[2])])
            print("########second tag_pos in controller#########")
            print(m_tag_pos)
            path = get_trajectory2_func("line", m_tag_pos, 300, controller_name, cur_pos)

            # For interpolation
            max_index = len(path.joint_trajectory.points)-1
            current_index = 0

            # For timing
            start_t = rospy.Time.now()
            r = rospy.Rate(rate)
            count = count + 1

            while not rospy.is_shutdown():
                # print("!!!!!!!!!!!!!!In second while loop!!!!!!!!!!!!!!")

                # Find the time from start
                t = (rospy.Time.now() - start_t).to_sec()

                # If the controller has timed out, stop moving and return false
                # if timeout is not None and t >= timeout:
                #     # Set velocities to zero
                #     self.stop_moving()
                #     return False

                current_position = get_joint_positions(self._limb)
                current_velocity = get_joint_velocities(self._limb)

                # Get the desired position, velocity, and effort
                (
                    target_position, 
                    target_velocity, 
                    target_acceleration, 
                    current_index
                ) = self.interpolate_path(path, t, current_index)

                if log:
                    times.append(t+(count-1)*6)
                    actual_positions.append(current_position)
                    actual_velocities.append(current_velocity)
                    target_positions.append(target_position)
                    target_velocities.append(target_velocity)

                # Run controller
                self.step_control(target_position, target_velocity, target_acceleration)

                # Sleep for a bit (to let robot move)
                r.sleep()

                if current_index >= max_index:
                    print("#######in break ############")
                    self.stop_moving()
                    break
        self.stop_moving()

        # For plotting
        


        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True




        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            # tag_pos = lookup_tag2(tag) # lookup_tag is not defined
            sub = listener()
            """
            We can't import from main, so I copied lookup_tag to utils and imported from there
            not sure if that's causing any problems. Still 
            """
            num_way = 200
            controller_name = 'jointspace'
            cur_pos_quat = self._kin.forward_position_kinematics()
            start_pos = np.array([cur_pos_quat[0], cur_pos_quat[1], cur_pos_quat[2]])
            path = LinearPath(10, self._kin, self._limb, start_pos, tag_pos)

            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(path, .2, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            print("####step_control time!!!!!#####")
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()
        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )

        return True
        # raise NotImplementedError

class FeedforwardJointVelocityController(Controller):
    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        self._limb.set_joint_velocities(joint_array_to_dict(target_velocity, self._limb))

class PDWorkspaceVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the
    PDJointVelocityController is that this controller compares the baxter's current WORKSPACE position and
    velocity desired WORKSPACE position and velocity to come up with a WORKSPACE velocity command to be sent
    to the baxter.  Then this controller should convert that WORKSPACE velocity command into a joint velocity
    command and sends that to the baxter.  Notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._kin.forward_psition_kinematics() and self._kin.forward_velocity_kinematics() to get 
        the current workspace position and velocity and self._limb.set_joint_velocities() to set 
        the joint velocity to something.  you may have to look at 
        http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html to convert the 
        output of forward_velocity_kinematics() to a numpy array.  You may find joint_array_to_dict() 
        in utils.py useful 

        MAKE SURE TO CONVERT QUATERNIONS TO EULER IN forward_position_kinematics().  
        you can use tf.transformations.euler_from_quaternion()

        your target orientation should be (0,0,0) in euler angles and (0,1,0,0) as a quaternion.  

        Parameters
        ----------
        target_position: 6x' ndarray of desired positions
        target_velocity: 6x' ndarray of desired velocities
        target_acceleration: 6x' ndarray of desired accelerations
        """

        """
        NOTE: it's very possible that I (Ryan) made a few syntax or other errors here.
        Hopefully my comments are useful enough that you can understand what we are trying
        to do with this controller. We also need to tune Kp and Kv (both 6x' for this controller).
        This will probably take a lot of trial and error, it's usually good to find a decent Kp first
        and then introduce Kv. It may also be good to begin tuning with only the feedback term and
        not the feedforward term.
        """
        # This part is self explanatory. We get arrays of 7 angles/velocities
        orient = np.array([3.14,0,3.14])
        zs = np.array([0,0,0])
        target_position = np.hstack([target_position,orient])
        target_velocity = np.hstack([target_velocity,zs])
        target_acceleration = np.hstack([target_acceleration,zs])

        angle_pos = get_joint_positions(self._limb)
        angle_vel = get_joint_velocities(self._limb)

        
        # Here we want to get a 6x' array of end effector position. The first line gives
        # us a 7x' array of xyz pos and the quaternion. We convert the quaternion to Euler
        # angles and then construct the 6x' array [x,y,z,ex,ey,ez]
        cur_pos_quat = self._kin.forward_position_kinematics()
        quat = np.array([cur_pos_quat[3], cur_pos_quat[4], cur_pos_quat[5], cur_pos_quat[6]])
        euler = tf.transformations.euler_from_quaternion(quat)
        cur_pos_euler = np.hstack([np.array([cur_pos_quat[0], cur_pos_quat[1], cur_pos_quat[2]]),euler])
        
        # twist is an object that contains vel and rot. We convert rot to euler angle velocities
        # and construct the 6x' that represents current velocity                                
        twist = self._kin.forward_velocity_kinematics()
        # Lily: I don't know why we just get a 6x' array
        vel = np.array([twist.vel[0], twist.vel[1], twist.vel[2]])
        rotv_e = np.array([twist.rot[0], twist.rot[1], twist.rot[2]])
        #rotv_e = tf.transformations.euler_from_quaternion(rot_q) 
        cur_vel = np.hstack([vel,rotv_e])
        
        # Finally we get to the controller. For a PD controller, we care about error and the derivative
        # of error. The feed back term is (Kp*e+Kv*e_dot). We want to set v = ff + fb = v_des + (Kp*e+Kv*e_dot).
        # Note that based off the way I defined error and feedback, it makes sense to have POSITIVE Kp and Kv.
        # However, we can only set robot joint velocities, and not end effector velocities. In other words, we have
        # x_dot, but need q_dot. We know x_dot=J*q_dot, where J is the Jacobian. Since the Jacobian isn't square, we 
        # use the psuedo-inverse to get q_dot = J_dag*x_dot. Finally, we have the joint velocities we need to set. 
        ff = target_velocity
        e = target_position - cur_pos_euler
        e_dot = target_velocity - cur_vel                                 
        fb = np.dot(self.Kp,e) + np.dot(self.Kv,e_dot)
        J_dag = self._kin.jacobian_pseudo_inverse() #do I put angle_pos into this?
        q_dot = np.dot(J_dag,(ff+fb))

        #convert 1x7 matrix to array
        q_dot =  np.array([q_dot.item(i) for i in range(7)])




        # print("########q_dot_dict######")
        # print(q_dot)
        # print(joint_array_to_dict(q_dot, self._limb))                                
        
        self._limb.set_joint_velocities(joint_array_to_dict(q_dot, self._limb))



class PDJointVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the 
    PDJointVelocityController is that this controller turns the desired workspace position and velocity
    into desired JOINT position and velocity.  Then it compares the difference between the baxter's 
    current JOINT position and velocity and desired JOINT position and velocity to come up with a
    joint velocity command and sends that to the baxter.  notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        # This part is self explanatory. We get arrays of 7 angles/velocities
        angle_pos = get_joint_positions(self._limb)
        angle_vel = get_joint_velocities(self._limb)
        
        # This is very similar to the PDWorkspace controller, but is simpler. See previous comments for explanation                                 
        e = target_position - angle_pos
        e_dot = target_velocity - angle_vel
        fb = np.dot(self.Kp,e) + np.dot(self.Kv,e_dot) 
        ff = target_velocity
        q_dot = ff + fb
        #print(q_dot)
        #print(joint_array_to_dict(q_dot, self._limb))                                      
        self._limb.set_joint_velocities(joint_array_to_dict(q_dot, self._limb))

class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Look in section 4.5 of MLS.

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
         # This part is self explanatory. We get arrays of 7 angles/velocities
        angle_pos = get_joint_positions(self._limb)
        angle_vel = get_joint_velocities(self._limb)

        e = target_position - angle_pos
        e_dot = target_velocity - angle_vel
        inertia = self._kin.inertia() # angle_pos?
        Jt = self._kin.jacobian_transpose()
        mcart = self._kin.cart_inertia()
        f = np.array([0,0,0.5,0,0,0])
        gravity = np.dot(np.dot(Jt,mcart),f)
        ff = np.dot(inertia,target_acceleration) + gravity
        fb = np.dot(self.Kp,e) + np.dot(self.Kv,e_dot)
        torque = ff + fb
        torque = np.array([torque.item(i) for i in range(7)])
 
        print("############# torque #################")
        print(torque)
        
        self._limb.set_joint_torques(joint_array_to_dict(torque, self._limb))


        # raise NotImplementedError









#######################                          
# GRAD STUDENTS ONLY ##                        
#######################                         
class WorkspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """

        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        raise NotImplementedError

class JointspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        raise NotImplementedError
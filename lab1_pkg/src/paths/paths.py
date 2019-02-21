#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa
"""
import numpy as np
import math
import matplotlib.pyplot as plt

from utils.utils import *

try:
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except:
    pass

class MotionPath:
    def __init__(self, limb, kin, total_time):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        total_time : float
            number of seconds you wish the trajectory to run for
        """
        self.limb = limb
        self.kin = kin
        self.total_time = total_time

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        pass

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        pass

    def plot(self, num=300):
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t) for t in times])
        target_velocities = np.vstack([self.target_velocity(t) for t in times])

        plt.figure()
        plt.subplot(3,2,1)
        plt.plot(times, target_positions[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Position")

        plt.subplot(3,2,2)
        plt.plot(times, target_velocities[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Velocity")
            
        plt.subplot(3,2,3)
        plt.plot(times, target_positions[:,1], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Y Position")

        plt.subplot(3,2,4)
        plt.plot(times, target_velocities[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Y Velocity")
            
        plt.subplot(3,2,5)
        plt.plot(times, target_positions[:,2], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Z Position")

        plt.subplot(3,2,6)
        plt.plot(times, target_velocities[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Z Velocity")

        plt.show()

    def to_robot_trajectory(self, num_waypoints=300, jointspace=True):
        """
        Parameters
        ----------
        num_waypoints : float
            how many points in the :obj:`moveit_msgs.msg.RobotTrajectory`
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  
        """
        traj = JointTrajectory()
        traj.joint_names = self.limb.joint_names()    
        points = []
        for t in np.linspace(0, self.total_time, num=num_waypoints):
            point = self.trajectory_point(t, jointspace) # make sure this point is the right type of message
            points.append(point)
            #print(point)

        # We want to make a final point at the end of the trajectory so that the 
        # controller has time to converge to the final point.
        # extra_point = self.trajectory_point(self.total_time, jointspace)

        # extra_point.time_from_start = rospy.Duration.from_sec(self.total_time + 1)
        # points.append(extra_point)


        traj.points = points
        traj.header.frame_id = 'base'
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    def trajectory_point(self, t, jointspace):
        """
        takes a discrete point in time, and puts the position, velocity, and
        acceleration into a ROS JointTrajectoryPoint() to be put into a 
        RobotTrajectory.  
        
        Parameters
        ----------
        t : float
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  

        Returns
        -------
        :obj:`trajectory_msgs.msg.JointTrajectoryPoint`
        """
        point = JointTrajectoryPoint()
        delta_t = .01
        if jointspace:
            x_t, x_t_1, x_t_2 = None, None, None
            ik_attempts = 0
            theta_t_2 = self.get_ik(self.target_position(t-2*delta_t))
            theta_t_1 = self.get_ik(self.target_position(t-delta_t))
            theta_t   = self.get_ik(self.target_position(t))
            # print(self.target_position(t))
            #theta_t = np.array(theta_t)
            # print(theta_t)
            
            # we said you shouldn't simply take a finite difference when creating
            # the path, why do you think we're doing that here? cause you're mean

            point.positions = theta_t
            point.velocities = (theta_t - theta_t_1) / delta_t
            point.accelerations = (theta_t - 2*theta_t_1 + theta_t_2) / (2*delta_t)

        else:
            point.positions = self.target_position(t)
            point.velocities = self.target_velocity(t)
            point.accelerations = self.target_acceleration(t)
        point.time_from_start = rospy.Duration.from_sec(t)
        return point

    def get_ik(self, x, max_ik_attempts=100):
        """
        gets ik
        
        Parameters
        ----------
        x : 3x' :obj:`numpy.ndarray`
            workspace position of the end effector
        max_ik_attempts : int
            number of attempts before short circuiting

        Returns
        -------
        7x' :obj:`numpy.ndarray`
            joint values to achieve the passed in workspace position
        """
        #Johnny ediited the max_ik_attempts to 100. Previously is 10.
        #Johnny editted to debug
        # print("x: ", x)

        #Johnny editted to debug ^
        ik_attempts, theta = 0, None
        while theta is None and not rospy.is_shutdown():
            theta = self.kin.inverse_kinematics(
                position=x,
                orientation=[0, 1, 0, 0]
            )
            ik_attempts += 1
            if ik_attempts > max_ik_attempts:
                rospy.signal_shutdown(
                    'MAX IK ATTEMPTS EXCEEDED AT x(t)={}'.format(x)
                )

        #Johnny editted the following part for debugging purposes:
        # print(x)


        #Johnny ediited the above ^^^^

        return theta

class LinearPath(MotionPath):
    def __init__(self, total_time, kin, limb, start_pos, ar_tag_pos):
        """
        Remember to call the constructor of MotionPath (__init__)

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit

        """
        #raise NotImplementedError
        MotionPath.__init__(self, limb, kin, total_time)
        start_pos = np.array([abs(start_pos[0]), abs(start_pos[1]), abs(start_pos[2])])


        #The ar tracking function was not working properly so we added the block below 
        #to correct it.
        if start_pos[1]>start_pos[0]:
            a = start_pos[0]
            b = start_pos[1]
            start_pos[0] = b
            start_pos[1] = a
        print("### Modified start_pos ###")


        ar_tag_pos = np.array([abs(ar_tag_pos[0]), abs(ar_tag_pos[1]), abs(ar_tag_pos[2])])
        if ar_tag_pos[1]>ar_tag_pos[0]:
            a = ar_tag_pos[0]
            b = ar_tag_pos[1]
            ar_tag_pos[0] = b
            ar_tag_pos[1] = a
        self.start_pos = start_pos

        print("!!!!!!!!!!!!!!!!start_pos!!!!!!!!!!!!!!!!")
        print(start_pos)
        print("!!!!!!!!!ar_tag_pos in LinearPath!!!!!!!!!")
        print(ar_tag_pos)
        ar_tag_pos[2] =  start_pos[2]
        self.ar_tag_pos = ar_tag_pos
        self.points_generated = []

        



    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
            idk where exactly this would come from yet
        """
        # get joint positions and use fk to get end effector position?
        # ar_tag from topic

        cur_pos = self.target_velocity(time)*time + self.start_pos

        self.points_generated.append(cur_pos)
        #print(self.start_pos)
        # print(cur_pos)
        return cur_pos
              

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_position()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """

        avg_vel = (self.ar_tag_pos - self.start_pos)/self.total_time

        return avg_vel

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_velocity()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        return np.array([0, 0, 0])

class CircularPath(MotionPath):
    def __init__(self, total_time, kin, limb, ar_tag_pos):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        # raise NotImplementedError
        self.r = .1 
        MotionPath.__init__(self, limb, kin, total_time)
        self.ar_tag_pos = np.array([ar_tag_pos[0],ar_tag_pos[1],ar_tag_pos[2]])
        self.ar_tag_pos[2] =  0.282
        self.start_pos = [ar_tag_pos[0],ar_tag_pos[1]+self.r,ar_tag_pos[2]]
        self.w = 2*math.pi/self.total_time
        

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired x,y,z position in workspace coordinates of the end effector
        """

        x_pos = self.r*sin(self.w*time)+self.ar_tag_pos[0]
        y_pos = self.r*cos(self.w*time)+self.ar_tag_pos[1]
        z_pos = self.ar_tag_pos[2]
        # print(x_pos,y_pos)
        # raise NotImplementedError
        return np.array([x_pos,y_pos,z_pos])

    def target_velocity(self, time):
        """
        Returns the arm's desired velocity in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_position()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired x,y,z velocity in workspace coordinates of the end effector
        """

        x_v = self.w*self.r*cos(self.w*time)
        y_v = -self.w*self.r*sin(self.w*time)
        z_v = 0
        # raise NotImplementedError
        return np.array([x_v,y_v,z_v])

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_velocity()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired acceleration in workspace coordinates of the end effector
        """
        x_a = -self.w**2*self.r*sin(self.w*time)
        y_a = -self.w**2*self.r*cos(self.w*time)
        z_a = 0
        # raise NotImplementedError
        return np.array([x_a,y_a,z_a])


class MultiplePaths(MotionPath):
    """
    Remember to call the constructor of MotionPath
    
    You can implement multiple paths a couple ways.  The way I chose when I took
    the class was to create several different paths and pass those into the 
    MultiplePaths object, which would determine when to go onto the next path.
    """
    def __init__(self, total_time, kin, limb, points):
        MotionPath.__init__(self, limb, kin, total_time)

        #Johnny editted: The order of coordinates needs to be determined prior to initializing MutiplePath object

        #Possible implementation to allocate the time more evenly by calculate the norm of each vector:
        # length_1 =  length(points[1] - points[0])
        # length_2 =  length(points[2] - points[1])
        # length_3 =  length(points[3] - points[2])
        # length_4 =  length(points[0] - points[1])
        # total_length = length_1 + length_2 + length_3 + length_4
        # time_1 =  total_time * length_1/total_length
        # time_2 =  total_time * length_2/total_length
        # time_3 =  total_time * length_3/total_length
        # time_4 =  total_time * length_4/total_length


        self.points = points
        self.cur_start = 0
        self.last_checkpoint_time = 0
        self.total_time = total_time
        self.path1 = LinearPath(self.total_time/4, kin, limb, points[1], points[2])
        self.path2 = LinearPath(self.total_time/4, kin, limb, points[2], points[3])
        self.path3 = LinearPath(self.total_time/4, kin, limb, points[3], points[0])
        self.path4 = LinearPath(self.total_time/4, kin, limb, points[0], points[1])


    def get_current_path(self):
        """
        Johnny added:
        Returns an integer that represents which segment of the rectangle the robot is currently executing.

        """
        raise NotImplementedError

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float        

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired position in workspace coordinates of the end effector
        """
        """
        start_pos = self.points[self.cur_start]
        seg_time = time - self.last_checkpoint_time

        #The arguement of target-velocity dosent matter
        cur_pos = self.target_velocity(time)*seg_time + start_pos

        
        # or time > (self.total_time / 4)*(self.cur_start + 1)
        cur_pos_norm = length(cur_pos - start_pos)

        next_corner = self.points[(self.cur_start + 1)%4]
        
        seg_norm = length(next_corner - start_pos)
        print("cur_pos : ", cur_pos, "segment: ", self.cur_start, seg_norm - cur_pos_norm)

        if cur_pos_norm >= seg_norm:
            self.cur_start = (self.cur_start + 1) % 4
            self.last_checkpoint_time = time
        return cur_pos
        """

        #Possibly use rospy.sleep()
        total_time = self.total_time


        if time < total_time/4:
            return self.path1.target_position(time)

        elif time - total_time/4 == 0:
            rospy.sleep(0.5)

        elif time < total_time/2:
            return self.path2.target_position(time - (total_time/4 + 0.5))
            # return self.path2.target_position(time - (total_time/4 ))


        elif time - total_time/2 == 0:
            rospy.sleep(0.5)

        elif time <= total_time/4*3:
            return self.path3.target_position(time - (total_time/2 + 1))
            # return self.path3.target_position(time - (total_time/2))


        elif time - total_time/4*3 == 0:
            rospy.sleep(0.5)

        else:
            return self.path4.target_position(time - (total_time/4*3 + 1.5))
            # return self.path4.target_position(time - (total_time/4*3))

    def target_velocity(self, time):
        """
        Returns the arm's desired velocity in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        """
        start_point = self.points[self.cur_start]
        cur_target = self.points[(self.cur_start + 1) % 4]
        total_time = self.total_time / 4
        avg_vel = (cur_target - start_point)/ total_time
        return avg_vel
        """
        total_time = self.total_time
        if time <= self.total_time/4:
            return self.path1.target_velocity(time)

        elif time - total_time/4 == 0:
            rospy.sleep(0.5)

        elif time <= self.total_time/2:
            return self.path2.target_velocity(time - (total_time/4 + 0.5))

        elif time - total_time/2 == 0:
            rospy.sleep(0.5)

        elif time <= self.total_time/4*3:
            return self.path3.target_velocity(time - (total_time/2 + 1))

        elif time - total_time/4*3 == 0:
            rospy.sleep(0.5)


        else:
            return self.path4.target_velocity(time - (total_time/4*3 + 1.5))

    def target_acceleration(self, time):
        """
        Returns the arm's desired acceleration in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        #return np.array([0, 0, 0])
        if time <= self.total_time/4:
            return self.path1.target_acceleration(time)
        elif time <= self.total_time/2:
            return self.path2.target_acceleration(time)
        elif time <= self.total_time/4*3:
            return self.path3.target_acceleration(time)
        else:
            return self.path4.target_acceleration(time)

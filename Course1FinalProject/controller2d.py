#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self.wheel_base_guess    = 5.0

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        # for i in range(len(self._waypoints)):
        #     dist = np.linalg.norm(np.array([
        #             self._waypoints[i][0] - self._current_x,
        #             self._waypoints[i][1] - self._current_y]))
        #     if dist < min_dist:
        #         min_dist = dist
        #         min_idx = i
        min_idx = self.get_closest_waypoint(self._current_x, self._current_y)         
                
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def get_closest_waypoint(self, x, y):
        min_idx       = 0
        min_dist      = float("inf")
        
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - x,
                    self._waypoints[i][1] - y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i        
        
        return min_idx

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def get_front_rear_waypoints(self, idx, waypoints, x, y):
        if idx == 0:
            rear = 0
            front = idx+1
        elif (idx >= len(waypoints[0])):
            rear = idx-1
            front = idx
        else:
            behind_dist = np.linalg.norm(np.array([
                waypoints[idx-1, 0] - x,
                waypoints[idx-1, 1] - y]))
            forward_dist = np.linalg.norm(np.array([
                waypoints[idx+1, 0] - x,
                waypoints[idx+1, 1] - y]))
            if behind_dist > forward_dist:
                rear = idx - 1
                front = idx
            else:
                rear = idx
                front = idx + 1
        
        return rear, front


    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('int_v_err_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            v_err = (v_desired - v)
            kp_v = 0.5
            p_v_throttle = kp_v * v_err 
            ki_v = 0.1
            int_sat_bound = (1.0 - p_v_throttle) / ki_v
            i_v_error = self.vars.int_v_err_previous + v_err*(t-self.vars.t_previous)
            i_v_error = np.fmax(np.fmin(i_v_error, int_sat_bound), -int_sat_bound)
            i_v_throttle = ki_v*i_v_error
            
            accel_cmd = p_v_throttle + i_v_throttle
            
            if accel_cmd >= 0:
                throttle_output = accel_cmd 
                brake_output    = 0
            else:
                throttle_output = 0 
                brake_output    = -accel_cmd              

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            
            # calculate target index
            idx = self.get_closest_waypoint(x, y)
                        
            rear, front = self.get_front_rear_waypoints(idx, waypoints, x, y)
                    
            # calculate target yaw
            c_yaw = np.arctan2( (waypoints[front][1] - waypoints[rear][1]),
                               (waypoints[front][0] - waypoints[rear][0]) )
            
            #print(c_yaw*180.0/np.pi)
            
            # stanley controller implementation
            
            # yaw error
            theta_e = normalize_angle(c_yaw - yaw)
            
            # cross track error, feedforward
            # compute cross track error
            x_proj = x + self.wheel_base_guess*np.cos(yaw)
            y_proj = y + self.wheel_base_guess*np.sin(yaw)
            
            idx_proj = self.get_closest_waypoint(x_proj, y_proj)
            
            rear_proj, front_proj = self.get_front_rear_waypoints(idx_proj, waypoints, x_proj, y_proj)
            dy_proj = waypoints[front_proj][1] - waypoints[rear_proj][1]
            dx_proj = waypoints[front_proj][0] - waypoints[rear_proj][0]
            
            #print(front_proj)
            #print(rear_proj)
            
            norm_proj = np.hypot(dx_proj,dy_proj)
            yaw_proj = [dx_proj/norm_proj, dy_proj/norm_proj]
            
            #print(np.arctan2(yaw_proj[1],yaw_proj[0])*180.0/np.pi)
            
            dist_rear_vec = [self._waypoints[rear_proj][0] - x_proj, self._waypoints[rear_proj][1] - y_proj]
            
            dist_proj = np.linalg.norm(np.array([
                    dist_rear_vec[0],
                    dist_rear_vec[1]]))

            s_proj = np.dot(dist_rear_vec, yaw_proj)

            cte = 0
            
            if dist_proj**2 - s_proj**2 > 0:
                cte = np.sqrt(dist_proj**2 - s_proj**2)
            
            #print(cte)
            
            #print(np.sign(np.cross(dist_rear_vec, yaw_proj)))
            
            cte = -1.0*cte*np.sign(np.cross(dist_rear_vec, yaw_proj))
            ke = 0.2
            theta_d = np.fmin(np.fmax(np.arctan2(ke * cte, max(v, 1.0)),-0.5),0.5) 
            
            
            
            steer_output  = theta_e + theta_d
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)
            
        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.int_v_err_previous = i_v_error
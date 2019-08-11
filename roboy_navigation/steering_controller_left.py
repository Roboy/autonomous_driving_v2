#!/usr/bin/python
import argparse
from threading import Thread

from math import pi, floor

import rospy

from roboy_navigation.async_pid import AsyncPID
from roboy_navigation.steering_helper import TargetAngleListener, \
    AngleSensorListener, MyoMuscleController, rad_to_deg, \
    get_compensation


class SteeringController:

    def __init__(self,
                 fpga_id, left_motor_id, right_motor_id,
                 sample_rate=100, Kp=1, Ki=0, Kd=0,
                 min_displacement=10, max_displacement=300,
                 max_steering_angle_deg=30,
                 zero_angle_raw=2190,
                 right_angle_raw=2570, right_angle=30,
                 left_angle_raw=1810, left_angle=-30,
                 sim=False):
        self.angle_sensor_listener = AngleSensorListener(zero_angle_raw=zero_angle_raw,
                                                         right_angle_raw=right_angle_raw,
                                                         right_angle=right_angle,
                                                         left_angle_raw=left_angle_raw,
                                                         left_angle=left_angle)
        self.target_angle_listener = TargetAngleListener()
        self.muscle_controller = MyoMuscleController(fpga_id=fpga_id, 
                                                     left_motor_id=left_motor_id, 
                                                     right_motor_id=right_motor_id)
        self.right_pid = AsyncPID(target_val_provider=self.get_target_angle, 
                                  actual_val_provider=self.get_actual_angle, 
                                  control_callback=self.set_spring_displacement_right, 
                                  sample_rate=sample_rate, 
                                  Kp=Kp, Ki=Ki, Kd=Kd, 
                                  lower_limit=min_displacement, 
                                  upper_limit=max_displacement)
        self.left_pid = AsyncPID(target_val_provider=self.get_target_angle, 
                                 actual_val_provider=self.get_actual_angle, 
                                 control_callback=self.set_spring_displacement_left, 
                                 sample_rate=sample_rate, 
                                 Kp=-Kp, Ki=-Ki, Kd=-Kd, 
                                 lower_limit=min_displacement, 
                                 upper_limit=max_displacement)
        self.min_displacement = min_displacement
        self.max_displacement = max_displacement
        self.max_steering_angle_deg = max_steering_angle_deg
        self.max_steering_angle = float(max_steering_angle_deg) / 180 * pi
        self.compensation = get_compensation()
        self.right_comp = self.compensation[0][0]
        self.left_comp = self.compensation[0][1]
        self.update_param = True
        self.sim = sim

    def start(self):
        rospy.init_node('steering_controller_left')
        #TODO: refactor
        rospy.set_param('~wheel_base', 1.6)
        self.target_angle_listener.start()
        self.angle_sensor_listener.start()
        self.muscle_controller.start()
        self.left_pid.start()


    def get_target_angle(self):
        angle = self.target_angle_listener.get_latest_target_angle()
        return self.clip_bounds(angle)

    def get_actual_angle(self):
        actual_angle = self.angle_sensor_listener.get_latest_smooth_angle()
        if self.update_param:
            angle_deg_discrete = floor(actual_angle * 180 / pi)
            (self.right_comp, self.left_comp) = self.compensation[angle_deg_discrete]
       	    right_lowlim = self.min_displacement / self.right_comp
            right_upplim = self.max_displacement / self.right_comp
            left_lowlim = self.min_displacement / self.left_comp
            left_upplim = self.max_displacement / self.left_comp
            self.right_pid.set_limits(right_lowlim, right_upplim)
            self.left_pid.set_limits(left_lowlim, left_upplim)
        return actual_angle

    def set_spring_displacement_right(self, displacement):
        print(displacement)
        displacement *= self.right_comp
        print(displacement, 'right')
        self.muscle_controller.send_command_right(displacement)

    def set_spring_displacement_left(self, displacement):
        displacement *= self.left_comp
        print(displacement, 'left')
        self.muscle_controller.send_command_left(displacement)

    def clip_bounds(self, angle):
        if -self.max_steering_angle<= angle <= self.max_steering_angle:
            return angle
        rospy.logwarn('steering_helper.py: Requested target angle=%.2f is not '
                      'in the admissible bounds (%.2f, %.2f)',
                      rad_to_deg(angle),
                      -self.max_steering_angle_deg,
                      self.max_steering_angle_deg
                      )
        return min(max(angle, -self.max_steering_angle), self.max_steering_angle)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ROS node to control the steering myomuscles')
    parser.add_argument('--left_motor_id', type=int, required=True)
    parser.add_argument('--right_motor_id', type=int, required=True)
    parser.add_argument('--fpga_id', type=int, default=4)
    parser.add_argument('--sample_rate', type=int, default=100)
    parser.add_argument('--Kp', type=int, default=100)
    parser.add_argument('--Ki', type=int, default=0)
    parser.add_argument('--Kd', type=int, default=0)
    parser.add_argument('--max_disp', type=int, default=300)
    parser.add_argument('--min_disp', type=int, default=10)
    parser.add_argument('--max_steering_angle', type=int, default=30,
                        help='Max steering angle in degrees')
    parser.add_argument('--zero_angle_raw', type=int, default=2190)
    parser.add_argument('--right_angle_raw', type=int, default=2570)
    parser.add_argument('--right_angle', type=int, default=30)
    parser.add_argument('--left_angle_raw', type=int, default=1810)
    parser.add_argument('--left_angle', type=int, default=-30)
    parser.add_argument('--sim', type=bool, default=False)
    args, _ = parser.parse_known_args()
    print('steering_controller config:')
    print(args)
    SteeringController(
        args.fpga_id, args.left_motor_id, args.right_motor_id,
        args.sample_rate, args.Kp, args.Ki, args.Kd,
        args.min_disp, args.max_disp, args.max_steering_angle,
        args.zero_angle_raw, args.right_angle_raw, args.right_angle, 
        args.left_angle_raw, args.left_angle, 
        args.sim
    ).start()

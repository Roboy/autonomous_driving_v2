#!/usr/bin/python
import argparse
from math import pi, floor
import yaml
import os
import rospkg

import rospy

from roboy_navigation.async_pid import AsyncPID
from roboy_navigation.steering_helper import TargetAngleListener, \
    AngleSensorListener, MyoMuscleController, rad_to_deg


class SteeringController:

    def __init__(self,
                 fpga_id, motor_id, comp,
                 sample_rate=100, Kp=1, Ki=0, Kd=0,
                 min_displacement=10, max_displacement=300,
                 max_steering_angle_deg=30,
                 zero_angle_raw=2190,
                 right_angle_raw=2570, right_angle=30,
                 left_angle_raw=1810, left_angle=-30):
        self.angle_sensor_listener = AngleSensorListener(zero_angle_raw=zero_angle_raw,
                                                         right_angle_raw=right_angle_raw,
                                                         right_angle=right_angle,
                                                         left_angle_raw=left_angle_raw,
                                                         left_angle=left_angle)
        self.target_angle_listener = TargetAngleListener()
        self.muscle_controller = MyoMuscleController(fpga_id=fpga_id, 
                                                     motor_id=motor_id)
        self.motor_id = motor_id
        self.pid = AsyncPID(target_val_provider=self.get_target_angle, 
                                  actual_val_provider=self.get_actual_angle, 
                                  control_callback=self.set_spring_displacement, 
                                  sample_rate=sample_rate, 
                                  Kp=Kp, Ki=Ki, Kd=Kd, 
                                  lower_limit=min_displacement, 
                                  upper_limit=max_displacement)
        self.min_displacement = min_displacement
        self.max_displacement = max_displacement
        self.max_steering_angle_deg = max_steering_angle_deg
        self.max_steering_angle = float(max_steering_angle_deg) / 180 * pi
        self.compensation = comp
        self.update_param =True

    def start(self):
        node_name = 'steering_controller_motor_' + str(self.motor_id)
        rospy.init_node(node_name)
        #TODO: refactor
        rospy.set_param('~wheel_base', 1.6)
        self.target_angle_listener.start()
        self.angle_sensor_listener.start()
        self.muscle_controller.start()
        self.pid.start()

    def get_target_angle(self):
        angle = self.target_angle_listener.get_latest_target_angle()
        return self.clip_bounds(angle)

    def get_actual_angle(self):
        actual_angle = self.angle_sensor_listener.get_latest_smooth_angle()
        if self.update_param:
            clipped_angle = self.clip_bounds(actual_angle)
            angle_deg_discrete = str(int(floor(clipped_angle * 180 / pi)))
            self.comp = float(self.compensation[angle_deg_discrete])
       	    lower_limit = self.min_displacement / self.comp
            upper_limit = self.max_displacement / self.comp
            self.pid.set_limits(lower_limit, upper_limit)
        return actual_angle

    def set_spring_displacement(self, displacement):
        #displacement *= self.comp
        print(displacement)
        if displacement < self.min_displacement:
            displacement = self.min_displacement
        elif displacement > self.max_displacement:
            displacement = self.max_displacement
        print(displacement, self.motor_id)
        self.muscle_controller.send_command(displacement)

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
    config_path = os.path.join(rospkg.RosPack().get_path('roboy_navigation'), 'config')
    # electronics
    with open(os.path.join(config_path, 'electronics.yaml'), 'r') as ymlfile:
        electronics = yaml.safe_load(ymlfile) 
    # controller
    with open(os.path.join(config_path, 'controller.yaml'), 'r') as ymlfile:
        controller = yaml.safe_load(ymlfile)
    # compensation
    with open(os.path.join(config_path, 'compensation.yaml'), 'r') as ymlfile:
        compensation = yaml.safe_load(ymlfile)
    # calibration
    with open(os.path.join(config_path, 'calibration.yaml'), 'r') as ymlfile:
        calibration = yaml.safe_load(ymlfile)
    # geometry
    with open(os.path.join(config_path, 'geometry.yaml'), 'r') as ymlfile:
        geometry = yaml.safe_load(ymlfile)
    
    parser = argparse.ArgumentParser(description='ROS node to control the steering myomuscles')
    parser.add_argument('--motor_id', type=int)
    parser.add_argument('--fpga_id', type=int, default=electronics['IDs']['fpga_id'])
    parser.add_argument('--sample_rate', type=int, default=controller['sample_rate'])
    parser.add_argument('--Kp', type=int, default=controller['gains']['k_p'])
    parser.add_argument('--Ki', type=int, default=controller['gains']['k_i'])
    parser.add_argument('--Kd', type=int, default=controller['gains']['k_d'])
    parser.add_argument('--max_disp', type=int, default=controller['displacement']['max_disp'])
    parser.add_argument('--min_disp', type=int, default=controller['displacement']['min_disp'])
    parser.add_argument('--direction', type=str)
    args, _ = parser.parse_known_args()

    if args.direction == 'right':
        motor_id = electronics['IDs']['right_motor_id']
        comp = compensation['right_comp']
        args.Kp *= -1
        args.Ki *= -1
        args.Kd *= -1
    elif args.direction == 'left':
        motor_id = electronics['IDs']['left_motor_id']
        comp = compensation['left_comp']
    else:
        print('Wrong direction argument')
    if args.motor_id is not None:
        motor_id = args.motor_id

    print('steering_controller config:')
    print(args)
    config_path = os.path.join(rospkg.RosPack().get_path('roboy_navigation'), 'config')
    SteeringController(
        args.fpga_id, motor_id, comp,
        args.sample_rate, args.Kp, args.Ki, args.Kd,
        args.min_disp, args.max_disp, geometry['max_steering_angle'],
        calibration['raw']['raw_middle'], calibration['raw']['raw_right'], calibration['angles']['right'], 
        calibration['raw']['raw_left'], calibration['angles']['left']
    ).start()

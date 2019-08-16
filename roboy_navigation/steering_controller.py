#!/usr/bin/python
import argparse
from math import pi, floor

import rospy

from roboy_navigation.async_pid import AsyncPID
from roboy_navigation.steering_helper import TargetAngleListener, \
    AngleSensorListener, MyoMuscleController, rad_to_deg, \
    get_compensation


class SteeringController:

    def __init__(self,
                 fpga_id, motor_id,
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
        self.compensation = get_compensation(self.motor_id)
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
            self.comp = self.compensation[angle_deg_discrete]
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
    parser = argparse.ArgumentParser(description='ROS node to control the steering myomuscles')
    parser.add_argument('--motor_id', type=int, required=True)
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
    parser.add_argument('--direction', type=str)
    args, _ = parser.parse_known_args()
    if args.direction == 'right':
        pass
    elif args.direction == 'left':
        args.Kp *= -1
        args.Ki *= -1
        args.Kd *= -1
    else:
        print('wrong direction argument')
    print('steering_controller config:')
    print(args)
    SteeringController(
        args.fpga_id, args.motor_id,
        args.sample_rate, args.Kp, args.Ki, args.Kd,
        args.min_disp, args.max_disp, args.max_steering_angle,
        args.zero_angle_raw, args.right_angle_raw, args.right_angle, 
        args.left_angle_raw, args.left_angle
    ).start()

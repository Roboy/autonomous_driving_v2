#!/usr/bin/python
import argparse

from math import pi

import rospy

from roboy_navigation.async_pid import AsyncPID
from roboy_navigation.steering_helper import TargetAngleListener, \
    AngleSensorListener, MyoMuscleController, rad_to_deg


class SteeringController:

    def __init__(self,
                 fpga_id, left_motor_id, right_motor_id,
                 sample_rate=100, Kp=1, Ki=0, Kd=0,
                 min_displacement=10,
                 max_displacement=300,
                 max_steering_angle_deg=10,
                 zero_angle_raw=2600):
        self.angle_sensor_listener = AngleSensorListener(
            zero_angle_raw=zero_angle_raw
        )
        self.target_angle_listener = TargetAngleListener()
        self.muscle_controller = MyoMuscleController(
            fpga_id=fpga_id,
            left_motor_id=left_motor_id,
            right_motor_id=right_motor_id
        )
        self.async_pid = AsyncPID(
            target_val_provider=self.get_target_angle,
            actual_val_provider=self.get_actual_angle,
            control_callback=self.set_spring_displacement,
            sample_rate=sample_rate,
            Kp=Kp, Ki=Ki, Kd=Kd
        )
        self.min_displacement = min_displacement
        self.max_displacement = max_displacement
        self.max_steering_angle_deg = max_steering_angle_deg
        self.max_steering_angle = float(max_steering_angle_deg) / 180 * pi

    def start(self):
        rospy.init_node('steering_controller')
        #TODO: refactor
        rospy.set_param('~wheel_base', 1.6)
        self.target_angle_listener.start()
        self.angle_sensor_listener.start()
        self.muscle_controller.start()
        self.async_pid.start()

    def get_target_angle(self):
        angle = self.target_angle_listener.get_latest_target_angle()
        return self.clip_bounds(angle)

    def get_actual_angle(self):
        return self.angle_sensor_listener.get_latest_smooth_angle()

    def set_spring_displacement(self, displacement):
        print(displacement)
        disp_left, disp_right = (self.min_displacement, -displacement) if displacement < 0 \
            else (displacement, self.min_displacement)
        disp_left = max(disp_left, self.min_displacement)
        disp_right = max(disp_right, self.min_displacement)
        disp_left = min(disp_left, self.max_displacement)
        disp_right = min(disp_right, self.max_displacement)
        self.muscle_controller.send_command(disp_left, disp_right)

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
    parser.add_argument('--Kp', type=int, default=500000)
    parser.add_argument('--Ki', type=int, default=0)
    parser.add_argument('--Kd', type=int, default=100000)
    parser.add_argument('--max_disp', type=int, default=300)
    parser.add_argument('--min_disp', type=int, default=10)
    parser.add_argument('--max_steering_angle', type=int, default=10,
                        help='Max steering angle in degrees')
    parser.add_argument('--zero_angle_raw', type=int, default=2600)
    args, _ = parser.parse_known_args()
    print('steering_controller config:')
    print(args)
    SteeringController(
        args.fpga_id, args.left_motor_id, args.right_motor_id,
        args.sample_rate, args.Kp, args.Ki, args.Kd,
        args.min_disp, args.max_disp, args.max_steering_angle,
        args.zero_angle_raw
    ).start()

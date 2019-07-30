#!/usr/bin/python
import argparse

from math import pi, floor

import rospy

from roboy_navigation.async_pid import AsyncPID
from roboy_navigation.steering_helper import TargetAngleListener, \
    AngleSensorListener, MyoMuscleController, rad_to_deg, \
    get_compensation, testing_seq


class SteeringController:

    def __init__(self,
                 fpga_id, left_motor_id, right_motor_id,
                 sample_rate=100, Kp=1, Ki=0, Kd=0,
                 min_displacement=10,
                 max_displacement=300,
                 max_steering_angle_deg=30,
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
        self.right_pid = AsyncPID(
            target_val_provider=self.get_target_angle,
            actual_val_provider=self.get_actual_angle,
            control_callback=self.set_spring_displacement_right,
            sample_rate=sample_rate,
            Kp=Kp, Ki=Ki, Kd=Kd,
	    lower_limit=min_displacement, upper_limit=max_displacement
        )
	self.left_pid = AsyncPID(
	    target_val_provider=self.get_target_angle,
            actual_val_provider=self.get_actual_angle,
            control_callback=self.set_spring_displacement_left,
            sample_rate=sample_rate,
            Kp=-Kp, Ki=-Ki, Kd=-Kd,
	    lower_limit=min_displacement, upper_limit=max_displacement
        )
        self.min_displacement = min_displacement
        self.max_displacement = max_displacement
        self.max_steering_angle_deg = max_steering_angle_deg
        self.max_steering_angle = float(max_steering_angle_deg) / 180 * pi
        self.compensation = get_compensation()
        self.right_comp = self.compensation[0][0]
        self.left_comp = self.compensation[0][1]
        self.seq_counter = -1
        self.finished_instance = False
        self.testing_seq = testing_seq()

    def start(self):
        rospy.init_node('steering_controller')
        #TODO: refactor
        rospy.set_param('~wheel_base', 1.6)
        self.target_angle_listener.start()
        self.angle_sensor_listener.start()
        self.muscle_controller.start()
        self.right_pid.start()
        self.left_pid.start()

    def act_test_seq(self):
        self.right_pid.set_target_value_provider(self.internal_target_angle)
        self.left_pid.set_target_value_provider(self.internal_target_angle)
        self.seq_counter = 0
        for i in len(self.testing_seq):
            while not self.finished_instance:
                pass
            self.seq_counter += 1
            self.finished_instance = False
        self.seq_counter = -1

	self.right_pid.set_target_value_provider(self.get_target_angle)
	self.left_pid.set_target_value_provider(self.get_target_angle)

    def internal_target_angle(self):
	return self.testing_seq[counter][1]

    def check_if_finished(self, angle):
        if not (self.seq_counter == -1):
            if abs(angle - self.internal_target_angle()) <= 0.1:
                self.finished_instance = True

    def get_target_angle(self):
        angle = self.target_angle_listener.get_latest_target_angle()
        return self.clip_bounds(angle)

    def get_actual_angle(self, updata_param = True):
        acutal_angle = self.angle_sensor_listener.get_latest_smooth_angle()
        if update_param:
            angle_deg_discrete = floor(actual_angle * 180 / pi)
            (self.right_comp, self.left_comp) = self.compensation[angle_deg_discrete]
       	    right_lowlim = self.min_displacement / right_comp
            right_upplim = self.max_displacement / right_comp
            left_lowlim = self.min_displacement / left_comp
            left_upplim = self.max_displacement / left_comp
            self.right_pid.set_limits(right_lowlim, right_upplim)
            self.left_pid.set_limits(left_lowlim, left_upplim)
        return acutal_angle

    def set_spring_displacement_right(self, displacement):
        check_if_finished(get_acutal_angle(update_param=False))
        diplacement *= self.right_comp
        print(displacement)
        self.muscle_controller.send_command_right(displacement)

    def set_spring_displacement_left(self, displacement):
        displacement *= self.left_comp
        print(displacement)
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
    parser.add_argument('--Kp', type=int, default=500000)
    parser.add_argument('--Ki', type=int, default=0)
    parser.add_argument('--Kd', type=int, default=100000)
    parser.add_argument('--max_disp', type=int, default=300)
    parser.add_argument('--min_disp', type=int, default=10)
    parser.add_argument('--max_steering_angle', type=int, default=30,
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

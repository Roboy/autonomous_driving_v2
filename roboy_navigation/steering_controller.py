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
    """ This Class contains a ROS-node with the PID-controllers used to control the myo-muscles on the rickshaw. The node receives cmd_vel either from the planning container or the startup sequence and the angle values from the angle sensor on the rickshaw. The controller then calculates the needed displacement for a myo-muscle and sends the command to the fpga on the rickshaw. This node only controls one myo-muscle of the rickshaw. In order to use the two muscles on the rickshaw launch two intances of the node, which is automatically done by provided launch files.
    Usage:
    This ROS-node should be used via the startup.launch file, which also launches the startup sequence and the angle-converter.
    If you don't want to test the startup sequence use the control_start.launch file.
    
    ...

    Attributes
    ----------
    min_displacement : int
        minimum displacement a myo-muscles should receive. Should be >0 so the muscles will keep a constant strain.
    max_displacement : int
        maximum displacement a myo-muscle should receive. Limits the force a muscle can pull with.
    max_steering_angle_deg : int
        the maximum angle the rickshaw can steer to. Needed in order to check if the cmd_vel send is valid and reachable (degree).
    max_steering_angle : float
        maximum steering angle in radian (radian).
    compensation : dictionary
        gain scheduling parameters that compensate the non-linearity caused by the rotation of the rickshaw (not needed when implementing a two-point controller, could improve a more refined control-approach).
    update_param : bool
        enables the update of the compencsation as well as the anti-windup bounds (not working currently).
    motor_id : int
        id of the slave-select pin the motor is connected to on the FPGA (check wiring).
    angle_sensor_listener : AngleSensorListener
        listens to the rickshaw angle, needed by the controller as an input.
    target_angle_listener : TargetAngleListener
        listens to the cmd_vel, needed by the controller as an input.
    muscle_controller : MyoMusclesController
        configures myo_muscles and publishes commands to the muscles.
    pid : AsyncPID
        PID-controller that calulates the command displacement the myo-muscle receives.

    Methods
    ----------
    start()
        starts the controller, angles listeners and configures the myo-muscles 
    get_target_angle()
        gets the current target angle from the cmd_vel command and passes it on to the PID-controller
    get_actual_angle()
        gets the current rickshaw angle and passes it on to the PID-controller, updates parameters
    set_spring_displacement()
        checks for the displacement bounds and sends the desired displacement via the muscle_controller to the myo_muscles
    clip_bounds()
       checks if the desired angle is in the admissable bound, if not, clips the angle and gives back an warning   
    """
    def __init__(self, fpga_id, motor_id, 
                 comp, sample_rate=100, 
                 Kp=1, Ki=0, Kd=0,
                 min_displacement=10, max_displacement=300,
                 max_steering_angle_deg=30):
        """
        :param fpga_id: ID of the fpga, that controls the myo_muscles
        :type fpga_id: int
        :param motor_id: ID of the motor, that is controlled (check wiring)
        :type motor_id: int
        :param comp: non_linear compensation, due to rotation
        :type comp: dict
        :param sample_rate: controller sample rate
        :type sample_rate: int
        :param Kp: Proportional-Gain of controller
        :type Kp: int
        :param Ki: Integral-Gain of controller
        :type Ki: int
        :param Kd: Dervitative-Gain of controller
        :type Kd: int
        :param min_displacement: minimum displacement of the myo-muscle
        :type min_displacement: int
        :param max_displacement: maximum displacement of the myo-muscle
        :type max_displacement: int
        :param max_steering_angle_deg: maximum angle the rickshaw can steer to (degree)
        :type max_steering_angle_deg: int
        """
        self.min_displacement = min_displacement
        self.max_displacement = max_displacement
        self.max_steering_angle_deg = max_steering_angle_deg
        self.max_steering_angle = float(max_steering_angle_deg) / 180 * pi
        self.compensation = comp
        self.update_param =True
        self.motor_id = motor_id
        self.angle_sensor_listener = AngleSensorListener()
        self.target_angle_listener = TargetAngleListener()
        self.muscle_controller = MyoMuscleController(fpga_id=fpga_id, 
                                                     motor_id=motor_id)
        self.pid = AsyncPID(target_val_provider=self.get_target_angle, 
                            actual_val_provider=self.get_actual_angle, 
                            control_callback=self.set_spring_displacement, 
                            sample_rate=sample_rate, 
                            Kp=Kp, Ki=Ki, Kd=Kd, 
                            lower_limit=min_displacement, 
                            upper_limit=max_displacement)
        

    def start(self):
        """
        Starts the ROS-node of the controller; starts the target angle listener for the controller; starts the angle sensor listener for the controller; 
        starts the internal controller for the muscles; starts the PID-controller for one steering muscle
        """
        node_name = 'steering_controller_motor_' + str(self.motor_id)
        rospy.init_node(node_name)
        #TODO: refactor
        rospy.set_param('~wheel_base', 1.6)
        self.target_angle_listener.start()
        self.angle_sensor_listener.start()
        self.muscle_controller.start()
        self.pid.start()

    def get_target_angle(self):
        """
        Gets the latest target angle from a cmd_vel and passes it to the PID-controller
        :returns: clipped target angle in radian (float)
        """
        angle = self.target_angle_listener.get_latest_target_angle()
        return self.clip_bounds(angle)

    def get_actual_angle(self):
        """
        Gets the latest smooth angle from the angle sensor and passes it to the PID-controller
        Optionally updates the controller parameters and non-linear compensation
        :returns: current smooth angle in radian (float)
        """
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
        """
        Passes on the control value (displacement) to the internal muscle control; clips the displacement if too low or too high
        :param displacement: displacement value the internal control should reach
        :type displacement: int

        :returns: nothing
        """
        #displacement *= self.comp
        print(displacement)
        if displacement < self.min_displacement:
            displacement = self.min_displacement
        elif displacement > self.max_displacement:
            displacement = self.max_displacement
        print(displacement, self.motor_id)
        self.muscle_controller.send_command(displacement)

    def clip_bounds(self, angle):
        """
        gives out a warning if a desired angle is out of bounds and clips this value to be in range of the maximum steering angle
        :param angle: desired angle for steering (radian)
        :type angle: float
        
        :returns: clipped steering angle (radian, float)
        """
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
    # read the yaml-config files
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
    SteeringController(args.fpga_id, motor_id, 
                       comp,
                       args.sample_rate, 
                       args.Kp, args.Ki, args.Kd,
                       args.min_disp, args.max_disp, 
                       geometry['max_steering_angle'] ).start()

"""
An asynchronous PID controller.
"""
import rospy

from simple_pid import PID
from threading import Thread

from roboy_navigation.srv import *


class AsyncPID:
    """ This Class contains a configurable PID-controller, that is used for the high-level controller of a steering-myo-muscle. It also sets up a ROS-service to reconfigure the controller.
    Usage:
    This class is used in the steering_controller.py file and acts as the controller determining the wanted displacement for the internal myo-muscle control.
    INFO: This class was mainly written by AD in WiSe18/19 and this documentation was completed by AD2 in SuSe19, therefore documentation may differ from actual functionality.

    ...

    Attributes
    ----------
    target_val_provider : callable
        returns the target value for the controller.
    actual_val_provider : callable
        returns the actual value of the controller variable.
    control_callback : callable
        accepts control signal.
    sample_rate : int 
        rate at which to run controller, in hertz.
    pid : PID
        PID controller for high-level myo-muscle control from simple_pid

    Methods
    ----------
    start()
        starts the subscriber
    _run()
        subscriber for target angle, converts Twist to angle.
    update_pid_config(cfg)
        gives back last known target angle (radian).
    set_limits(lower_limit, upper_limit)
        reconfigures the output limits of the PID-controller
        TODO: incorperate intro update_pid_config
    """

    def __init__(self, target_val_provider, actual_val_provider,
                 control_callback, sample_rate, Kp=100, Ki=0.1, Kd=0.05, 
		         lower_limit=10, upper_limit=300):
        """
        :param target_val_provider: callable, returns the target value for the
                controller.
        :type target_val_provider: callable
        :param actual_val_provider: callable, returns the actual value of the
                controller variable.
        :type actual_val_provider: callable
        :param control_callback: callable, accepts control signal.
        :type control_callback: callable
        :param sample_rate: rate at which to run controller, in hertz.
        :type int:
        :param Kp: P-Gain for controller
        :type Kp: int
        :param Ki: I-Gain for controller
        :type Ki: int
        :param Kd: D-Gain for controller
        :type Kd: int
	    :param lower_limit: anti-windup for the controller.
        :type int:
	    :param upper_limit: anti-windup for the controller.
        :type int:
        """
        self.setpoint_provider = target_val_provider
        self.input_provider = actual_val_provider
        self.control_callback = control_callback
        self.sample_rate = sample_rate
        self.pid = PID(Kp, Ki, Kd)
        self.pid.output_limits = (lower_limit, upper_limit)

    def start(self):
        """
        Starts the ROS-service for config and starts _run()
        :returns: nothing
        """
        print('started PID')
        rospy.Service('pid_config', PIDConfig, self.update_pid_config)
        Thread(target=self._run).run()

    def _run(self):
        """
        Starts the timed while loop (with rospy.Rate), that acts as the clock for the controller
        Feeds values in the controller, gets result and calls control_callback function
        :returns: nothing
        """
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            setpoint = self.setpoint_provider()
            input = self.input_provider()
            error = input - setpoint
            print('Error:', error, 'Input: ', input, 'Setpoint: ', setpoint)
            control = self.pid(error)
            self.control_callback(control)
            rate.sleep()

    def update_pid_config(self, cfg):
        """
        Reconfigures the values for the PID-controller
        :param cfg: Parameters for the PID-controller
        :type cfg: list
        :returns: bool, true if successful (not sure)
        """
        self.pid = PID(cfg.Kp, cfg.Ki, cfg.Kd)
        rospy.loginfo('PID reconfigured (%.2f, %.2f %.2f)',
                      cfg.Kp, cfg.Kd, cfg.Ki)
        return PIDConfigResponse(True)

    def set_limits(self, lower_limit, upper_limit):
        """
        Reconfigures the output limits for the PID-controller (anti-windup)
        Only needed, if the non-linear compensation is active
        :param lower_limit: lower limit for the contoller
        :type lower_limit: int
        :param upper_limit: upper limit for the controller
        :type upper_limit: int

        :returns: nothing
        """
        self.pid.ouput_limits = (lower_limit, upper_limit)
        print('output:limits', self.pid.output_limits)

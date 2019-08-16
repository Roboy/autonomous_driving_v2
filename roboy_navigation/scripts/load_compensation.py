import yaml
import numpy as np
from math import floor
import os



def get_compensation():

    left_motor = {}
    right_motor = {}

    pkg_location = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(os.path.dirname(__file__))))
    config_dir = os.path.join(pkg_location, 'config')
    print(config_dir)

    with open(os.path.join(config_dir, 'geometry.yaml'), 'r') as cfgfile:
        cfg = yaml.load(cfgfile)

    motors = cfg['Motors']
    force_points = cfg['ForcePoints']

    motor_r_static = np.array([motors['right_motor']['x'],
                               motors['right_motor']['y'],
                               motors['right_motor']['z']])
    motor_l_static = np.array([motors['left_motor']['x'],
                               motors['left_motor']['y'],
                               motors['left_motor']['z']])
    liftingarm_r = np.array([force_points['fp_rm']['x'],
                             force_points['fp_rm']['y'],
                             force_points['fp_rm']['z']])
    liftingarm_l = np.array([force_points['fp_lm']['x'],
                             force_points['fp_lm']['y'],
                             force_points['fp_lm']['z']])
    max_angle = cfg['max_steering_angle']

    for angle in range(-max_angle, max_angle, 1):
        angle_rad = (angle + 0.5) * np.pi /180
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        R = np.array(((c,-s,0),(s,c,0),(0,0,1)))
        # calc motor rotation postion
        mot_r = np.matmul(R, motor_r_static)
        mot_l = np.matmul(R, motor_l_static)

        tendon_r = mot_r - liftingarm_r
        tendon_l = mot_l - liftingarm_l
        cross_r = np.cross(liftingarm_r, tendon_r)
        cross_l = np.cross(liftingarm_l, tendon_l)

        right_motor[str(int(floor(angle)))] = str(abs(np.divide(np.linalg.norm(tendon_r), cross_r[2])))
        left_motor[str(int(floor(angle)))] = str(abs(np.divide(np.linalg.norm(tendon_l), cross_l[2])))
    
    compensation = dict(
        right_comp = right_motor,
        left_comp = left_motor
    )

    with open(os.path.join(config_dir, 'compensation.yaml'), 'w') as ymlfile:
        yaml.dump(compensation, ymlfile, default_flow_style=False)

if __name__ == '__main__':
    get_compensation()



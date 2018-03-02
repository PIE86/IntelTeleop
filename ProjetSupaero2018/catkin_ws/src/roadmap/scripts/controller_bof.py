import numpy as np
import rospy
from opt_control.msg import Control

# from irepa import NX, NU

OPT_CONTROL_ACTION_SERVER = 'solve_ocp'
SIMU_CMD = 'TODOOOOO'

rospy.wait_for_service(OPT_CONTROL_ACTION_SERVER)
rospy.loginfo('End of wait for control predictor')

# Control frequency
CPS = 20


class Controller:

    def __init__(self):
        # Last state trajectory calculated
        self.U = np.array([])
        self.t_idx = 0
        self.pub = rospy.Publisher(SIMU_CMD, Control, queue_size=10)



    def send_next_control(self):
        """Choose next control depending on the current_state and self.U
        Callback to service controller"""
        self.t_idx += 1
        u = self.U[self.t_idx, :]
        self.pub.publish(u)

    def update_U(self, msg):
        self.t_idx = 0
        print('ZODIZJAOD')
        print('UPDATE U')
        print(msg)


if __name__ == '__main__':
    controller = Controller()
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(CPS)
    while not rospy.is_shutdown():
        controller.send_next_control()
        rate.sleep()

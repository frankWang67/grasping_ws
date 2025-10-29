import rtde_receive
import rtde_control


class RTDEClient(object):
    def __init__(self, robot_ip='192.168.1.102'):
        self.robot_ip = robot_ip
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip, rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP, 50002)

    def get_joints(self):
        return self.rtde_r.getActualQ()
    
    def move_joints(self, q):
        self.rtde_c.moveJ(q, speed=0.2, acceleration=0.5)
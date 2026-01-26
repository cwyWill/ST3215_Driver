import numpy as np


class BallBalance:
    def __init__(self):
        self.l1 = 60
        self.l2 = 80
        self.l3 = 110
        self.l4 = 60
        self.g  = 9.81
        self.delta_list = [0, 2*np.pi/3, 4*np.pi/3]
    
    def IK(self, alpha, beta, h):
        ''' Inverse Kinematics: calculate the servo angles from platform orientation and height
        Arguments:
            alpha: rotation around x axis (rad)
            beta: rotation around y axis (rad)
            h: height of the platform (mm)
        Returns:
            theta_list: list of servo angles (rad)
        '''
        theta_list = [None] * 3
        for i in range(3):
            delta = self.delta_list[i]
            d = np.cos(alpha) * np.cos(beta) / np.sqrt(np.cos(alpha)**2 * np.sin(beta)**2 * np.cos(delta)**2 + 
                np.sin(alpha)**2 * np.sin(delta)**2 + np.cos(alpha)**2 * np.cos(beta)**2 \
                - 2 * np.cos(alpha) * np.sin(alpha) * np.sin(beta) * np.cos(delta) * np.sin(delta)) * self.l3
            px = d * np.cos(delta)
            py = d * np.sin(delta)
            pz = np.sqrt(self.l3**2 - d**2) + h
            P3 = np.array([px, py, pz])
            P1 = np.array([self.l4 * np.cos(delta), self.l4 * np.sin(delta), 0])
            r_sq = np.sum(np.square(P3 - P1))
            # r_sq = vector_square(P3, P1)
            r = np.sqrt(r_sq)
            cphi = (r**2 + self.l1**2 - self.l2**2) / (2 * r * self.l1)
            sphi = np.sqrt(1-cphi**2)
            phi = np.arctan2(sphi, cphi)
            
            s_sq = d**2 + pz**2
            cpsi = (self.l4**2 + r_sq - s_sq) / (2 * self.l4 * r)
            spsi = np.sqrt(1-cpsi**2)
            psi = np.arctan2(spsi, cpsi)
            
            t = np.pi - phi - psi
            theta_list[i] = t
        return theta_list

if __name__ == "__main__":
    bb = BallBalance()
    alpha = np.deg2rad(10)
    beta = np.deg2rad(5)
    h = 100
    theta_list = bb.IK(alpha, beta, h)
    print("Servo angles (degree):", np.rad2deg(theta_list))





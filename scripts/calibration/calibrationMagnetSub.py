#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import sys
import rospy
import numpy as np
from scipy import linalg
from geometry_msgs.msg import Point


class Magnetometer():
    ''' Magnetometer class with calibration capabilities.

        Parameters
        ----------
        sensor : str
            Sensor to use.
        F : float (optional)
            Expected earth magnetic field intensity, default=1.
    '''

    def __init__(self, sensor_topic, F=1.):

        # initialize values
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)
        self.first = True
        self.sub = rospy.Subscriber(sensor_topic, Point, self.calibrate, queue_size = 20)
        self.s = []
        

    def read(self):
        ''' Get a sample.

            Returns
            -------
            s : list
                The sample in uT, [x,y,z] (corrected if performed calibration).
        '''
        
        s = np.array(self.sensor.read()).reshape(3, 1)
        s = np.dot(self.A_1, s - self.b)
        return [s[0,0], s[1,0], s[2,0]]

    def calibrate(self, data):
        ''' Performs calibration. '''

        if len(self.s) < 1000:
            self.first = False
            stop_condition = rospy.Time.now() + rospy.Duration(60)
            self.s.append([data.x, data.y, data.z])
            print(len(self.s))
        
        else:
            print("Init calibrating!")

            # ellipsoid fit
            s = np.array(self.s).T
            M, n, d = self.__ellipsoid_fit(s)
            
            print("M")
            print(M)
            print("n")
            print(n)
            print("d")
            print(d)

            # calibration parameters
            # note: some implementations of sqrtm return complex type, taking real
            M_1 = linalg.inv(M)
            self.b = -np.dot(M_1, n)
            self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) *
                            linalg.sqrtm(M))
            
            print("M_1")
            print(M_1)
            print("b")
            print(self.b)
            print("A_1")
            print(self.A_1)

    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadric-form parameters
        M = np.array([[v_1[0], v_1[3], v_1[4]],
                      [v_1[3], v_1[1], v_1[5]],
                      [v_1[4], v_1[5], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d
    
    
    
    
if __name__ == '__main__':
    try:
        rospy.init_node("calibration")
        print('Reading data...')
        Magnetometer("magnet")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         

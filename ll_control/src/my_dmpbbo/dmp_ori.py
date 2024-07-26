#!/usr/bin/env python
from __future__ import division
import warnings

import numpy as np
from copy import copy, deepcopy
import matplotlib.pyplot as plt
import quaternion

class OriDmp():
    def __init__(self, ini_quat, ini_time, goal_quat, tau, **kwargs):       
        """_summary_

        Args:
            ini_quat (list[float] or np.array): 4-dim [w, x,y,z] normalized
            ini_time (float): 
            goal_quat (list[float] o np.array): 4-dim [w, x,y,z] normalized
            tau (float): 
        """
        self._ini_time = copy(ini_time)
        self._itau = 1/copy(tau)
        self._eta = np.zeros(3)
        
        
        self._ini_quat = np.quaternion(*ini_quat)
        self._goal_quat = np.quaternion(*goal_quat)

        if (self._ini_quat.norm() != 1):
            warnings.warn("Initial quaternion not normalized. Normalizing automatically.", RuntimeWarning)            
            self._ini_quat /= self._ini_quat.norm() 
            
        if (self._goal_quat.norm() != 1):
            warnings.warn("Goal quaternion not normalized. Normalizing automatically.", RuntimeWarning)            
            self._goal_quat /= self._goal_quat.norm()      
        
        
        
        self._q = deepcopy(self._ini_quat)
        self._ref_ori = deepcopy(self._ini_quat)
        
        
        self._time_history = [self._ini_time]
        self._quat_history = [self._ini_quat]
        #parameters
        self._alpha_z = 48.0
        self._beta_z = 12.0
        
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
                
    def set_goal(self, goal_quat):
        self._goal_quat = np.quaternion(*goal_quat)
    
    def get_tau(self):
        return copy(1/self._itau)
        
    def integrateOri(self, q, eta, dt):
        #assert isinstance(q, np.quaternion)
        post_log = np.log(self._goal_quat*np.conjugate(q))
        post_log = np.array([post_log.x, post_log.y, post_log.z])
        
        deta = self._alpha_z * (self._beta_z*2.0* post_log - self._eta)*self._itau
        
        pre_exp = dt*self._itau*self._eta/2.0
        pre_exp = np.quaternion(0, pre_exp[0], pre_exp[1], pre_exp[2])
        
        q = np.exp(pre_exp) * q
        #deta=1
        eta += dt*deta

        return q, eta
    
    def integrateOriInternal(self, dt):
        self._q, self._eta = self.integrateOri(self._q, self._eta, dt)
        self._quat_history.append(self._q)
        self._time_history.append(self._time_history[-1]+dt)
        return self._q


    def plot_traj(self):
        quat_tuples = [(quat.x, quat.y, quat.z, quat.w) for quat in self._quat_history]
        
        qx, qy, qz, qw = zip(*quat_tuples)
        
        fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

        # Plot each component in a separate subplot
        axs[0].plot(self._time_history, qw)
        axs[0].set_ylabel('w(t)')
        axs[0].legend()

        axs[1].plot(self._time_history, qx)
        axs[1].set_ylabel('x(t)')
        axs[1].legend()

        axs[2].plot(self._time_history, qy)
        axs[2].set_ylabel('y(t)')
        axs[2].legend()

        axs[3].plot(self._time_history, qz)
        axs[3].set_ylabel('z(t)')
        axs[3].set_xlabel('t')
        axs[3].legend()

        # Add a title to the figure
        fig.suptitle('Quaternion Components Over Time')

        # Display the plot
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    ori_dmp = OriDmp(np.array([1, 0,0,0]), 0.0, np.array([0.9, 0.43588989435, 0, 0]),3.0)
    for i in range(5001):
        q = ori_dmp.integrateOriInternal(1.0/1000.0)
    ori_dmp.plot_traj()
        
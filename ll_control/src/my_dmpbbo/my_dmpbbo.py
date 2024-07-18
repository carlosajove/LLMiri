#!/usr/bin/env python

from __future__ import division


import numpy as np
from copy import copy, deepcopy
import matplotlib.pyplot as plt


class KulDMP():

    def __init__(self, init_pos, init_time, goal, tau, **kwargs):
        """
            init_pos (): end-effector pos
            init_time (_type_): 0?
            goal (_type_): end-effector goal
            tau (_type_): time duration
        """
        
        self._tau = deepcopy(tau)
        print(tau)
        self._goal = deepcopy(goal)
        self._init_state = np.array([0, init_pos, init_pos, 1, 0])

        # Append the computes state and time history
        self._state_hist = {'euler': [deepcopy(self._init_state)], 'kuta': [deepcopy(self._init_state)]}  
        self._time_hist = {'euler': [deepcopy(init_time)], 'kuta': [deepcopy(init_time)]}
        
        
        #TODO 
        #self._dt_min = 1e-4  #sets the minimum integraition time step
        
        #Params come from dmpbbo implementation
        self._max_rate, self._inflection_ratio = self._sigmoid_system_for_gating(self._tau, 0.1)
        self._Ks_cached = None
        self._ks = self._get_ks()
        self._alpha_y = 20.0 #1.0 #20.0
        self._beta_y  = 5.0 #0.25  #5.0
        self._alpha_g = -15.0
        
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
                
        assert self._beta_y == self._alpha_y/4
    
    
    def dz(self, x):  #x = (z, y, g^, v, s)
        res = self._alpha_y*(self._beta_y*(x[2] - x[1]) - x[0]) 
        #res += add learned term not considered here
        return res/self._tau
    
    def dy(self, x):
        return x[0]/self._tau
    
    def dg(self, x):
        res = -self._alpha_g*(self._goal - x[2])
        return res/self._tau
        
    def dv(self, x):
        #res = -self._alpha_v*x[3]*(1 - x[3]/self._v_max)
        
        #TODO:Understand which are the parameters. Not important if not f_theta implemented
        res = -self._max_rate*x[3]*(1 - x[3]/self._ks)
        res = -self._ks*x[3]*(1 - x[3]/self._max_rate)
        return res/self._tau
    
    def ds(self, x):
        if (x[4] < 1): 
            return 1/self._tau
        return 0
    
    def df(self, x):
        return np.array([self.dz(x), 
                         self.dy(x),
                         self.dg(x),
                         self.dv(x),
                         self.ds(x)])
            
            
    #TODO: implement runge kuta
    def _integrate_step_euler(self, x, dt):
        f = x + dt*self.df(x)
        return f
    
       
    def integrate_euler_end_effector(self, x, dt):
        return self._integrate_step_euler(x, dt)[1]   
       
        
    def _integrate_step_runge_kutta(self, x, dt):
        u"""Integrate the system one time step using 4th order Runge-Kutta integration.

        See http://en.wikipedia.org/wiki/Runge-Kutta_method#The_Runge.E2.80.93Kutta_method


        @param dt: Duration of the time step
        @param x: Current state
        @return: (x_updated, xd_updated) - Updated state and its rate of change, dt time later.
        """
        k1 = self.df(x)
        input_k2 = x + dt * 0.5 * k1
        k2 = self.df(input_k2)
        input_k3 = x + dt * 0.5 * k2
        k3 = self.df(input_k3)
        input_k4 = x + dt * k3
        k4 = self.df(input_k4)

        x_updated = x + dt * (k1 + 2.0 * (k2 + k3) + k4) / 6.0
        #xd_updated = self.differential_equation(x_updated)
        return x_updated #, xd_updated  
        
  
    def integrate_step(self, dt, method):
        tr = False
        if method == 'euler':
            tr = True
            x = self._state_hist[method][-1]
            x_updated = self._integrate_step_euler(x, dt) 

        if method == 'kuta':
            tr = True
            x = self._state_hist[method][-1]
            print('kuta_x', x)
            x_updated = self._integrate_step_runge_kutta(x, dt)
        
        if(not tr):
              raise Exception("no integration method. supported methods: ['euler', 'kuta']") 
        
        self._state_hist[method].append(x_updated)
        self._time_hist[method].append(self._time_hist[method][-1]+dt)
        
        return x_updated
    

       
       
 
    """
    FROM DMPBBO LIB
    """
    
    def _get_ks(self):
        if self._Ks_cached is not None:
            # Cache available; simply return it.
            return self._Ks_cached

        # Variable rename so that it is the same as on the Wikipedia page
        N_0s = 1  # noqa

        # The idea here is that the initial state (called N_0s above), max_rate (r above) and the
        # inflection_ratio are set by the user.
        # The only parameter that we have left to tune is the "carrying capacity" K.
        #   http://en.wikipedia.org/wiki/Logistic_function#In_ecology:_modeling_population_growth
        # In the below, we set K so that the initial state is N_0s for the given r and tau

        # Known
        #   N(t) = K / ( 1 + (K/N_0 - 1)*exp(-r*t))
        #   N(t_inf) = K / 2
        # Plug into each other and solve for K
        #   K / ( 1 + (K/N_0 - 1)*exp(-r*t_infl)) = K/2
        #              (K/N_0 - 1)*exp(-r*t_infl) = 1
        #                             (K/N_0 - 1) = 1/exp(-r*t_infl)
        #                                       K = N_0*(1+(1/exp(-r*t_infl)))

        r = self._max_rate 
        t_infl = self._tau * self._inflection_ratio
        self._Ks_cached = N_0s * (1.0 + (1.0 / np.exp(-r * t_infl)))

            # If Ks is too close to N_0===initial_state, then the differential equation will always
            # return 0. See differential_equation below
            #   xd = max_rate_*x*(1-(x/Ks))
            # For initial_state this is
            #   xd = max_rate_*initial_state*(1-(initial_state/Ks))
            # If initial_state is very close/equal to Ks we get
            #   xd = max_rate_*Ks*(1-(Ks/Ks))
            #   xd = max_rate_*Ks*(1-1)
            #   xd = max_rate_*Ks*0
            #   xd = 0
            # And integration fails, especially for Euler integration.
            # So we now give a warning if this is likely to happen.
        div = np.divide(N_0s, self._Ks_cached) - 1.0
        if abs(div) < 10e-9:  # 10e-9 determined empirically
            print("In function SigmoidSystem, Ks is too close to N_0s. This may lead to errors ",
                "during numerical integration. Recommended solution: choose a lower magnitude ",
                "for the maximum rate of change (currently it is {r}) ".format(r))

        return self._Ks_cached
    
    def _sigmoid_system_for_gating(self, tau, y_tau_0_ratio=0.1, n_dims=1):
        # Known (analytical solution)
        #   N(t) = K / ( 1 + (K/N_0 - 1)*exp(-r*t))
        # Known in this function
        #   N_0 = 1, K = N_0 + D = 1 + D, N_tau = ratio*N_0
        #
        # Compute r = max_rate from the above
        #   N_tau = ratio*N_0
        #   N(tau) = N_tau = ratio*N_0 = K / ( 1 + (K/N_0 - 1)*exp(-r*tau))
        #   ratio = (1 + D) / ( 1 + ((1+D)/1 - 1)*exp(-r*tau))
        #   ratio = (1 + D) / ( 1 + D*exp(-r*tau))
        #   1 + D*exp(-r*tau) = (1 + D)/ratio
        #   exp(-r*tau) = (((1 + D)/ratio)-1)/D
        #   r = -log((((1 + D)/ratio)-1)/D)/tau

        # Choosing even smaller D leads to issues with Euler integration (tested empirically)
        d = 10e-7
        max_rate = -np.log((((1 + d) / y_tau_0_ratio) - 1) / d) / tau

        # Known (see _get_ks())
        #   K = N_0*(1+(1/exp(-r*t_infl)))
        # Known in this function
        #   N_0 = 1, K = N_0 + D = 1 + D, r < 0
        #
        # Compute inflection time from the above
        #   1 + D = 1*(1+(1/exp(-r*t_infl)))
        #   D = 1/exp(-r*t_infl)
        #   1/D = exp(-r*t_infl)
        #   -ln(1/D) = r*t_infl
        # The above defined a relationship between r and t_infl for a given D
        t_infl = -np.log(1 / d) / max_rate
        inflection_ratio = t_infl / tau

        return max_rate, inflection_ratio
        
    """
    """ 
        
    def plot_state_traj(self, method, show = False):
        name_list = ['z', 'y', 'g', 'v', 's' ]
        if method != 'euler' and method != 'kuta':
            assert Exception('method must be euler or kuta')
            
            
        for i in range(5):
            plt.figure()

            y = [state[i] for state in self._state_hist[method]]
            plt.plot(self._time_hist[method], y)
            plt.title(name_list[i] + ' ' + method)
            if show:
                plt.show(block=False)
        #if show:
        #    plt.show()
        
        
class MultiKulDmp():   
    def __init__(self, init_pos, init_time, goal, tau, **kwargs):
        """
            init_pos (np.array): N-d end-effector pos
            init_time (float): 0?
            goal (np.array): N-d end-effector goal
            tau (float): time duration
        """
        assert init_pos.ndim == 1
        assert init_pos.shape == goal.shape
        
        self._dim = len(init_pos)
        self._dmp_list = [KulDMP(init_pos[i], init_time, goal[i], tau) for i in range(self._dim)]
    
    
    
    def integrate_step(self, dt, method):
        res = []
        for dmp in self._dmp_list:
            res.append(dmp.integrate_step(dt, method)[1])
        return res
    
    def get_dmps(self):
        return self._dmp_list
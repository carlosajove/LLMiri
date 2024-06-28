import sys
import os
sys.path.insert(0, '/home/carribalzaga/dmpbbo')



import numpy as np
from copy import copy

from _dmpbbo.Dmp import Dmp


from _dmpbbo.FunctionApproximator import FunctionApproximator

class zero_forcing_term(FunctionApproximator):
    def __init__(self):
        self._dim_input = 1
        self._model_params = 0


    @staticmethod
    def _predict(inputs, model_params):
        return np.zeros_like(inputs)
    
    @staticmethod 
    def plot_model_parameters(self, inputs_min, inputs_max, **kwargs):
        print("plot model params")




class dmp_kul_traj():
    def __init__(self, tau, initial_state, final_state):
        self._tau = copy(tau)           #time constant
        self._ini_ef_pos = copy(initial_state)   #3d
        self._final_ef_pos = copy(final_state)
        self._dmp = Dmp(tau, initial_state, final_state, [zero_forcing_term(), zero_forcing_term(), zero_forcing_term()], dmp_type="KULVICIUS_2012_JOINING")
        
    def compute(self, t):
        xs_ana, xds_ana, forcing_terms_ana, fa_outputs_ana = self._dmp.analytical_solution(np.array([t]))

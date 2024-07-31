import math

def normalize_quaternion(q):
    norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    if norm == 0:
        return (0.0, 0.0, 0.0, 1.0)  
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


class LinearInterpolation():
    def __init__(self, start_pos, start_quat, end_pos, end_quat, duration):
        self._start_pos = start_pos 
        self._start_quat = start_quat
        self._end_pos = end_pos
        self._end_quat = end_quat
        self._duration = duration
        
    def compute_trajectory(self, t):
        pos =  (self._end_pose - self._start_pos)/self._duration * t
        quat = (self._end_quat - self._start_quat)/self._duration * t #do a quaternion trajectory // quaternions need normalisation
        
    
    
    
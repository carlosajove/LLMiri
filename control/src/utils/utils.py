import numpy as np
import quaternion

def NumpyfromGetPose(getpose):
    pos = np.array([getpose.position_x, getpose.position_y, getpose.position_z])
    ori = np.array([getpose.orientation_x, getpose.orientation_y, getpose.orientation_z, getpose.orientation_w])
    
    return pos, ori

def SetPosefromNumpy(pos, ori, setpose): #weight last quaternion
    """_summary_

    Args:
        pos (_type_): 3 dim
        ori (_type_): 4-dim quaternion [x,y,z,w]
        setpose (srv.Setpose): 
    """
    setpose.position_x = pos[0]
    setpose.position_y = pos[1]
    setpose.position_z = pos[2]
    setpose.orientation_x = ori[0]
    setpose.orientation_y = ori[1]
    setpose.orientation_z = ori[2]
    setpose.orientation_w = ori[3]
    
    
    
def PosefromNumpy(np_pos, np_ori, pose):
    """
    Fills a ROS geometry_msgs.Pose message with position and orientation data from NumPy arrays.

    Args:
        np_pos (np.array): A 3D NumPy array representing the position (x, y, z) in meters.
        np_ori (np.array): A 4D NumPy array representing the orientation as a quaternion (x, y, z, w).
        pose (geometry_msg.Pose): A ROS geometry_msgs.Pose message to be filled with the position and orientation data.

    Returns:
        None (modifies the pose message in-place).
    """
    pose.position.x = np_pos[0]
    pose.position.y = np_pos[1]
    pose.position.z = np_pos[2]
    pose.orientation.x = np_ori[0]
    pose.orientation.y = np_ori[1]
    pose.orientation.z = np_ori[2]
    pose.orientation.w = np_ori[3]



def quaternion_to_numpy(q, convention = 'xyzw'):
    """
        Converts a quaternion.quaternion to a numpy array based on the specified convention.
        Args:
            q (quaternion.quaternion):  The quaternion.quaternion to be converted.
            convention (str): The order of elements in the output array. 
                        Defaults to 'xyzw' (scalar-last). Supported options are:
                        'xyzw', 'wxyz'.
        Returns:
            A numpy array of shape (4,) containing the quaternion components 
            in the specified order.
    """
    if convention == 'xyzw': return np.array([q.x, q.y, q.z, q.w])
    if convention == 'wxyz': return np.array([q.w, q.x, q.y, q.z])
    
    
    
def convert_quaternion_convention(q, convention = 'xyzw'):
  """
  Converts a NumPy quaternion array between 'xyzw' and 'wxyz' conventions.

  Args:
      q (np.array): A 4D NumPy array representing the quaternion (must be in opposite convention to the q convention).
      convention (str): The desired output convention. Supported options are 'xyzw' and 'wxyz'.

  Returns:
      A new 4D NumPy array representing the quaternion in the specified convention.

  """
  if convention not in ('xyzw', 'wxyz'):
    raise ValueError("Unsupported convention. Supported options are 'xyzw' and 'wxyz'.")

  if convention == 'wxyz':
    return np.array([q[3], q[0], q[1], q[2]])  # wxyz
  else:
    return np.array([q[1], q[2], q[3], q[0]])  # xyzw
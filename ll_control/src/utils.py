import numpy as np

def NumpyfromGetPose(getpose):
    pos = np.array([getpose.position_x, getpose.position_y, getpose.position_z])
    ori = np.array([getpose.orientation_x, getpose.orientation_y, getpose.orientation_z, getpose.orientation_w])
    
    return pos, ori

def SetPosefromNumpy(pos, ori, setpose): #weight last quaternion
    print(setpose)
    print(setpose.position_x)
    print(pos[0])
    setpose.position_x = pos[0]
    setpose.position_y = pos[1]
    setpose.position_z = pos[2]
    setpose.orientation_x = ori[0]
    setpose.orientation_y = ori[1]
    setpose.orientation_z = ori[2]
    setpose.orientation_w = ori[3]
    
    
def PosefromNumpy(np_pos, np_ori, pose):
    pose.position.x = np_pos[0]
    pose.position.y = np_pos[1]
    pose.position.z = np_pos[2]
    pose.orientation.x = np_ori[0]
    pose.orientation.y = np_ori[1]
    pose.orientation.z = np_ori[2]
    pose.orientation.w = np_ori[3]
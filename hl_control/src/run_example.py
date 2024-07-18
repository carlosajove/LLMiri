import rospy

from gazebo_msgs.msg import ModelStates






    
    
    
rospy.spin()
while rospy.is_shutdown():
    
    pick_up_object_position = 1

    delivery_position =  1


class run_example():
    def __init__(self):
        rospy.init_node('run_example')

        gazebo_model_states_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.getModelStatesCallback)

        
        self._object_dict = {}
        rospy.spin()
        
    def getModelStatesCallback(self, data):
        for obj, pose, twist in zip(data.name, data.pose, data.twist):
            self._object_dict[obj] = {'pose': pose, 'twist': twist}
    
    
    def setObjectGoal(self, name):
        
#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import String
from llm_gazebo.msg import ObjectDescriptionList, NameDescriptionPair

def load_descriptions(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def publisher():
    print("dhfhefha kjASFA")
    rospy.init_node('object_description_publisher', anonymous=True)
    pub = rospy.Publisher('llm_context/object_description', ObjectDescriptionList, queue_size=1)
    rate = rospy.Rate(0.5)  # 1 Hz

    description_file = rospy.get_param('~description_file')
    descriptions = load_descriptions(description_file)

    while not rospy.is_shutdown():
        obj_list = []
        for obj in descriptions['objects']:
            a = NameDescriptionPair()
            a.name = obj['name']
            a.description = obj['description']
            obj_list.append(a)
        
        res = ObjectDescriptionList(objects=obj_list)
        rospy.loginfo("Publishing ObjectDescriptionList with %d objects", len(obj_list))
        pub.publish(res)    
        rate.sleep()
        


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
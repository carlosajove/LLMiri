#!/usr/bin/python

import roslaunch.rlutil
import rospy
import roslaunch

def main():
        
    rospy.init_node('testee')
    # Define the packages and launch files
    launch_list = [
        ['franka_gazebo', 'custom_panda.launch'],
        ['rosbridge_server', 'rosbridge_websocket.launch']
    ]
        # Add more (package, launch_file) tuples as needed

    # Create a UUID for the launch process
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    resolved_launch_args = []
    for launch_files in launch_list:
        resolved_launch_args.append(roslaunch.rlutil.resolve_launch_arguments(launch_files))
        print('res', roslaunch.rlutil.resolve_launch_arguments(launch_files))

    print('args', resolved_launch_args)
    launch_files = [(res[0]) for res in resolved_launch_args]
    # Set up the roslaunch parent with all the launch files
    print('paths', launch_files)
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    try:
        launch.start()
        rospy.loginfo("All launch files started successfully.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupted")
    finally:
        rospy.loginfo("Shutting down all launch files")
        launch.shutdown()

if __name__ == '__main__':
    main()

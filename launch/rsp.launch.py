import os
import rospy
import rospkg
import xacro
from xml.etree.ElementTree import Element, tostring

def generate_launch_description():
    # Initialize ROS node
    rospy.init_node('robot_description_launch', anonymous=True)

    # Get the package path
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('cps_rmp220_support')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Read parameters
    use_sim_time = rospy.get_param('~use_sim_time', False)
    use_ros_control = rospy.get_param('~use_ros_control', True)

    # Process the URDF file using xacro
    try:
        urdf_tree = xacro.process_file(
            xacro_file,
            mappings={
                'use_ros_control': str(use_ros_control).lower(),
                'sim_mode': str(use_sim_time).lower(),
            }
        )
        robot_description = tostring(urdf_tree.getroot(), encoding='unicode')
    except Exception as e:
        rospy.logerr(f"Error processing xacro file: {e}")
        robot_description = ""

    # Set the parameter for the robot description
    rospy.set_param('/robot_description', robot_description)
    rospy.set_param('/use_sim_time', use_sim_time)

    # Launch nodes
    state_publisher_command = "rosrun robot_state_publisher robot_state_publisher"
    joint_state_publisher_command = "rosrun joint_state_publisher joint_state_publisher"

    os.system(state_publisher_command)
    os.system(joint_state_publisher_command)

if __name__ == '__main__':
    try:
        generate_launch_description()
    except rospy.ROSInterruptException:
        pass

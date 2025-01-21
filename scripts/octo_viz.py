#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
import tf.transformations as tf
import os

def read_xyz_and_orientation_from_file(file_path):
    try:
        with open(file_path, 'r') as f:
            line = f.readline().strip()
            if not line:
                return None
            parts = line.split(',')
            if len(parts) < 6:
                rospy.logwarn("Not enough values in line to parse x, y, z, and orientation (roll, pitch, yaw)")
                return None
            try:
                # Position
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                # Orientation as Euler angles (roll, pitch, yaw)
                roll = float(parts[3])
                pitch = float(parts[4])
                yaw = float(parts[5])                
                gripper = float(parts[6])

                return (x, y, z, roll, pitch, yaw, gripper)
            except ValueError:
                rospy.logwarn("Could not parse floats from line: {}".format(line))
                return None
    except IOError:
        rospy.logerr("Could not open file: {}".format(file_path))
        return None

def compute_gradient_color(index, total):
    if total <= 1:
        return (0.0, 0.5, 1.0, 1.0)
    ratio = float(index) / (total - 1)
    r = ratio
    g = 0.5 * (1 - ratio)
    b = 1.0 - ratio
    a = 1.0
    return (r, g, b, a)

def transform_point(point, pose):
    translation_matrix = tf.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
    rotation_matrix = tf.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    transform = tf.concatenate_matrices(translation_matrix, rotation_matrix)

    point_homogeneous = [point[0], point[1], point[2], 1.0]
    transformed_point_homogeneous = transform.dot(point_homogeneous)
    return transformed_point_homogeneous[:3]

def transform_quaternion_to_robot_base(quaternion):
    """
    Keep the orientation in comparison to the robot base.
    This assumes the robot base is aligned with the fixed world frame.
    """
    return quaternion

def create_orientation_axis_marker(position, quaternion, marker_id):
    """
    Create a set of markers to represent the orientation of a point with an XYZ axis.
    """
    markers = []

    # Define the scale for the arrows
    arrow_scale = {
        "shaft_diameter": 0.02,
        "head_diameter": 0.05,
        "head_length": 0.05
    }

    # Rotation matrix from quaternion
    rotation_matrix = tf.quaternion_matrix(quaternion)

    # Define the directions of the X, Y, Z axes in local frame
    axes = {
        "x": [1, 0, 0],
        "y": [0, 1, 0],
        "z": [0, 0, 1]
    }
    colors = {
        "x": (1.0, 0.0, 0.0, 1.0),  # Red
        "y": (0.0, 1.0, 0.0, 1.0),  # Green
        "z": (0.0, 0.0, 1.0, 1.0)   # Blue
    }

    for axis_name, direction in axes.items():
        # Transform the local axis direction to world frame
        transformed_direction = rotation_matrix[:3, :3].dot(direction)

        # Create the arrow marker
        marker = Marker()
        marker.header.frame_id = "fr3_link0"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "orientation_axis"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = arrow_scale["shaft_diameter"]
        marker.scale.y = arrow_scale["head_diameter"]
        marker.scale.z = arrow_scale["head_length"]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = colors[axis_name]

        # Define the arrow's start and end points
        start_point = Point(x=position[0], y=position[1], z=position[2])
        end_point = Point(
            x=position[0] + transformed_direction[0] * 0.2,  # Scale the arrow length
            y=position[1] + transformed_direction[1] * 0.2,
            z=position[2] + transformed_direction[2] * 0.2
        )
        marker.points = [start_point, end_point]

        markers.append(marker)
        marker_id += 1  # Increment ID for each marker

    return markers

def create_interactive_marker(server):
    """
    Create an interactive marker to represent the camera axis in RViz.
    """
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "fr3_link0"
    int_marker.name = "camera_axis"
    int_marker.description = "Camera Axis"
    int_marker.scale = 0.5

    # Set the initial position of the marker
    int_marker.pose.position.x = 0.0
    int_marker.pose.position.y = 0.0
    int_marker.pose.position.z = 0.0
    int_marker.pose.orientation.w = 1.0

    # Add 6-DOF controls to the interactive marker
    for axis, orientation in zip(["x", "y", "z"], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]):
        # Translation control
        control = InteractiveMarkerControl()
        control.orientation.w = orientation[3]
        control.orientation.x = orientation[0]
        control.orientation.y = orientation[1]
        control.orientation.z = orientation[2]
        control.name = f"move_{axis}"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Rotation control
        control = InteractiveMarkerControl()
        control.orientation.w = orientation[3]
        control.orientation.x = orientation[0]
        control.orientation.y = orientation[1]
        control.orientation.z = orientation[2]
        control.name = f"rotate_{axis}"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

    server.insert(int_marker)
    server.applyChanges()
    return int_marker

def write_fixed_output_file(output_path, position, roll, pitch, yaw, final_value):
    try:
        with open(output_path, 'w') as f:
            f.write(f"{position[0]},{position[1]},{position[2]},{roll},{pitch},{yaw},{final_value}\n")
    except IOError:
        rospy.logerr("Could not write to file: {}".format(output_path))

def save_camera_location(file_path, position, orientation):
    """
    Save the current camera position and orientation to a file.
    """
    try:
        with open(file_path, 'w') as f:
            f.write(f"{position.x},{position.y},{position.z},{orientation.x},{orientation.y},{orientation.z},{orientation.w}\n")
    except IOError:
        rospy.logerr("Could not save camera location to file: {}".format(file_path))

def set_camera_startup_position(file_path, interactive_marker):
    """
    Set the starting position of the camera from a saved file.
    If the file does not exist, use the default position.
    """
    if os.path.exists(file_path):
        try:
            with open(file_path, 'r') as f:
                line = f.readline().strip()
                parts = line.split(',')
                if len(parts) == 7:
                    interactive_marker.position.x = float(parts[0])
                    interactive_marker.position.y = float(parts[1])
                    interactive_marker.position.z = float(parts[2])
                    interactive_marker.orientation.x = float(parts[3])
                    interactive_marker.orientation.y = float(parts[4])
                    interactive_marker.orientation.z = float(parts[5])
                    interactive_marker.orientation.w = float(parts[6])
        except (IOError, ValueError) as e:
            rospy.logerr("Could not read camera startup position from file: {}. Using default. Error: {}".format(file_path, e))
    else:
        rospy.loginfo("Camera startup file not found. Using default position.")

def main():
    rospy.init_node('marker_array_visualizer', anonymous=True)
    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    server = InteractiveMarkerServer("interactive_marker_server")
    rate = rospy.Rate(1)  # 1 Hz

    file_path = "/home/caleb/robochem_steps/octo_action.txt"
    save_file_path = "/home/caleb/robochem_steps/fixed_octo_action.txt"
    camera_location_path ="/home/caleb/robochem_steps/example_transfer.txtcamera_location.txt"

    recent_points = []

    # Create the interactive marker for the camera axis
    interactive_marker = create_interactive_marker(server)
    interactive_pose = server.get("camera_axis").pose
    set_camera_startup_position(camera_location_path, interactive_pose)

    while not rospy.is_shutdown():
        data = read_xyz_and_orientation_from_file(file_path)
        if data is not None:
            x, y, z, roll, pitch, yaw, gripper = data
            recent_points.append((x, y, z))
            if len(recent_points) > 10:  # Keep only the last 10 samples
                recent_points.pop(0)

            # Get the pose of the interactive marker
            interactive_pose = server.get("camera_axis").pose
            save_camera_location(camera_location_path, interactive_pose.position, interactive_pose.orientation)

            # Convert Euler angles to quaternion
            quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

            # Keep orientation in the robot base frame
            robot_base_quaternion = transform_quaternion_to_robot_base(quaternion)

            # Create a MarkerArray
            marker_array = MarkerArray()

            # Visualize the recent points
            marker = Marker()
            marker.header.frame_id = "fr3_link0"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "marker_array_visualizer"
            marker.id = 1
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.lifetime = rospy.Duration(0)

            for i, point in enumerate(recent_points):
                transformed_point = transform_point(point, interactive_pose)
                p = Point(x=transformed_point[0], y=transformed_point[1], z=transformed_point[2])
                marker.points.append(p)

                r, g, b, a = compute_gradient_color(i, len(recent_points))
                c = ColorRGBA(r=r, g=g, b=b, a=a)
                marker.colors.append(c)

            marker_array.markers.append(marker)

            # Add orientation axis markers for the final point
            if recent_points:
                final_position = transform_point(recent_points[-1], interactive_pose)
                orientation_markers = create_orientation_axis_marker(final_position, robot_base_quaternion, marker_id=10)
                marker_array.markers.extend(orientation_markers)

            # Publish the MarkerArray
            pub.publish(marker_array)
            write_fixed_output_file(save_file_path, transformed_point, roll, pitch, yaw, gripper)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point, Pose
# from std_msgs.msg import ColorRGBA
# from interactive_markers.interactive_marker_server import InteractiveMarkerServer
# from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
# import tf.transformations as tf

# def read_xyz_and_orientation_from_file(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             line = f.readline().strip()
#             if not line:
#                 return None
#             parts = line.split(',')
#             if len(parts) < 6:
#                 rospy.logwarn("Not enough values in line to parse x, y, z, and orientation (roll, pitch, yaw)")
#                 return None
#             try:
#                 # Position
#                 x = float(parts[0])
#                 y = float(parts[1])
#                 z = float(parts[2])
#                 # Orientation as Euler angles (roll, pitch, yaw)
#                 roll = float(parts[3])
#                 pitch = float(parts[4])
#                 yaw = float(parts[5])
#                 return (x, y, z, roll, pitch, yaw)
#             except ValueError:
#                 rospy.logwarn("Could not parse floats from line: {}".format(line))
#                 return None
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#         return None

# def compute_gradient_color(index, total):
#     if total <= 1:
#         return (0.0, 0.5, 1.0, 1.0)
#     ratio = float(index) / (total - 1)
#     r = ratio
#     g = 0.5 * (1 - ratio)
#     b = 1.0 - ratio
#     a = 1.0
#     return (r, g, b, a)

# def transform_point(point, pose):
#     translation_matrix = tf.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
#     rotation_matrix = tf.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
#     transform = tf.concatenate_matrices(translation_matrix, rotation_matrix)

#     point_homogeneous = [point[0], point[1], point[2], 1.0]
#     transformed_point_homogeneous = transform.dot(point_homogeneous)
#     return transformed_point_homogeneous[:3]

# def create_orientation_axis_marker(position, quaternion, marker_id):
#     """
#     Create a set of markers to represent the orientation of a point with an XYZ axis.
#     """
#     markers = []

#     # Define the scale for the arrows
#     arrow_scale = {
#         "shaft_diameter": 0.02,
#         "head_diameter": 0.05,
#         "head_length": 0.05
#     }

#     # Rotation matrix from quaternion
#     rotation_matrix = tf.quaternion_matrix(quaternion)

#     # Define the directions of the X, Y, Z axes in local frame
#     axes = {
#         "x": [1, 0, 0],
#         "y": [0, 1, 0],
#         "z": [0, 0, 1]
#     }
#     colors = {
#         "x": (1.0, 0.0, 0.0, 1.0),  # Red
#         "y": (0.0, 1.0, 0.0, 1.0),  # Green
#         "z": (0.0, 0.0, 1.0, 1.0)   # Blue
#     }

#     for axis_name, direction in axes.items():
#         # Transform the local axis direction to world frame
#         transformed_direction = rotation_matrix[:3, :3].dot(direction)

#         # Create the arrow marker
#         marker = Marker()
#         marker.header.frame_id = "fr3_link0"
#         marker.header.stamp = rospy.Time.now()
#         marker.ns = "orientation_axis"
#         marker.id = marker_id
#         marker.type = Marker.ARROW
#         marker.action = Marker.ADD
#         marker.scale.x = arrow_scale["shaft_diameter"]
#         marker.scale.y = arrow_scale["head_diameter"]
#         marker.scale.z = arrow_scale["head_length"]
#         marker.color.r, marker.color.g, marker.color.b, marker.color.a = colors[axis_name]

#         # Define the arrow's start and end points
#         start_point = Point(x=position[0], y=position[1], z=position[2])
#         end_point = Point(
#             x=position[0] + transformed_direction[0] * 0.2,  # Scale the arrow length
#             y=position[1] + transformed_direction[1] * 0.2,
#             z=position[2] + transformed_direction[2] * 0.2
#         )
#         marker.points = [start_point, end_point]

#         markers.append(marker)
#         marker_id += 1  # Increment ID for each marker

#     return markers

# def main():
#     rospy.init_node('marker_array_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     server = InteractiveMarkerServer("interactive_marker_server")
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"
#     recent_points = []

#     while not rospy.is_shutdown():
#         data = read_xyz_and_orientation_from_file(file_path)
#         if data is not None:
#             x, y, z, roll, pitch, yaw = data
#             recent_points.append((x, y, z))
#             if len(recent_points) > 10:  # Keep only the last 10 samples
#                 recent_points.pop(0)

#             # Convert Euler angles to quaternion
#             quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

#             # Create a MarkerArray
#             marker_array = MarkerArray()

#             # Visualize the recent points
#             marker = Marker()
#             marker.header.frame_id = "fr3_link0"
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "marker_array_visualizer"
#             marker.id = 1
#             marker.type = Marker.SPHERE_LIST
#             marker.action = Marker.ADD

#             marker.scale.x = 0.05
#             marker.scale.y = 0.05
#             marker.scale.z = 0.05
#             marker.lifetime = rospy.Duration(0)

#             for i, point in enumerate(recent_points):
#                 p = Point(x=point[0], y=point[1], z=point[2])
#                 marker.points.append(p)

#                 r, g, b, a = compute_gradient_color(i, len(recent_points))
#                 c = ColorRGBA(r=r, g=g, b=b, a=a)
#                 marker.colors.append(c)

#             marker_array.markers.append(marker)

#             # Add orientation axis markers for the final point
#             if recent_points:
#                 final_position = recent_points[-1]
#                 orientation_markers = create_orientation_axis_marker(final_position, quaternion, marker_id=10)
#                 marker_array.markers.extend(orientation_markers)

#             # Publish the MarkerArray
#             pub.publish(marker_array)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

# #!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point, Pose
# from std_msgs.msg import ColorRGBA
# from interactive_markers.interactive_marker_server import InteractiveMarkerServer
# from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
# import tf.transformations as tf

# def read_xyz_and_orientation_from_file(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             line = f.readline().strip()
#             if not line:
#                 return None
#             parts = line.split(',')
#             if len(parts) < 6:
#                 rospy.logwarn("Not enough values in line to parse x, y, z, and orientation (roll, pitch, yaw)")
#                 return None
#             try:
#                 # Position
#                 x = float(parts[0])
#                 y = float(parts[1])
#                 z = float(parts[2])
#                 # Orientation as Euler angles (roll, pitch, yaw)
#                 roll = float(parts[3])
#                 pitch = float(parts[4])
#                 yaw = float(parts[5])
#                 return (x, y, z, roll, pitch, yaw)
#             except ValueError:
#                 rospy.logwarn("Could not parse floats from line: {}".format(line))
#                 return None
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#         return None

# def compute_gradient_color(index, total):
#     if total <= 1:
#         return (0.0, 0.5, 1.0, 1.0)
#     ratio = float(index) / (total - 1)
#     r = ratio
#     g = 0.5 * (1 - ratio)
#     b = 1.0 - ratio
#     a = 1.0
#     return (r, g, b, a)

# def transform_point(point, pose):
#     translation_matrix = tf.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
#     rotation_matrix = tf.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
#     transform = tf.concatenate_matrices(translation_matrix, rotation_matrix)

#     point_homogeneous = [point[0], point[1], point[2], 1.0]
#     transformed_point_homogeneous = transform.dot(point_homogeneous)
#     return transformed_point_homogeneous[:3]

# def create_orientation_axis_marker(position, quaternion, marker_id):
#     """
#     Create a set of markers to represent the orientation of a point with an XYZ axis.
#     """
#     markers = []

#     # Define the scale for the arrows
#     arrow_scale = {
#         "shaft_diameter": 0.02,
#         "head_diameter": 0.05,
#         "head_length": 0.05
#     }

#     # Rotation matrix from quaternion
#     rotation_matrix = tf.quaternion_matrix(quaternion)

#     # Define the directions of the X, Y, Z axes in local frame
#     axes = {
#         "x": [1, 0, 0],
#         "y": [0, 1, 0],
#         "z": [0, 0, 1]
#     }
#     colors = {
#         "x": (1.0, 0.0, 0.0, 1.0),  # Red
#         "y": (0.0, 1.0, 0.0, 1.0),  # Green
#         "z": (0.0, 0.0, 1.0, 1.0)   # Blue
#     }

#     for axis_name, direction in axes.items():
#         # Transform the local axis direction to world frame
#         transformed_direction = rotation_matrix[:3, :3].dot(direction)

#         # Create the arrow marker
#         marker = Marker()
#         marker.header.frame_id = "fr3_link0"
#         marker.header.stamp = rospy.Time.now()
#         marker.ns = "orientation_axis"
#         marker.id = marker_id
#         marker.type = Marker.ARROW
#         marker.action = Marker.ADD
#         marker.scale.x = arrow_scale["shaft_diameter"]
#         marker.scale.y = arrow_scale["head_diameter"]
#         marker.scale.z = arrow_scale["head_length"]
#         marker.color.r, marker.color.g, marker.color.b, marker.color.a = colors[axis_name]

#         # Define the arrow's start and end points
#         start_point = Point(x=position[0], y=position[1], z=position[2])
#         end_point = Point(
#             x=position[0] + transformed_direction[0] * 0.2,  # Scale the arrow length
#             y=position[1] + transformed_direction[1] * 0.2,
#             z=position[2] + transformed_direction[2] * 0.2
#         )
#         marker.points = [start_point, end_point]

#         markers.append(marker)
#         marker_id += 1  # Increment ID for each marker

#     return markers

# def main():
#     rospy.init_node('marker_array_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     server = InteractiveMarkerServer("interactive_marker_server")
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"
#     recent_points = []
#     interactive_marker = create_interactive_marker(server)

#     while not rospy.is_shutdown():
#         data = read_xyz_and_orientation_from_file(file_path)
#         if data is not None:
#             x, y, z, roll, pitch, yaw = data
#             recent_points.append((x, y, z))
#             if len(recent_points) > 10:  # Keep only the last 10 samples
#                 recent_points.pop(0)

#             # Get the pose of the interactive marker
#             interactive_pose = server.get("camera_axis").pose

#             # Convert Euler angles to quaternion
#             quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

#             # Create a MarkerArray
#             marker_array = MarkerArray()

#             # Visualize the recent points
#             marker = Marker()
#             marker.header.frame_id = "fr3_link0"
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "marker_array_visualizer"
#             marker.id = 1
#             marker.type = Marker.SPHERE_LIST
#             marker.action = Marker.ADD

#             marker.scale.x = 0.05
#             marker.scale.y = 0.05
#             marker.scale.z = 0.05
#             marker.lifetime = rospy.Duration(0)

#             for i, point in enumerate(recent_points):
#                 transformed_point = transform_point(point, interactive_pose)
#                 p = Point(x=transformed_point[0], y=transformed_point[1], z=transformed_point[2])
#                 marker.points.append(p)

#                 r, g, b, a = compute_gradient_color(i, len(recent_points))
#                 c = ColorRGBA(r=r, g=g, b=b, a=a)
#                 marker.colors.append(c)

#             marker_array.markers.append(marker)

#             # Add orientation axis markers for the final point
#             if recent_points:
#                 final_position = transform_point(recent_points[-1], interactive_pose)
#                 orientation_markers = create_orientation_axis_marker(final_position, quaternion, marker_id=10)
#                 marker_array.markers.extend(orientation_markers)

#             # Publish the MarkerArray
#             pub.publish(marker_array)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

# #!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point, Pose
# from std_msgs.msg import ColorRGBA
# from interactive_markers.interactive_marker_server import InteractiveMarkerServer
# from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
# import tf.transformations as tf

# def read_xyz_from_file(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             line = f.readline().strip()
#             if not line:
#                 return None
#             parts = line.split(',')
#             if len(parts) < 3:
#                 rospy.logwarn("Not enough values in line to parse x,y,z")
#                 return None
#             try:
#                 x = float(parts[0])
#                 y = float(parts[1])
#                 z = float(parts[2])
#                 return (x, y, z)
#             except ValueError:
#                 rospy.logwarn("Could not parse floats from line: {}".format(line))
#                 return None
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#         return None

# def compute_gradient_color(index, total):
#     if total <= 1:
#         return (0.0, 0.5, 1.0, 1.0)
#     ratio = float(index) / (total - 1)
#     r = ratio
#     g = 0.5 * (1 - ratio)
#     b = 1.0 - ratio
#     a = 1.0
#     return (r, g, b, a)

# def transform_point(point, pose):
#     """
#     Transform a 3D point relative to a given pose (position + orientation).
#     """
#     translation_matrix = tf.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
#     rotation_matrix = tf.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
#     transform = tf.concatenate_matrices(translation_matrix, rotation_matrix)

#     point_homogeneous = [point[0], point[1], point[2], 1.0]
#     transformed_point_homogeneous = transform.dot(point_homogeneous)
#     return transformed_point_homogeneous[:3]

# def create_interactive_marker(server):
#     """
#     Create an interactive marker to represent the camera axis in RViz.
#     """
#     int_marker = InteractiveMarker()
#     int_marker.header.frame_id = "fr3_link0"  # Replace with appropriate frame_id if needed
#     int_marker.name = "camera_axis"
#     int_marker.description = "Camera Axis"
#     int_marker.scale = 0.5

#     # Set the initial position of the marker
#     int_marker.pose.position.x = 0.0
#     int_marker.pose.position.y = 0.0
#     int_marker.pose.position.z = 0.0
#     int_marker.pose.orientation.w = 1.0

#     # Add 6-DOF controls to the interactive marker
#     for axis, orientation in zip(["x", "y", "z"], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]):
#         # Translation control
#         control = InteractiveMarkerControl()
#         control.orientation.w = orientation[3]
#         control.orientation.x = orientation[0]
#         control.orientation.y = orientation[1]
#         control.orientation.z = orientation[2]
#         control.name = f"move_{axis}"
#         control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
#         int_marker.controls.append(control)

#         # Rotation control
#         control = InteractiveMarkerControl()
#         control.orientation.w = orientation[3]
#         control.orientation.x = orientation[0]
#         control.orientation.y = orientation[1]
#         control.orientation.z = orientation[2]
#         control.name = f"rotate_{axis}"
#         control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
#         int_marker.controls.append(control)

#     server.insert(int_marker)
#     server.applyChanges()
#     return int_marker

# def main():
#     rospy.init_node('marker_array_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     server = InteractiveMarkerServer("interactive_marker_server")
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"
#     recent_points = []
#     interactive_marker = create_interactive_marker(server)

#     while not rospy.is_shutdown():
#         xyz = read_xyz_from_file(file_path)
#         if xyz is not None:
#             x, y, z = xyz
#             recent_points.append((x, y, z))
#             if len(recent_points) > 10:  # Keep only the last 10 samples
#                 recent_points.pop(0)

#             # Get the pose of the interactive marker
#             interactive_pose = server.get("camera_axis").pose

#             # Create a MarkerArray
#             marker_array = MarkerArray()
#             marker = Marker()
#             marker.header.frame_id = "fr3_link0"
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "marker_array_visualizer"
#             marker.id = 1
#             marker.type = Marker.SPHERE_LIST
#             marker.action = Marker.ADD

#             marker.scale.x = 0.05
#             marker.scale.y = 0.05
#             marker.scale.z = 0.05
#             marker.lifetime = rospy.Duration(0)

#             for i, point in enumerate(recent_points):
#                 transformed_point = transform_point(point, interactive_pose)
#                 p = Point(x=transformed_point[0], y=transformed_point[1], z=transformed_point[2])
#                 marker.points.append(p)

#                 r, g, b, a = compute_gradient_color(i, len(recent_points))
#                 c = ColorRGBA(r=r, g=g, b=b, a=a)
#                 marker.colors.append(c)

#             marker_array.markers.append(marker)
#             pub.publish(marker_array)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

# #!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point, Pose
# from std_msgs.msg import ColorRGBA
# from interactive_markers.interactive_marker_server import InteractiveMarkerServer
# from interactive_markers.menu_handler import MenuHandler
# from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
# import tf.transformations as tf

# def read_xyz_from_file(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             line = f.readline().strip()
#             if not line:
#                 return None
#             parts = line.split(',')
#             if len(parts) < 3:
#                 rospy.logwarn("Not enough values in line to parse x,y,z")
#                 return None
#             try:
#                 x = float(parts[0])
#                 y = float(parts[1])
#                 z = float(parts[2])
#                 return (x, y, z)
#             except ValueError:
#                 rospy.logwarn("Could not parse floats from line: {}".format(line))
#                 return None
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#         return None

# def compute_gradient_color(index, total):
#     if total <= 1:
#         return (0.0, 0.5, 1.0, 1.0)
#     ratio = float(index) / (total - 1)
#     r = ratio
#     g = 0.5 * (1 - ratio)
#     b = 1.0 - ratio
#     a = 1.0
#     return (r, g, b, a)

# def transform_point(point, pose):
#     """
#     Transform a 3D point relative to a given pose (position + orientation).
#     """
#     # Create a transformation matrix from the pose
#     trans = tf.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
#     rot = tf.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
#     transform = tf.concatenate_matrices(trans, rot)

#     # Convert the point to homogeneous coordinates
#     point_homogeneous = [point[0], point[1], point[2], 1.0]

#     # Apply the transformation matrix
#     transformed_point = transform.dot(point_homogeneous)

#     # Return the transformed point in Cartesian coordinates
#     return transformed_point[:3]


# def create_interactive_marker(server):
#     """
#     Create an interactive marker to represent the camera axis in RViz.
#     """
#     int_marker = InteractiveMarker()
#     int_marker.header.frame_id = "fr3_link0"
#     int_marker.name = "camera_axis"
#     int_marker.description = "Camera Axis"
#     int_marker.scale = 0.5

#     # Set initial position of the marker
#     int_marker.pose.position.x = 0.0
#     int_marker.pose.position.y = 0.0
#     int_marker.pose.position.z = 0.0
#     int_marker.pose.orientation.w = 1.0

#     # Add a control for moving in XYZ
#     control = InteractiveMarkerControl()
#     control.name = "move_xyz"
#     control.interaction_mode = InteractiveMarkerControl.MOVE_3D
#     int_marker.controls.append(control)

#     # Add a control for rotating
#     control = InteractiveMarkerControl()
#     control.name = "rotate_xyz"
#     control.interaction_mode = InteractiveMarkerControl.ROTATE_3D
#     int_marker.controls.append(control)

#     # Add the marker to the server
#     server.insert(int_marker)
#     server.applyChanges()

#     return int_marker

# def main():
#     rospy.init_node('marker_array_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     server = InteractiveMarkerServer("interactive_marker_server")
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"

#     # Keep a list of the most recent 10 points
#     recent_points = []

#     # Create the interactive marker for the camera axis
#     interactive_marker = create_interactive_marker(server)

#     while not rospy.is_shutdown():
#         xyz = read_xyz_from_file(file_path)
#         if xyz is not None:
#             x, y, z = xyz
#             recent_points.append((x, y, z))
#             if len(recent_points) > 10:  # Keep only the last 10 samples
#                 recent_points.pop(0)

#             # Get the pose of the interactive marker
#             interactive_pose = server.get("camera_axis").pose

#             # Create a MarkerArray
#             marker_array = MarkerArray()

#             # Create a single Marker
#             marker = Marker()
#             marker.header.frame_id = "world"  # Adjust if needed
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "marker_array_visualizer"
#             marker.id = 1
#             marker.type = Marker.SPHERE_LIST
#             marker.action = Marker.ADD
            
#             # Set scale for spheres
#             marker.scale.x = 0.05
#             marker.scale.y = 0.05
#             marker.scale.z = 0.05

#             marker.lifetime = rospy.Duration(0)  # 0 is forever

#             # Add the most recent 10 points, transformed relative to the marker pose
#             for i, point in enumerate(recent_points):
#                 transformed_point = transform_point(point, interactive_pose)
#                 p = Point(x=transformed_point[0], y=transformed_point[1], z=transformed_point[2])
#                 marker.points.append(p)

#                 r, g, b, a = compute_gradient_color(i, len(recent_points))
#                 c = ColorRGBA(r=r, g=g, b=b, a=a)
#                 marker.colors.append(c)

#             # Add this marker to the array
#             marker_array.markers.append(marker)

#             # Publish the MarkerArray
#             pub.publish(marker_array)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

# #!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point
# from std_msgs.msg import ColorRGBA

# def read_xyz_from_file(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             line = f.readline().strip()
#             if not line:
#                 return None
#             # Split by comma
#             parts = line.split(',')
#             # We need at least 3 values for x, y, z
#             if len(parts) < 3:
#                 rospy.logwarn("Not enough values in line to parse x,y,z")
#                 return None
#             # Parse first three floats
#             try:
#                 x = float(parts[0])
#                 y = float(parts[1])
#                 z = float(parts[2])
#                 return (x, y, z)
#             except ValueError:
#                 rospy.logwarn("Could not parse floats from line: {}".format(line))
#                 return None
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#         return None

# def compute_gradient_color(index, total):
#     """
#     Compute a color for the point at 'index' given 'total' points.
#     This will produce a gradient from light blue (first point) to red (last point).
#     """
#     if total <= 1:
#         return (0.0, 0.5, 1.0, 1.0)  # Default light blue for single point

#     ratio = float(index) / (total - 1)
#     r = ratio  # Ranges from 0.0 to 1.0
#     g = 0.5 * (1 - ratio)  # Fades from 0.5 to 0.0
#     b = 1.0 - ratio  # Fades from 1.0 to 0.0
#     a = 1.0
#     return (r, g, b, a)

# def main():
#     rospy.init_node('marker_array_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"

#     # Keep a list of the most recent 10 points
#     recent_points = []

#     while not rospy.is_shutdown():
#         xyz = read_xyz_from_file(file_path)
#         if xyz is not None:
#             x, y, z = xyz

#             # Add the new point to the list
#             recent_points.append((x, y, z))
#             if len(recent_points) > 10:  # Keep only the last 10 samples
#                 recent_points.pop(0)

#             # Create a MarkerArray
#             marker_array = MarkerArray()

#             # Create a single Marker
#             marker = Marker()
#             marker.header.frame_id = "fr3_link0"  # adjust if needed
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "marker_array_visualizer"
#             marker.id = 1
#             marker.type = Marker.SPHERE_LIST
#             marker.action = Marker.ADD
            
#             # Set scale for spheres
#             marker.scale.x = 0.05
#             marker.scale.y = 0.05
#             marker.scale.z = 0.05

#             marker.lifetime = rospy.Duration(0)  # 0 is forever

#             # Add the most recent 10 points
#             for i, (px, py, pz) in enumerate(recent_points):
#                 p = Point(x=px, y=py, z=pz)
#                 marker.points.append(p)

#                 r, g, b, a = compute_gradient_color(i, len(recent_points))
#                 c = ColorRGBA(r=r, g=g, b=b, a=a)
#                 marker.colors.append(c)

#             # Add this marker to the array
#             marker_array.markers.append(marker)

#             # Publish the MarkerArray
#             pub.publish(marker_array)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


# #!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point
# from std_msgs.msg import ColorRGBA

# def read_xyz_from_file(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             line = f.readline().strip()
#             if not line:
#                 return None
#             # Split by comma
#             parts = line.split(',')
#             # We need at least 3 values for x, y, z
#             if len(parts) < 3:
#                 rospy.logwarn("Not enough values in line to parse x,y,z")
#                 return None
#             # Parse first three floats
#             try:
#                 x = float(parts[0])
#                 y = float(parts[1])
#                 z = float(parts[2])
#                 return (x, y, z)
#             except ValueError:
#                 rospy.logwarn("Could not parse floats from line: {}".format(line))
#                 return None
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#         return None

# def compute_gradient_color(index, total):
#     """
#     Compute a color for the point at 'index' given 'total' points.
#     This will produce a gradient starting from red at the first point
#     to green at the last point.

#     For example:
#     - index=0 (first point) -> Red (1.0, 0.0, 0.0)
#     - index=total-1 (last point) -> Green (0.0, 1.0, 0.0)
#     Intermediate points linearly interpolate.
#     """
#     if total <= 1:
#         # Only one point, just red
#         return (1.0, 0.0, 0.0, 1.0)

#     ratio = float(index) / (total - 1)
#     r = 1.0 - ratio
#     g = ratio
#     b = 0.0
#     a = 1.0
#     return (r, g, b, a)

# def main():
#     rospy.init_node('marker_array_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"

#     # Keep a list of all points that have been read so far
#     all_points = []

#     while not rospy.is_shutdown():
#         xyz = read_xyz_from_file(file_path)
#         if xyz is not None:
#             x, y, z = xyz
#             all_points.append((x, y, z))

#             # Create a MarkerArray
#             marker_array = MarkerArray()

#             # Create a single Marker
#             marker = Marker()
#             marker.header.frame_id = "fr3_link0"  # adjust if needed
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "marker_array_visualizer"
#             marker.id = 1
#             marker.type = Marker.SPHERE_LIST
#             marker.action = Marker.ADD
            
#             # Set scale for spheres
#             marker.scale.x = 0.05
#             marker.scale.y = 0.05
#             marker.scale.z = 0.05

#             marker.lifetime = rospy.Duration(0)  # 0 is forever

#             # Add all points accumulated so far
#             for i, (px, py, pz) in enumerate(all_points):
#                 p = Point(x=px, y=py, z=pz)
#                 marker.points.append(p)

#                 r, g, b, a = compute_gradient_color(i, len(all_points))
#                 c = ColorRGBA(r=r, g=g, b=b, a=a)
#                 marker.colors.append(c)

#             # Add this marker to the array
#             marker_array.markers.append(marker)

#             # Publish the MarkerArray
#             pub.publish(marker_array)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

# #!/usr/bin/env python

# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point
# from std_msgs.msg import ColorRGBA

# def read_xyz_from_file(file_path):
#     try:
#         with open(file_path, 'r') as f:
#             line = f.readline().strip()
#             if not line:
#                 return None
#             # Split by comma
#             parts = line.split(',')
#             # We need at least 3 values for x, y, z
#             if len(parts) < 3:
#                 rospy.logwarn("Not enough values in line to parse x,y,z")
#                 return None
#             # Parse first three floats
#             try:
#                 x = float(parts[0])
#                 y = float(parts[1])
#                 z = float(parts[2])
#                 return (x, y, z)
#             except ValueError:
#                 rospy.logwarn("Could not parse floats from line: {}".format(line))
#                 return None
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#         return None

# def main():
#     rospy.init_node('marker_array_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"

#     while not rospy.is_shutdown():
#         xyz = read_xyz_from_file(file_path)
#         print("Read x={}, y={}, z={}".format(*xyz))
        
#         if xyz is not None:
#             x, y, z = xyz

#             # Create a MarkerArray
#             marker_array = MarkerArray()

#             # Create a single Marker
#             marker = Marker()
#             marker.header.frame_id = "fr3_link0"  # adjust if needed
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "marker_array_visualizer"
#             marker.id = 1
#             marker.type = Marker.POINTS
#             marker.action = Marker.ADD
            
#             # Set scale for POINTS (x & y scale represent diameter)
#             marker.scale.x = 0.05
#             marker.scale.y = 0.05

#             marker.lifetime = rospy.Duration(0)  # 0 is forever

#             # Add the single point
#             p = Point()
#             p.x = x
#             p.y = y
#             p.z = z
#             marker.points.append(p)

#             # Assign a color
#             c = ColorRGBA()
#             c.r = 1.0
#             c.g = 1.0
#             c.b = 0.0
#             c.a = 1.0
#             marker.colors.append(c)

#             # Add this marker to the array
#             marker_array.markers.append(marker)

#             # Publish the MarkerArray
#             pub.publish(marker_array)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass



# #!/usr/bin/env python

# import rospy
# import time
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point

# def read_points_from_file(file_path):
#     points = []
#     try:
#         with open(file_path, 'r') as f:
#             for line in f:
#                 line = line.strip()
#                 if not line:
#                     continue
#                 parts = line.split()
#                 if len(parts) < 3:
#                     continue
#                 try:
#                     x = float(parts[0])
#                     y = float(parts[1])
#                     z = float(parts[2])
#                     points.append((x, y, z))
#                     print(x, y, z)
#                 except ValueError:
#                     rospy.logwarn("Skipping a line due to parsing error: {}".format(line))
#     except IOError:
#         rospy.logerr("Could not open file: {}".format(file_path))
#     return points

# def color_gradient(i, n):
#     """
#     Compute a color gradient for the i-th point out of n total.
#     For simplicity, let's vary red and green, keeping blue constant.
#     This will create a gradient from red to green:
#     - i=0: red = 1.0, green = 0.0
#     - i=n-1: red = 0.0, green = 1.0
#     """
#     if n <= 1:
#         # If there's only one point, just pick something
#         return (1.0, 0.0, 0.0, 1.0)
#     ratio = float(i) / (n - 1)
#     r = 1.0 - ratio
#     g = ratio
#     b = 0.5  # constant blue component
#     a = 1.0
#     return (r, g, b, a)

# def main():
#     rospy.init_node('point_visualizer', anonymous=True)
#     pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
#     rate = rospy.Rate(1)  # 1 Hz

#     file_path = "/home/caleb/robochem_steps/octo_action.txt"

#     while not rospy.is_shutdown():
#         points = read_points_from_file(file_path)
#         print("Read {} points".format(len(points)))
        
#         # Prepare the Marker
#         marker = Marker()
#         marker.header.frame_id = "map"  # or "world", depending on your TF setup
#         marker.header.stamp = rospy.Time.now()
#         marker.ns = "point_visualizer"
#         marker.id = 0
#         marker.type = Marker.POINTS
#         marker.action = Marker.ADD
        
#         # Set scale of the points (for POINTS, x and y scale are diameter)
#         marker.scale.x = 0.05
#         marker.scale.y = 0.05
        
#         marker.lifetime = rospy.Duration(0)  # 0 is forever
#         marker.frame_locked = False
        
#         # We'll store points and their corresponding colors
#         for i, (x, y, z) in enumerate(points):
#             p = Point()
#             p.x = x
#             p.y = y
#             p.z = z
#             marker.points.append(p)
            
#             r, g, b, a = color_gradient(i, len(points))
#             # Use per-point color by filling marker.colors correspondingly
#             from std_msgs.msg import ColorRGBA
#             c = ColorRGBA()
#             c.r = r
#             c.g = g
#             c.b = b
#             c.a = a
#             marker.colors.append(c)
        
#         # Publish the marker
#         pub.publish(marker)
        
#         # Sleep until next cycle
#         rate.sleep()


# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

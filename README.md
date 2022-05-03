# ROS-Project
A simple project that controls the Platypous robot.

The project's goal is to make the robot stay a certain distance from the wall.

## Classes/Enums:
### Direction (Enum):
Holds the main directions the robot scans for. The values correspond to the related degrees in the LIDAR's array.

### Turning (Enum):
Indicates the direction the robot is turning.

### Orientation (Class):
Used to store Euler rotations.

### platypous_controller (Class):
The main class for controlling the robot.

## Methods:

### \_\_init__:
The default constructor for the robot controller.
Subscribes to the related topics, and initializes the Twist type publisher.

### publisher:
Initializes the Twist type publisher.

### listener
Subscribes to the related topics.

### wheelTwistOdometry(msg)
Parameters:
- msg: the callback message

Callback function for wheel odometry.


### laserScan(msg)
Parameters:
- msg: the callback message

Callback function for LIDAR.

### get_wall_angle(starting_angle)
Parameters:
- starting_angle: Start of the scan, takes in the degree in the LIDAR format.

Returns the angle of the wall

### move(speed_m_per_s)
Parameters:
- speed_m_per_s: the speed of the robot

The main method that moves the robot

### get_closeset(Direction)
Parameters:
- Direction: Direction of the scan

Returns the closest wall's distance in a given range

### get_distance(degree, real_degrees=True)
Parameters:
- degree: The direction of the scan
- real_degrees: Default value is true. If set to true the function automatically converts to LIDAR degree format

Returns the distance on the wall in a given direction.

### detect_collision(Direction)
Parameters:
- Direction: direction of the scan

Returns true if there is a wall in the way of the scan.
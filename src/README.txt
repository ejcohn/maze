Nate Dimick and Eli Cohn
COSI 119a
Maze Runner

To run:
- Ensure roscore is running in a terminal window
- run: rosrun maze maze_simple.py

The purpose of this assignment was to design code such that when a robot was placed in a maze, it would be able to
escape the maze. Our algorithm was based off of the wall following algorithm and left hand rule -- meaning that the
robot would find a wall and the proceed to keep that wall on its left side. This would eventually lead the robot to
escape the maze.

In terms of implementation, the robot has five different states: no wall, cornered, following wall, just lost
left wall, and looking to pick up left wall. The purpose of these states is as follows:
- No wall: When there is no wall (to the left of the robot), the robot will go straight until it finds a wall, entering
the cornered state.
- Cornered: When the robot has both a wall ahead and a wall to its left, the robot will turn right 90 degrees.
- Following wall: When there is a wall to the robot's left, the robot will continue to move forward. The robot exits
this state if it becomes cornered, there is no wall, or until the wall it is following ends.
- Just lost left wall: If the robot was following the wall and the wall to its left ends, the robot will turn left 90
degrees, move forward and proceed to find follow the wall (aka the state of looking to pick up left wall).
- Looking to pick up left wall: This state is entered after the left wall was lost. If after 2.5 seconds of proceeding
straight the wall the robot was trying to follow was not found, the state is reset to "no wall" and the robot will
start the process from the beginning.

The most amount of trouble we had from this assignment was getting odometry to work. While the subscriber was working
(we even created a node -- angle_node.py included in this src file!), we had trouble getting the robot to turn the
specified amount. We were able to create some work-arounds and got it mostly working in the end, but it was not without
a significant amount of tinkering. The way we ended up making it work was by simplifying the algorithm (in maze_simple)
and tweaking it.

Files included in the project:
- maze_simply.py: Our final product, as described above.
- angle_node.py: The publisher that converts quaternions to radians. This is not used in our end result.
- key_publisher.py, odom_test.py, turn_test.py, simple_teleop.py: testing files to test different features
of turns, odometry, etc.
- maze.py, maze2.py: Previous iterations of the maze.

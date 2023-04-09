# A* Algorithm adapted for a Turtlebot3

## Description

The A* algorithm used in the code is a pathfinding algorithm that aims the shortest path between two points. The algorithm is adapted for a Turtlebot robot in this implementation, which takes into account the orientation and RPMs (rotations per minute) of the robot's wheels.

The A* algorithm operates by keeping two sets of nodes, one open and one closed. The open set includes nodes that have yet to be evaluated, whereas the closed set includes nodes that have already been evaluated. The algorithm begins by inserting the first node into the open set. Then, it selects the node from the open set with the lowest total cost (the sum of the path cost from the start and the heuristic cost), explores its neighbors, and updates the costs. If the goal is met, the path is rebuilt from the parent nodes.

The algorithm in this implementation generates successors (neighbors) for each node by simulating the Turtlebot's movements for various RPM values of its wheels. The total cost is calculated as the sum of the action cost and the heuristic cost (based on the RPM values) (Euclidean distance to the goal node).

Both the part 1 and part 2 python scripts have been included in the src folder of the package `tb_astar`.
 
The script consists of several functions, including:

1. `Astar(screen, start, end, start_orientation, end_orientation, MAP, rpm1 = 50, rpm2 = 100)`: Astar algorithm implementation.
2. `draw_shapes(clearance)`: Draws obstacles (shapes) on the Pygame screen.
3. `create_map(screen, clearance)`: Creates and saves a map of the environment with a clearance around the obstacles.
4. `select_box(screen, pos, pos_angle)`: Function used to display the start and end node.
5. `draw_points(point_list)`: Draws the points of the calculated path on the screen.
6. `save_path(path, orient, filename='path.txt')`: Saves the calculated path to a text file.
7. `parse_arguments()`: Parses command-line arguments for RPM values.
8. `main()`: Main function that runs the whole script.


## Dependencies:

- `Pygame` is a Python library for creating video games and multimedia applications.
- `NumPy` is a Python library for numerical computing.
- `math`: A Python standard library for mathematical operations.
- `time`: A standard Python library for manipulating time.
- `tkinter`: A Python standard library for creating graphical user interfaces (GUIs).

## Project Members

- Tarun Trilokesh (UID: 118450766)
- Harshal Shirsath (UID: 119247419)

## Installation 

- To install the dependencies, you can use pip, the Python package installer. Run the following command:
    ```
    pip install pygame numpy
    ```
-  tkinter and the other standard libraries come pre-installed with Python, so you don't need to install them separately.

## Execution

- Once in the catkin workspace, navigate to scr folder in the package `tb_astar`, use `cd src/tb_astar/src`.
- Run the script from the command line with optional arguments for RPM values:
    ```
    python3 astar_explore.py --RPM1 25 --RPM2 50
    ```
    or
    ```
    python3 astar_explore.py --RPM1 50 --RPM2 100
    ```
- This commands will run the script with RPM1 set to 50 and RPM2 set to 100. If you don't provide these arguments, the script will use the default values (50 for RPM1 and 100 for RPM2).
- A Pygame window will appear, displaying the environment with obstacles. Click on the screen to select a starting point and an ending point for the Turtlebot.
- The astar algorithm uses the obstacle map created and saved in `map.txt` to save time. The `map.txt` is included in the package, if deleted the algorithm creates new file which will take approximately 20 seconds wait for the pygame window to load and display white border around the obstacles.
- The A* algorithm will calculate the optimal path between the start and end point (set to (0,0) and (0,5)), avoiding the obstacles. The path will be displayed on the Pygame screen.
- The calculated path will be saved to a text file called `path.txt` in the src folder of the package.
- To exit the program, close the Pygame window.
- Once exited the pygame window in a new terminal window navigate to catkin workspace and run the command `roslaunch tb_astar tb_astar.launch`.
- Then navigate to src directory in the package `tb_astar` and run the command `python3 turtlebot_controller.py`.
- The turtlebot will navigate the obstacles and reach the goal node.


## Miscellaneous

- The code has been pushed to the github account of all the team members, github links are below:
  - git clone https://github.com/tarunreddyy/Planning-for-Autonomous-Robots.git
  - git clone https://github.com/HarshShirsath/Turtlebot3-in-Gazebo-using-A-algorithm.git
  - Navigate to the "A_star_algorithm_adapted_for_turtlebot3" or "" directory and follow the instructions.
- Tutorial video has been uploaded to the google drive find the link below:
  - `https://drive.google.com/file/d/1mR8Lo2B31e-gOIFcxk7lCrU8SMId14nl/view?usp=sharing`
# MLND Capstone Project: Plot and Navigate a VirtualMaze

## Files

* `robot.py`: This script establishes the robot class. This contains the main implementation done for this project.
* `maze.py`: This script contains functions for constructing the maze and for checking for walls upon robot movement or sensing. It has been provided with the starter code.
* `tester.py`: This script will be run to test the robot’s ability to navigate mazes. It has been provided with the starter code
* `stats.py`: A modified version of `tester.py` that only outputs the runtimes and the score on one line
* `showmaze.py`: This script can be used to create a visual demonstration of what a maze looks like. It has been provided with the starter code
* `test_maze_0{1,2,3}.txt`: These files provide three sample mazes upon which to test the robot. They have been provided with the starter code
test_maze_04.txt: The 12x12 maze which is designed by myself for testing the robustness of my robot.
* `Report.pdf`: The final report describing my work on this capstone project

## Commands

To send the robot through the maze:

`python tester.py <maze_file>  <strategy>`

`<strategy>` can be `random` or `astar`

To do the same, but receive the output on a single line:

`python stats.py <maze_file>  <strategy>`

You can run `stats.py` in a loop and send the output to a file for later analysis:

`$ for i in {1..1000}; do python stats.py test_maze_01.txt random; done > random_01.csv`

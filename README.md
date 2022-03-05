[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Verified on Ubuntu 20.04 and python 3.8.10, packages used are heapq, cv2 and NumPy.

# dijkstra-algorithm
Implementing Dijkstra's Algorithm to find the shortest path for a point robot from starting position to goal position in a given map.

## Assumptions
The robot is a point robot with a radius of 5mm as clearance <br>

The robot action space is an 8 connected space, that means now you can move the robot in up, down, left, right & diagonally between up-left, up-right, down-left and down-right directions. <br>

![Action Space of Robot](/res/action.png "Action Space of Robot")
Action Sets = {(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)}<br>

![Map](/res/map.png "Map")

## To install the dependencies
```
sudo pip install numpy
sudo pip install opencv-python
```

## Steps to run
To clone the file:
```
git clone https://github.com/Rishabh96M/dijkstra-algorithm.git
cd dijkstra-algorithm
```

To run the code:
```
python dijkstra-algorithm.py
```
This code will ask the user for starting point and goal point on the map. After the path is found, It will animate all searched nodes and display the optimal path found using dijkstra-algorithm <br>

This *videos* folder contains example videos on how the program works. 

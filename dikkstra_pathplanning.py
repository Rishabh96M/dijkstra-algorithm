# dijkstra path planning
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: Using dijkstra's algorith to find the optimum path from staring
# to goal poisiton

import heapq as heap
import matplotlib.pyplot as plt
import numpy as np


def listOfValidPoints(map_len, map_bre):
    """
    Definition
    ---
    Method to generate a list of all valid points on a map

    Parameters
    ---
    map_len : length of map
    map_bre : breadth of map

    Returns
    ---
    validPoints : list of all the valid points
    """
    validPoints = []

    # Defining Circle
    xc = 300
    yc = 185
    rc = 40

    # Defining Polygon
    x1 = 36
    y1 = 185
    x2 = 115
    y2 = 210
    x3 = 80
    y3 = 180
    x4 = 105
    y4 = 100

    # Defining Hexagon
    x5 = 200
    y5 = 140.4
    x6 = 235
    y6 = 120.2
    x7 = 235
    y7 = 79.8
    x8 = 200
    y8 = 59.6
    x9 = 165
    y9 = 79.8
    x0 = 165
    y0 = 120.2

    m21 = (y2 - y1) / (x2 - x1)
    m32 = (y3 - y2) / (x3 - x2)
    m43 = (y4 - y3) / (x4 - x3)
    m14 = (y1 - y4) / (x1 - x4)

    m65 = (y6 - y5) / (x6 - x5)
    m87 = (y8 - y7) / (x8 - x7)
    m98 = (y9 - y8) / (x9 - x8)
    m50 = (y5 - y0) / (x5 - x0)

    th = np.linspace(0, 2 * 3.14, 720)
    plt.plot(xc + (rc * np.cos(th)), yc + (rc * np.sin(th)), 'b-')

    plt.plot((x1, x2), (y1, y2), 'b-')
    plt.plot((x2, x3), (y2, y3), 'b-')
    plt.plot((x3, x4), (y3, y4), 'b-')
    plt.plot((x4, x1), (y4, y1), 'b-')

    plt.plot((x5, x6), (y5, y6), 'b-')
    plt.plot((x6, x7), (y6, y7), 'b-')
    plt.plot((x7, x8), (y7, y8), 'b-')
    plt.plot((x8, x9), (y8, y9), 'b-')
    plt.plot((x9, x0), (y9, y0), 'b-')
    plt.plot((x0, x5), (y0, y5), 'b-')

    for x in range(0, map_len + 1):
        for y in range(0, map_bre + 1):
            if ((x - xc)**2 + (y - yc)**2) <= rc**2:
                continue
            if (y-y1) <= (m21*(x-x1)) and (y-y2) >= (m32*(x-x2)) and \
               (y-y4) >= (m14*(x-x4)):
                continue
            if (y-y1) <= (m21*(x-x1)) and (y-y3) <= (m43*(x-x3)) and \
               (y-y4) >= (m14*(x-x4)):
                continue
            if x <= x6 and x >= x9 and y >= y9 and y <= y6:
                continue
            if y >= y6 and (y-y5) <= (m65*(x-x5)) and (y-y0) <= (m50*(x-x0)):
                continue
            if y <= y9 and (y-y7) >= (m87*(x-x7)) and (y-y8) >= (m98*(x-x8)):
                continue
            validPoints.append((x, y))
    return validPoints


def getAdjNodes(curr_node, validPoints):
    """
    Definition
    ---
    Method to generate all adjacent nodes for a given node

    Parameters
    ---
    curr_node : node of intrest
    validPoints : list of all valid points

    Returns
    ---
    adjNodes : list of adjacent nodes with cost from parent node
    """
    adjNodes = []
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4),
             (1, -1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4)]
    for move in moves:
        if (curr_node[0] + move[0], curr_node[1] + move[1]) in validPoints:
            adjNodes.append(((curr_node[0] + move[0], curr_node[1] + move[1]),
                            move[2]))
    return adjNodes


def updateNode(new_node, curr_node, node_cost, queue, parent_map, cost, goal):
    """
    Definition
    ---
    Method to update nodes based on cost and closed list of nodes

    Parameters
    ---
    new_node : node of intrest
    curr_node : parent node
    node_cost : dict of all nodes mapped to costs
    queue : priority queue of nodes to check
    parent_map : dict of nodes mapped to parent node_cost
    cost : cost to get to new node from parent node
    goal : goal node

    Returns
    ---
    Reached : if new_node is goal node returns True othervise returns False
    node_cost : dict of all nodes mapped to costs
    queue : priority queue of nodes to check
    parent_map : dict of nodes mapped to parent node_cost
    """
    new_cost = node_cost[curr_node] + cost
    temp_cost = node_cost.get(new_node)
    if not temp_cost or (temp_cost > new_cost):
        node_cost[new_node] = new_cost
        parent_map[new_node] = curr_node
        heap.heappush(queue, (new_cost, new_node))
    if new_node == goal:
        return True, node_cost, queue, parent_map
    return False, node_cost, queue, parent_map


def dijkstra_path(start, goal, validPoints):
    """
    Definition
    ---
    Method to get least cost path from starting to goal node using dijkstra's

    Parameters
    ---
    start : starting node
    goal : goal node
    validPoints : list of all valid points

    Returns
    ---
    Reached : if path is found True othervise False
    parent_map : dict of nodes mapped to parent node_cost
    node_cost : dict of all nodes mapped to costs
    """
    closed = []
    queue = []
    node_cost = {}
    parent_map = {}
    reached = False

    node_cost[start] = 0
    heap.heappush(queue, (0, start))

    if goal == start:
        reached = True
        parent_map[goal] = start

    while not reached and queue:
        curr_cost, curr_node = heap.heappop(queue)
        closed.append(curr_node)
        adjNodes = getAdjNodes(curr_node, validPoints)
        for new_node, cost in adjNodes:
            if new_node in closed:
                continue
            print('checking for node: ', new_node)
            flag, node_cost, queue, parent_map = updateNode(
                new_node, curr_node, node_cost, queue, parent_map, cost, goal)
            if flag:
                reached = True
                break
    return reached, parent_map, node_cost, closed


def getPath(parent_map, start, goal):
    """
    Definition
    ---
    Method to generate path using backtracking

    Parameters
    ---
    parent_map : dict of nodes mapped to parent node_cost
    start : starting node
    goal : goal node

    Returns
    ---
    path: list of all the points from starting to goal position
    """
    curr_node = goal
    parent_node = parent_map[goal]
    path = [curr_node]
    while not parent_node == start:
        curr_node = parent_node
        parent_node = parent_map[curr_node]
        path.append(curr_node)
    path.append(start)
    return path[::-1]


if __name__ == '__main__':
    map_len = 400
    map_bre = 250
    flag = False
    print('Validating all points in the map. Please wait...')
    points = listOfValidPoints(map_len, map_bre)

    start = input("Input Staring Position in format: x,y\n")
    start = (int(start.split(',')[0]), int(start.split(',')[1]))
    if start in points:
        goal = input("Input Goal Position in format: x,y\n")
        goal = (int(goal.split(',')[0]), int(goal.split(',')[1]))
        if goal in points:
            print('performing')
            flag, parent_map, node_cost, closed = dijkstra_path(
                start, goal, points)
            if flag:
                print('Path Found')
                path = getPath(parent_map, start, goal)
                print(path)
                plt.plot(goal[0], goal[1], 'ro', label='goal point')
                plt.plot(start[0], start[1], 'go', label='starting point')
                plt.xlim([0, map_len])
                plt.ylim([0, map_bre])
                # for point in closed:
                #     plt.plot(point[0], point[1], 'yo')
                #     plt.pause(0.00001)
                path_x = []
                path_y = []
                for point in path:
                    path_x.append(point[0])
                    path_y.append(point[1])
                plt.plot(path_x, path_y, 'k-', label='path')
                plt.legend()
                plt.show()
            else:
                print('Path not found')
                print(parent_map)
        else:
            print('Not a valid point')
    else:
        print('Not a valid point')

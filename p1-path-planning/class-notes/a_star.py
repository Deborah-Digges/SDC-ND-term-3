# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------
# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

expand = [ [-1 for i in range(len(grid[0]))] for i in range(len(grid)) ]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1
MARKER = 2

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

path = {}
"""
    State list of the form [[cost, f-value, position-x, position-y]]
    Where f-value = cost + heuristic
"""
def findGoal(state_list):
    for state in state_list:
        if( state[2] == goal[0] and state[3] == goal[1]):
            return state

"""
   State list of the form [[cost, f-value, position-x, position-y]]
   Return the node with the lowest f-value 
   Also removes the node from the list
"""
def findLowestFValueNode(state_list):
    lowest_f_node = min(state_list, key=lambda x: x[1])
    state_list.remove(lowest_f_node)
    return lowest_f_node

"""
    Mark a node as visited
"""
def mark(node):
    grid[node[2]][node[3]] = MARKER

def getFValue(node_cost, node_x, node_y):
    return node_cost + heuristic[node_x][node_y]

"""
    Reachable nodes are one more cost unit from the original node and satisfy the following conditions:
    1. They should be valid grid cells
        - no negative indices
        - no indices greater than grid dimensions
    2. They should not be blocked(contain a 1)
    3. They should not have been already visited(contain a 2)
"""
def findReachableNodes(node):
    node_cost = node[0] + cost
    node_x = node[2]
    node_y = node[3]

    return map(lambda x: [x[0], getFValue(x[0], x[1], x[2]) , x[1], x[2]], filter(lambda x: x[1] >= 0 and
                         x[2] >=0 and 
                         x[1] < len(grid) and
                         x[2] < len(grid[0]) and
                         grid[x[1]][x[2]]!=1  and 
                         grid[x[1]][x[2]]!=2,
    [[node_cost, node_x + delta_x, node_y + delta_y] for (delta_x, delta_y) in delta] ))

"""
    node: [cost, f-value, x-pos, y-pos]
    Updates the node with the order in which it was visited
"""
def markExpanded(node, count):
    expand[node[2]][node[3]] = count


"""
    Create a structure where each child node is aware of it's parent
"""
def populatePath(childNodes, parentNode):
    for child in childNodes:
        path[(child[2], child[3])] = (parentNode[2], parentNode[3])

"""
    Compare 2 nodes for equality
"""
def isEqual(node1, node2):
    return node1[0] == node2[0] and node1[1] == node2[1]

"""
    Starting from the final goal
    trace the path which was taken
"""
def tracePath(path, goal):
    node = tuple(goal)
    path_list = []
    while not isEqual(node, init):
        path_list.append(node)
        if node not in path:
            return None
        node = path[node]
    path_list.append(tuple(init))   
    path_list.reverse()
    return path_list

"""
    Given the list of nodes traversed,
    populate a matrix which shows appropriate
    arrows for the action taken at each cell
"""
def populatePathMatrix(path):
    if not path:
        return "No Solution"
    pathMatrix = [ ['' for i in range(len(grid[0]))] for i in range(len(grid)) ]

    for i in range(0, len(path) - 1):
        for j in range(len(delta)):
            if isEqual( tuple(map(lambda x, y: x + y, path[i], delta[j])), path[i + 1]):
                pathMatrix[path[i][0]][path[i][1]] = delta_name[j]
    return pathMatrix


def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    
    # initial node
    expansion_list = [[0, heuristic[init[0]][init[1]], init[0], init[1]]]
    expansion_list_temp = []
    expansion_count = 0

    # Continue as long as there are nodes to be expanded
    while len(expansion_list) != 0:

        # Check if goal has been reached
        state = findGoal(expansion_list)
        if(state):
            markExpanded(state, expansion_count)
            return state

        # Remove a node to be expanded
        next_node = findLowestFValueNode(expansion_list)

        # Update the order of expansion of the node
        markExpanded(next_node, expansion_count)
        expansion_count += 1

        # Mark the node as visited
        mark(next_node)

        reachable_nodes = findReachableNodes(next_node)
        for node in reachable_nodes:
            mark(node)

        # Populate path for backtracking
        populatePath(reachable_nodes, next_node)

        # The reachable nodes will be the ones we will later 
        # Add to the expansion list
        expansion_list.extend(reachable_nodes)
    
    return 'fail'

print search(grid, init, goal, cost)
for row in expand:
    print row

pathTraversed = tracePath(path, goal)

for row in populatePathMatrix(pathTraversed):
    print row
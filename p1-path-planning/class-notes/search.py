# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1
MARKER = 2

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

"""
    State list of the form [[cost, position-x, position-y]]
"""
def findGoal(state_list):
    for state in state_list:
        if( state[1] == goal[0] and state[2] == goal[1]):
            return state

"""
   State list of the form [[cost, position-x, position-y]]
   Return the node with the lowest cost 
   Also removes the node from the list
"""
def findLowestCodeNode(state_list):
    lowest_cost_node = min(state_list, key=lambda x: x[0])
    state_list.remove(lowest_cost_node)
    return lowest_cost_node

"""
    Mark a node as visited
"""
def mark(node):
    grid[node[1]][node[2]] = MARKER

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
    node_x = node[1]
    node_y = node[2]

    return filter(lambda x: x[1] >= 0 and
                         x[2] >=0 and 
                         x[1] < len(grid) and
                         x[2] < len(grid[0]) and
                         grid[x[1]][x[2]]!=1  and 
                         grid[x[1]][x[2]]!=2,
    [[node_cost, node_x + delta_x, node_y + delta_y] for (delta_x, delta_y) in delta] )


def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    
    # initial node
    expansion_list = [[0, init[0], init[1]]]
    expansion_list_temp = []

    while len(expansion_list) != 0:

        # Check if goal has been reached
        state = findGoal(expansion_list)
        if(state):
            return [state[0], state[1], state[2]]

        next_node = findLowestCodeNode(expansion_list)

        mark(next_node)

        reachable_nodes = findReachableNodes(next_node)
        expansion_list_temp.extend(reachable_nodes)

        if(len(expansion_list) == 0):
            expansion_list = expansion_list_temp
            expansion_list_temp = []
    
    return 'Search Failed'

print search(grid, init, goal, cost)
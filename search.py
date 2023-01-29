from problem import HeuristicFunction, Problem, S, A, Solution
from collections import deque
from helpers import utils
import heapq
from itertools import count

#TODO: Import any modules you want to use

# All search functions take a problem and a state
# If it is an informed search function, it will also receive a heuristic function
# S and A are used for generic typing where S represents the state type and A represents the action type

# All the search functions should return one of two possible type:
# 1. A list of actions which represent the path from the initial state to the final state
# 2. None if there is no solution

def BreadthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    frontier = deque()
    frontier.appendleft(initial_state)
    
    child_parent = dict() # used to relate each node to its parent and the action taken to reach it
    child_parent[initial_state] = ("start","start") # dumby parent tuple for initial state
    
    # create explored set and state_list i.e. solution
    explored = set()
    state_list = []
    
    while len(frontier) != 0:
        current_state = frontier[-1]
        frontier.pop()
        # check if current state is goal and prepare solution to return
        if problem.is_goal(current_state):
            if current_state == initial_state: return list()
            tple = child_parent[current_state]
            while tple != ("start","start"): 
                state_list.append(tple[1])
                tple = child_parent[tple[0]]
            state_list.reverse()
            return state_list
        
        explored.add(current_state)
        
        for action in problem.get_actions(current_state):
            child = problem.get_successor(current_state,action)
            if (child not in frontier) and (child not in explored):
                child_parent[child] = (current_state,action)
                frontier.appendleft(child)
    return None

def DepthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    frontier = deque()
    frontier.append(initial_state)

    child_parent = dict() # used to relate each node to its parent and the action taken to reach it
    child_parent[initial_state] = ("start","start") # dumby parent tuple for initial state
    
    # create explored set and state_list i.e. solution
    explored = set()
    state_list = []

    while len(frontier) != 0:

        current_state = frontier[-1]
        frontier.pop()
        # check if current state is goal and prepare solution to return
        if problem.is_goal(current_state):
            if current_state == initial_state: return list()
            tple = child_parent[current_state]
            while tple != ("start","start"):
                state_list.append(tple[1])
                tple = child_parent[tple[0]]
            state_list.reverse()
            return state_list

        explored.add(current_state)

        for action in problem.get_actions(current_state):
            child = problem.get_successor(current_state,action)
            if (child not in frontier) and (child not in explored):
                frontier.append(child)
                child_parent[child] = (current_state,action) # add parent node of child and action to child_parent dict
    return None

# used to check if child state of the current state is already in the frontier
def in_frontier (frontier,child):
    for values in frontier:
        if child == values[2]:
            return True
    return False


def UniformCostSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    child_parent = dict() # used to relate each node to its parent and the action taken, and cost to reach it
    child_parent[initial_state] = ("start","start",0) # a dumby tuple for the initial state

    cnt = count() # used to differentiate between equal costs to avoid comparing value which is non comparable
    
    frontier = []
    heapq.heapify(frontier) # used heapq to be able to turn it back into a list to iterate over it then heapify again
    heapq.heappush(frontier,[0,next(cnt),initial_state]) # chose list rather than tuple for it to be mutable
    
    explored = set()

    while len(frontier) != 0:
        current_node = heapq.heappop(frontier) # get smallest node

        if problem.is_goal(current_node[2]): # check if goal and return solution
            solution = []
            if current_node[2] == initial_state: return list()
            tple = child_parent[current_node[2]]
            while tple != ("start","start",0):
                solution.append(tple[1])  
                tple = child_parent[tple[0]]
            solution.reverse()
            return solution

        explored.add(current_node[2])

        for action in problem.get_actions(current_node[2]):

            child = problem.get_successor(current_node[2],action)
            child_cost = problem.get_cost(current_node[2],action)

            if (not in_frontier(list(frontier),child)) and (child not in explored):
                # calculate new total cost of child node to insert cost in frontier
                new_path_cost = 0
                tple = child_parent[current_node[2]]
                while tple != ("start","start",0):
                    new_path_cost = new_path_cost + tple[2]
                    tple = child_parent[tple[0]]
                new_path_cost = new_path_cost + child_cost

                heapq.heappush(frontier,[new_path_cost,next(cnt),child])
                child_parent[child] = (current_node[2],action,child_cost)
            
            elif in_frontier(list(frontier),child):
                
                frontier = list(frontier) # to loop and change values if needed
                for node in frontier:
                    if node[2] == child:

                        # calculate path cost for child from the 2 different paths
                        original_path_cost = 0
                        tple = child_parent[child]
                        while tple != ("start","start",0):
                            original_path_cost = original_path_cost + tple[2]
                            tple = child_parent[tple[0]]

                        new_path_cost = 0
                        tple = child_parent[current_node[2]]
                        while tple != ("start","start",0):
                            new_path_cost = new_path_cost + tple[2]
                            tple = child_parent[tple[0]]
                        new_path_cost = new_path_cost + child_cost

                        if original_path_cost > new_path_cost: # if new path is cheaper change node path in child_parent and update its cost
                            node[0] = new_path_cost # if there is a cheaper path to frontier node change its value
                            child_parent[child] = (current_node[2],action,child_cost) # change parent of child already in frontier if path is cheaper
                            break
                heapq.heapify(frontier) # turn back into a heap to maintain smallest element pop     
    return None
            


def AStarSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    child_parent = dict() # used to relate each node to its parent and the action taken, and cost to reach it
    child_parent[initial_state] = ("start","start",0) # a dumby tuple for the initial state

    cnt = count() # used to differentiate between equal costs to avoid comparing value which is non comparable
    
    frontier = []
    heapq.heapify(frontier) # used heapq to be able to turn it back into a list to iterate over it then heapify again
    heapq.heappush(frontier,[0,next(cnt),initial_state]) # chose list rather than tuple for it to be mutable
    
    explored = set()

    while len(frontier) != 0:
        current_node = heapq.heappop(frontier) # get smallest node

        if problem.is_goal(current_node[2]): # check if goal and return solution
            solution = []
            if current_node[2] == initial_state: return list()
            tple = child_parent[current_node[2]]
            while tple != ("start","start",0):
                solution.append(tple[1])  
                tple = child_parent[tple[0]]
            solution.reverse()
            return solution

        explored.add(current_node[2])

        for action in problem.get_actions(current_node[2]):

            child = problem.get_successor(current_node[2],action)
            child_cost = problem.get_cost(current_node[2],action)
            heuristic_value = heuristic(problem,child)

            if (not in_frontier(list(frontier),child)) and (child not in explored):
                # calculate new total cost of child node to insert cost + heuristic value in frontier
                new_path_cost = 0
                tple = child_parent[current_node[2]]
                while tple != ("start","start",0):
                    new_path_cost = new_path_cost + tple[2]
                    tple = child_parent[tple[0]]
                new_path_cost = new_path_cost + child_cost

                heapq.heappush(frontier,[new_path_cost+heuristic_value,next(cnt),child])
                child_parent[child] = (current_node[2],action,child_cost)
            
            elif in_frontier(list(frontier),child):
                
                frontier = list(frontier) # to loop and change values if needed
                for node in frontier:
                    if node[2] == child:

                        # calculate path cost for child from the 2 different paths
                        original_path_cost = 0
                        tple = child_parent[child]
                        while tple != ("start","start",0):
                            original_path_cost = original_path_cost + tple[2]
                            tple = child_parent[tple[0]]

                        new_path_cost = 0
                        tple = child_parent[current_node[2]]
                        while tple != ("start","start",0):
                            new_path_cost = new_path_cost + tple[2]
                            tple = child_parent[tple[0]]
                        new_path_cost = new_path_cost + child_cost

                        if original_path_cost > new_path_cost: # if new path is cheaper change node path in child_parent and update its cost
                            node[0] = new_path_cost + heuristic_value # if there is a cheaper path to frontier node change its value
                            child_parent[child] = (current_node[2],action,child_cost) # change parent of child already in frontier if path is cheaper
                            break
                heapq.heapify(frontier) # turn back into a heap to maintain smallest element pop     
    return None

def BestFirstSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    child_parent = dict() # used to relate each node to its parent and the action taken to reach it and heuristic
    child_parent[initial_state] = ("start","start") # a dumby tuple for the initial state

    cnt = count() # used to differentiate between equal costs to avoid comparing value which is non comparable
    
    frontier = []
    heapq.heapify(frontier) # used heapq to be able to turn it back into a list to iterate over it then heapify again
    heapq.heappush(frontier,[0,next(cnt),initial_state]) # chose list rather than tuple for it to be mutable
    
    explored = set()

    while len(frontier) != 0:
        current_node = heapq.heappop(frontier) # get smallest node

        if problem.is_goal(current_node[2]): # check if goal and return solution
            solution = []
            if current_node[2] == initial_state: return list()
            tple = child_parent[current_node[2]]
            while tple != ("start","start"):
                solution.append(tple[1])  
                tple = child_parent[tple[0]]
            solution.reverse()
            return solution

        explored.add(current_node[2])

        for action in problem.get_actions(current_node[2]):

            child = problem.get_successor(current_node[2],action)
            heuristic_value = heuristic(problem,child)

            if (not in_frontier(list(frontier),child)) and (child not in explored):

                heapq.heappush(frontier,[heuristic_value,next(cnt),child])
                child_parent[child] = (current_node[2],action)
            
            elif in_frontier(list(frontier),child):
                
                frontier = list(frontier) # to loop and change values if needed
                for node in frontier:
                    if node[2] == child:
                        if node[0] > heuristic_value:
                            node[0] = heuristic_value # if there is a cheaper path to frontier node chnage its value
                            child_parent[child] = (current_node[2],action) # change parent of child already in frontier if path is cheaper
                            break
                heapq.heapify(frontier) # turn back into a heap to maintain smallest element pop     
    return None
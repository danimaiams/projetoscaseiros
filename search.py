

import util

class SearchProblem:


    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
   
        util.raiseNotDefined()

    def getSuccessors(self, state):
      
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
      
        util.raiseNotDefined()


def tinyMazeSearch(problem):
   
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    explorado = [] 
    frontier = util.Stack()
    frontier.push([problem.getStartState(),[],[]])
    while frontier.isEmpty()==0:
        state, actions, c = frontier.pop()
        if state not in explorado:
            explorado.append(state)
            if problem.isGoalState(state):
                return actions
            for proximo, acao, custo in problem.getSuccessors(state):
                frontier.push([proximo,actions + [acao],custo])
    return []

def breadthFirstSearch(problem):
    frontier = util.Queue()
    explorado = [] 
    frontier.push([problem.getStartState(),[],[]])
    while frontier.isEmpty()==0:
        state, actions, c = frontier.pop()
        if state not in explorado:
            explorado.append(state)
            if problem.isGoalState(state):
                return actions
            for proximo, acao, custo in problem.getSuccessors(state):
                frontier.push([proximo,actions + [acao],custo])
    return []

def uniformCostSearch(problem):
    frontier = util.PriorityQueue()
    frontier.push([problem.getStartState(),[],0],0)
    explorado = [] 
    while frontier.isEmpty()==0:
        state, actions, c = frontier.pop()
        if state not in explorado:
            explorado.append(state)
            if problem.isGoalState(state):
                return actions
            for proximo, acao, custo in problem.getSuccessors(state):
                frontier.push([proximo,actions + [acao],c+custo],c+custo)
    return []

def nullHeuristic(state, problem=None):
  
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    frontier = util.PriorityQueue()
    explorado = []
    frontier.push([problem.getStartState(),[],0],0)
    while frontier.isEmpty()==0:
        state, actions, c = frontier.pop()
        if state not in explorado:
            explorado.append(state)
            if problem.isGoalState(state):
                return actions
            for proximo, acao, custo in problem.getSuccessors(state):
                h=heuristic(proximo,problem)
                frontier.push([proximo,actions + [acao],c+custo],c+custo+h)
    return []


bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


from game import Directions
from game import Agent
from game import Actions
import util
import time
import search

class GoWestAgent(Agent):
    "An agent that goes West until it can't."

    def getAction(self, state):
        "The agent receives a GameState (defined in pacman.py)."
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP



class SearchAgent(Agent):


    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):

        if fn not in dir(search):
            raise AttributeError, fn + ' is not a search function in search.py.'
        func = getattr(search, fn)
        if 'heuristic' not in func.func_code.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError, heuristic + ' is not a function in searchAgents.py or search.py.'
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            self.searchFunction = lambda x: func(x, heuristic=heur)

        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError, prob + ' is not a search problem type in SearchAgents.py.'
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
       
        if self.searchFunction == None: raise Exception, "No search function provided for SearchAgent"
        starttime = time.time()
        problem = self.searchType(state) # Makes a new search problem
        self.actions  = self.searchFunction(problem) # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
  
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP

class PositionSearchProblem(search.SearchProblem):
  

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1,1), start=None, warn=True, visualize=True):
 
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print 'Warning: this does not look like a regular search maze'

        self._visited, self._visitedlist, self._expanded = {}, [], 0

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def getSuccessors(self, state):


        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):

        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost

class StayEastSearchAgent(SearchAgent):
  
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)

class StayWestSearchAgent(SearchAgent):
  
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)

def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

class CornersProblem(search.SearchProblem):
 

    def __init__(self, startingGameState):

        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height-2, self.walls.width-2
        self.corners = ((1,1), (1,top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print 'Warning: no food in corner ' + str(corner)
        self._expanded = 0 
        
    def getStartState(self):
        return [self.startingPosition, []]

    def isGoalState(self, state):
        "Returns whether this search state is a goal state of the problem"
        "*** YOUR CODE HERE ***"

        #Confere se todos os cantos foram visitados
        s=0
        ncantos=4
        if not (len(state[1])==0):
            for aux2 in state[1]:
                if self.corners.__contains__(aux2):
                    s=(s+1)
            if (s == ncantos):
                return True
            else:
                return False
        return False

    

    def getSuccessors(self, state):
  
        successors = []
        visitados=state[1]
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          
            x,y = state[0]
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            hitsWall = self.walls[nextx][nexty]
            stepCost=self.getCostOfActions([action])
            newPosition=(nextx,nexty)
                        
            #Verifica se o Pacman bate na parede. Se n�o, anexa um novo estado � lista
            if hitsWall == False:
                if ((self.corners.__contains__(newPosition)) & (not visitados.__contains__(newPosition))):
                    successors.append(([newPosition,visitados + [newPosition]], action, stepCost))
                else:
                    successors.append(([newPosition,visitados], action, stepCost))
            
        self._expanded += 1
        return successors

    def getCostOfActions(self, actions):
   
        return len(actions)
    
def distancia(pos1,pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def cornersHeuristic(state, problem):
 
    corners = problem.corners 
    walls = problem.walls 
    objetivo=0
    somacantos=0
    cantovisitado=state[1]
    posicaopacman=state[0]
    cantoproximo=[]

    aux1=float('inf')
    for aux2 in corners:
        if aux2 not in cantovisitado:
            if distancia(posicaopacman,aux2) < aux1:
                aux1=distancia(posicaopacman,aux2)
                cantoproximo=aux2

    canto=cantoproximo
    cantonaovisitado=[]
    r=range(0,len(corners))
    for i in r:
        cantonaovisitado.append(corners[i])
    for i in cantovisitado:
        if i in cantonaovisitado:
            cantonaovisitado.remove(i)
    if cantoproximo in cantonaovisitado:
        cantonaovisitado.remove(cantoproximo)

    while (len(cantonaovisitado)>0):
        menordistancia=float('inf')
        for aux2 in cantonaovisitado:
            if ((distancia(canto,aux2)) < menordistancia):
                menordistancia=distancia(canto,aux2)
                cantomaisproximo=aux2
        canto=cantomaisproximo
        somacantos=somacantos+menordistancia
        cantonaovisitado.remove(cantomaisproximo)

    if len(cantovisitado)==len(corners):
        aux1=objetivo
        somacantos=objetivo
    
    return aux1+somacantos

class AStarCornersAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem

class FoodSearchProblem:
  
    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0
        self.heuristicInfo = {} 

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append( ( ((nextx, nexty), nextFood), direction, 1) )
        return successors

    def getCostOfActions(self, actions):
      
        x,y= self.getStartState()[0]
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

class AStarFoodSearchAgent(SearchAgent):
    "A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem

def foodHeuristic(state, problem):
    posicaopacman, foodGrid = state
    distanciatotal=0
    aux1=0
    aux3=3
    objetivo=0
    naocomido = foodGrid.asList()
    comidaproxima=[] ###################

    aux1=float('inf')
    for f in naocomido:
        if distancia(posicaopacman,f) < aux1:
            aux1=distancia(posicaopacman,f)
            comidaproxima=f ###############

    comida=comidaproxima #################

    if comida in naocomido:
        naocomido.remove(comida)

    while (len(naocomido)>aux3):
        menordistancia=float('inf')
        for f in naocomido:
            if ((distancia(comida,f)) < menordistancia):
                menordistancia=distancia(comida,f)
                comidaproxima=f
        comida=comidaproxima
        distanciatotal=distanciatotal+menordistancia
        naocomido.remove(comidaproxima)

    if len(foodGrid.asList())==objetivo:
        aux1=objetivo
        distanciatotal=objetivo
    return aux1+distanciatotal

class ClosestDotSearchAgent(SearchAgent):
    "Search for all food using a sequence of searches"
    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while(currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState) 
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception, 'findPathToClosestDot returned an illegal move: %s!\n%s' % t
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print 'Path found with cost %d.' % len(self.actions)


    def findPathToClosestDot(self, gameState):
        
        posicaopacman = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)
        frontier = util.Queue()
        visited = []
        
        #Implementação do A*
        frontier.push([problem.getStartState(),[],[]])
        while frontier.isEmpty()==0:
            state, listadeacoes, costs = frontier.pop()
            if state not in visited:
                visited.append(state)
                if problem.isGoalState(state):
                    return listadeacoes
                for proximo, action, cost in problem.getSuccessors(state):
                    frontier.push([proximo,listadeacoes + [action],cost])
        print 'Não foi encontrado um caminho'
        return []
        

class AnyFoodSearchProblem(PositionSearchProblem):
  

    def __init__(self, gameState):
        self.food = gameState.getFood()

        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0

    def isGoalState(self, state):
     
        x,y = state

        if state in self.food.asList():
            return True
        return False

##################
# Mini-contest 1 #
##################

class ApproximateSearchAgent(Agent):

    def registerInitialState(self, state):


    def getAction(self, state):

        util.raiseNotDefined()

def mazeDistance(point1, point2, gameState):
  
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + point1
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))



from util import manhattanDistance
from game import Directions
import random, util

from game import Agent
from game import Actions

class ReflexAgent(Agent):
  def getAction(proprio, estado):
    legalMoves = estado.getLegalActions()

    scores = [proprio.evaluationFunction(estado, acao) for acao in legalMoves]
    bestScore = max(scores)
    bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
    chosenIndex = random.choice(bestIndices) # Pick randomly among the best

    return legalMoves[chosenIndex]

  def evaluationFunction(proprio, currentGameState, acao):
    successorGameState = currentGameState.generatePacmanSuccessor(acao)
    newPos = successorGameState.getPacmanPosition()
    oldFood = currentGameState.getFood()
    newGhostStates = successorGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    return successorGameState.getScore()
	
def scoreEvaluationFunction(currentGameState):
  return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
  def __init__(proprio, evalFn = 'scoreEvaluationFunction', depth = '2'):
    proprio.index = constante # Pacman is always agent index 0
    proprio.evaluationFunction = util.lookup(evalFn, globals())
    proprio.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):

  def getMinMaxAction(proprio, estado, atual, profundidade):
    if (atual == estado.getNumAgents()) and profundidade>1:
      return proprio.getMinMaxAction(estado, 0, profundidade-1)
    if (atual == estado.getNumAgents()) and profundidade==1:
      return [None, proprio.evaluationFunction(estado)]

    acoes = estado.getLegalActions(atual)
    if len(acoes)==0:
      return [None, proprio.evaluationFunction(estado)]
    mav=float("Inf")
    mxav=float("-Inf")
    mina = None
    mxa = None
    for acao in acoes:
      if acao==Directions.STOP:
        continue
      [newaction, valor] = proprio.getMinMaxAction(estado.generateSuccessor(atual, acao), atual+1, profundidade)
      if valor<mav:
        mav=valor
        mina = acao
      if valor>mxav:
        mxav=valor
        mxa = acao
mav
    if atual!=0:
      return [mina, mav]
    else:
      return [mxa, mxav]

  def getAction(proprio, estado):
    atual = 0
    [acao, valor] = proprio.getMinMaxAction(estado, atual, proprio.depth)
    print("O valor é" + str(valor))
    return acao


class AlphaBetaAgent(MultiAgentSearchAgent):
  def getMinMaxAction(proprio, estado, atual, profundidade, melhor, pior):
    if (atual == estado.getNumAgents()) and profundidade>1:      
      return proprio.getMinMaxAction(estado, 0, profundidade-1,melhor, pior)
    if (atual == estado.getNumAgents()) and profundidade==1:
      return [None, proprio.evaluationFunction(estado)]

    acoes = estado.getLegalActions(atual)
    if len(acoes)==0:
      return [None, proprio.evaluationFunction(estado)]
    mav=float("Inf")
    mxav=float("-Inf")
    mina = None
    mxa = None
    for acao in acoes:
      if acao==Directions.STOP:
        continue
      [newaction, valor] = proprio.getMinMaxAction(estado.generateSuccessor(atual, acao), atual+1, profundidade,melhor, pior)
      if (atual != 0 and melhor>valor) or (atual == 0 and pior<valor):
        return [acao, valor]
      if valor<mav:
        mav=valor
        mina = acao
      if valor>mxav:
        mxav=valor
        mxa = acao
      if (atual==0 and valor>melhor):
        melhor == valor 
      if (atual!=0 and valor<pior):
        pior == valor

    if atual!=0:
      return [mina, mav]
    else:
      return [mxa, mxav]
  def getAction(proprio, estado):
    atual = 0
    melhorencontrado = float("-Inf")
    piorencontrado = float("Inf")
    [acao, valor] = proprio.getMinMaxAction(estado, atual, proprio.depth, melhorencontrado, piorencontrado)
    print("O valor é" + str(valor))
    return acao

class ExpectimaxAgent(MultiAgentSearchAgent):
  def getExpectiMaxAction(proprio, estado, atual, profundidade):
    if (atual == estado.getNumAgents()) and profundidade>1:
      return proprio.getExpectiMaxAction(estado, 0, profundidade-1)
    if (atual == estado.getNumAgents()) and profundidade==1:
      return [None, proprio.evaluationFunction(estado)]

    acoes = estado.getLegalActions(atual)
    if len(acoes)==0:
      return [None, proprio.evaluationFunction(estado)]
    mxav=float("-Inf")
    mxa = None
    soma = 0
    for acao in acoes:
      if acao==Directions.STOP:
        continue
      [newaction, valor] = proprio.getExpectiMaxAction(estado.generateSuccessor(atual, acao), atual+1, profundidade)
      soma = soma + valor
      if valor>mxav:
        mxav=valor
        mxa = acao

    if atual!=0:
      return [None, soma/len(acoes)]
    else:
      return [mxa, mxav]

  def getAction(proprio, estado):
    atual = 0
    [acao, valor] = proprio.getExpectiMaxAction(estado, atual, proprio.depth)
    print("O valor é " + str(valor))
    return acao


def closestFood(pos, food, walls):
  fringe = [(pos[0], pos[1], 0)]
  expanded = set()
  while fringe:
    pos_x, pos_y, dist = fringe.pop(0)
    if (pos_x, pos_y) in expanded:
      continue
    expanded.add((pos_x, pos_y))
    if food[pos_x][pos_y]:
      return dist
    nbrs = Actions.getLegalNeighbors((pos_x, pos_y), walls)
    for nbr_x, nbr_y in nbrs:
      fringe.append((nbr_x, nbr_y, dist+1))
  return None

def getFeatures(state):
  food = state.getFood()
  walls = state.getWalls()
  ghosts = state.getGhostPositions()

  features = util.Counter()
    
  features["bias"] = 1.0
    
  x, y = state.getPacmanPosition()
    
  badghostscount = 0
  for i in range(1, state.getNumAgents()):
    ghost_pos = state.getGhostPosition(i)
    ghost_state = state.getGhostState(i)
    if (x,y) in Actions.getLegalNeighbors(ghost_pos, walls) and not ghost_state.scaredTimer > 0:
      badghostscount = badghostscount + 1

  features["#-of-ghosts-1-step-away"] = sum((x, y) in Actions.getLegalNeighbors(g, walls) for g in ghosts)

  features["#-of-badghosts"] = badghostscount

  if not features["#-of-ghosts-1-step-away"] and food[x][y]:
    features["eats-food"] = 1.0
    
  dist = closestFood((x, y), food, walls)
  if dist is not None:
    features["closest-food"] = float(dist) / (walls.width * walls.height)
  return features
  

def betterEvaluationFunction(currentGameState):  
    
    """Serve para computar vitorias e derrotas e avaliar a pontuação"""

  pos = currentGameState.getPacmanPosition()
  food = currentGameState.getFood()
  walls = currentGameState.getWalls()

  features = getFeatures(currentGameState)

  if currentGameState.isWin():
    return 2000
  elif currentGameState.isLose():
    return -5000
  
  total = currentGameState.getScore()

  if features["#-of-ghosts-1-step-away"]:
    total = total - 300 * features["#-of-ghosts-1-step-away"]

  if features["closest-food"]:
    total = total + 50 * (1 - features["closest-food"]) * (1 - features["closest-food"])
  

  return total

better = betterEvaluationFunction


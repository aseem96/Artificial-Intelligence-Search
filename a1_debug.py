import math
import sys
from collections import deque

def heuristic(src, dest, coordinates):
    lat1 = float(coordinates[src][0])
    long1 = float(coordinates[src][1])
    lat2 = float(coordinates[dest][0])
    long2 = float(coordinates[dest][1])
    return math.sqrt((69.5 * (lat1 - lat2)) ** 2 + (69.5 * math.cos((lat1 + lat2)/360 * math.pi) * (long1 - long2)) ** 2)

def createAdjacencyList(filename,graph):
    for line in open(filename):
        tokens = line.split(',')
        sourceCity = tokens[0].strip()
        destCity = tokens[1].strip()
        distance = tokens[2].strip()
        if sourceCity in graph:
            graph[sourceCity].append(destCity)
        else:
            graph[sourceCity] = [destCity]
        if destCity in graph:
            graph[destCity].append(sourceCity)
        else:
            graph[destCity] = [sourceCity]
    for city in graph:
        graph[city] = sorted(graph[city])
    return graph

def dfs(graph, src, dest):
    stack = [(src, [src])]
    visited = set()
    while stack:
        (vertex, path) = stack.pop()
        if vertex not in visited:
            if vertex == dest:
                return path
            visited.add(vertex)
            for neighbor in reversed(graph[vertex]):
                stack.append((neighbor, path + [neighbor]))

def bfs(graph, src, dest):
    queue = []
    queue.append([src])
    while queue:
        path = queue.pop(0)
        vertex = path[-1]
        if vertex == dest:
            return path
        for neighbor in reversed(graph[vertex]):
            new_path = list(path)
            new_path.append(neighbor)
            queue.append(new_path)

def parseCities(filename, cities, coordinates):
    count = 0
    for line in open(filename):
        tokens = line.split(',')
        city = tokens[0].strip()
        latitude = tokens[1].strip()
        longitude = tokens[2].strip()
        cities[city] = count
        count = count + 1
        coordinates[city] = [latitude,longitude]
    return cities,coordinates

def createAdjacencyMatrix(filename, graph, cities):
    for line in open(filename):
        tokens = line.split(',')
        sourceCity = tokens[0].strip()
        destCity = tokens[1].strip()
        distance = tokens[2].strip()
        graph[cities[sourceCity]][cities[destCity]] = distance
        graph[cities[destCity]][cities[sourceCity]] = distance
    return graph

def distBetween(x,y,graph,cities):
    return int(graph[cities[x]][cities[y]])

def reconstructPath(cameFrom, dest):
   path = deque()
   node = dest
   path.appendleft(node)
   while node in cameFrom:
       node = cameFrom[node]
       path.appendleft(node)
   return path

def getLowest(openSet, fScore):
   lowest = float("inf")
   lowestNode = None
   for node in openSet:
       if fScore[node] < lowest:
           lowest = fScore[node]
           lowestNode = node
   return lowestNode

def astar(graph,src,dest,coordinates,cities,adjMatrix):
    cameFrom = dict()
    openSet = set([src])
    closedSet = set()
    gScore = dict()
    for city in cities:
        gScore[city] = math.inf
    fScore = dict()
    gScore[src] = 0
    fScore[src] = gScore[src] + heuristic(src,dest,coordinates)
    print("fScore = %s" %(fScore))
    while len(openSet) != 0:
        current = getLowest(openSet,fScore)
        print("Current = %s" %(current))
        if current == dest:
            return(reconstructPath(cameFrom,dest))
        openSet.remove(current)
        closedSet.add(current)
        print("closedSet = %s" %(closedSet))
        print("openSet = %s" %(openSet))
        for neighbor in graph[current]:
            if neighbor in closedSet:
                continue
            if neighbor not in openSet:
                openSet.add(neighbor)
            tentative_gScore = gScore[current] + distBetween(current,neighbor,adjMatrix,cities)
            if tentative_gScore >= gScore[neighbor]:
                continue
            cameFrom[neighbor] = current
            gScore[neighbor] = tentative_gScore
            fScore[neighbor] = gScore[neighbor] + heuristic(neighbor,dest,coordinates)
    return 0

def SearchUSA():
    searchtype = sys.argv[1]
    srccity = sys.argv[2]
    destcity = sys.argv[3]

    adjList = dict()
    adjList = createAdjacencyList("road_info.txt",adjList)

    cities = dict()
    coordinates = dict()
    cities,coordinates = parseCities("city_info.txt",cities,coordinates)
    adjMatrix = [[0 for x in range(len(adjList))] for y in range(len(adjList))]
    adjMatrix = createAdjacencyMatrix("road_info.txt",adjMatrix, cities)

    if searchtype.lower() == "dfs":
        path = dfs(adjList,srccity,destcity)
        print(path)
        print(len(path))
    elif searchtype.lower() == "bfs":
        path = bfs(adjList,srccity,destcity)
        print(path)
        print(len(path))
    elif searchtype.lower() == "astar":
        path = astar(adjList,srccity,destcity,coordinates,cities,adjMatrix)
        print(path)
        print(len(path))
    else:
        print("Wrong Choice")

SearchUSA()

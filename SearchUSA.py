import math
import sys
from collections import deque

def heuristic(src, dest, coordinates):          #heuristic function as given in question
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
    stack = [(src, [src])]                  #store the initial source vertex
    visited = set()                         #visited set is empty
    while stack:
        (vertex, path) = stack.pop()        #pop a node
        if vertex not in visited:           #check of visited
            if vertex == dest:              #reached destination
                return path, visited
            visited.add(vertex)             #add in visited
            for neighbor in reversed(graph[vertex]):
                stack.append((neighbor, path + [neighbor])) #push vertex and path in stack

def bfs(graph, src, dest):
    queue = []                              #empty queue
    queue.append([src])                     #add initial source vertex to queue
    visited = set()                         #visited set is empty
    while queue:
        path = queue.pop(0)                 #pop first element of queue
        vertex = path[-1]                   #current vertex
        visited.add(vertex)                 #visit vertex
        for neighbor in graph[vertex]:      #check neighbors
            if neighbor not in visited:
                if neighbor == dest:        #reached destination
                    path.append(dest)
                    return path,visited
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
   return list(path)

def getLowest(openSet, fScore):
   lowest = float("inf")
   lowestNode = None
   for node in openSet:
       if fScore[node] < lowest:
           lowest = fScore[node]
           lowestNode = node
   return lowestNode

def astar(graph,src,dest,coordinates,cities,adjMatrix):
    cameFrom = dict()       #For each node, which node it can most efficiently be reached from
    openSet = set([src])    #The set of currently discovered nodes that are not evaluated yet
    closedSet = set()       #The set of nodes already evaluated
    gScore = dict()         #For each node, the cost of getting from the start node to that node
    fScore = dict()         #For each node as intermediate, the total cost of getting from the start node to the goal
    for city in cities:
        gScore[city] = float('inf')
        fScore[city] = float('inf')
    gScore[src] = 0
    fScore[src] = gScore[src] + heuristic(src,dest,coordinates) #For the first node, that value is completely heuristic
    while len(openSet) != 0:
        current = getLowest(openSet,fScore)
        if current == dest:
            return(reconstructPath(cameFrom,dest),closedSet)
        openSet.remove(current)
        closedSet.add(current)
        for neighbor in graph[current]:
            if neighbor in closedSet:
                continue                    #Ignore the neighbor which is already evaluated
            if neighbor not in openSet:     #Discover a new node
                openSet.add(neighbor)
            tentative_gScore = gScore[current] + distBetween(current,neighbor,adjMatrix,cities) #The distance from start to a neighbor
            if tentative_gScore >= gScore[neighbor]:
                continue                    #This is not a better path
            cameFrom[neighbor] = current    #This path is the best until now. Record it
            gScore[neighbor] = tentative_gScore
            fScore[neighbor] = gScore[neighbor] + heuristic(neighbor,dest,coordinates)
    return 0

def SearchUSA():
    searchtype = sys.argv[1]    #dfs, bfs or astar
    srccity = sys.argv[2]       #source city
    destcity = sys.argv[3]      #destination city

    adjList = dict()            #define an empty dict
    adjList = createAdjacencyList("road_info.txt",adjList)  #create adjacency list

    cities = dict()             #store {"city_name":city_id (0/1/2...)}
    coordinates = dict()        #store {"city_name":[latitude, longitude]}
    cities,coordinates = parseCities("city_info.txt",cities,coordinates)
    adjMatrix = [[0 for x in range(len(adjList))] for y in range(len(adjList))] #n x n (n = number of unique cities)
    adjMatrix = createAdjacencyMatrix("road_info.txt",adjMatrix, cities)        #create adjacency matrix using cities

    if searchtype.lower() == "dfs":
        path,expanded = dfs(adjList,srccity,destcity)
        print "Expanded nodes: " + str(sorted(list(expanded)))
        print "Number of Expanded nodes: " + str(len(expanded))
        print "Path: " + str(path)
        print "Length of solution path: " + str(len(path)-1)
    elif searchtype.lower() == "bfs":
        path,expanded = bfs(adjList,srccity,destcity)
        print "Expanded nodes: " + str(sorted(list(expanded)))
        print "Number of Expanded nodes: " + str(len(expanded))
        print "Path: " + str(path)
        print "Length of solution path: " + str(len(path)-1)
    elif searchtype.lower() == "astar":
        path,expanded = astar(adjList,srccity,destcity,coordinates,cities,adjMatrix)
        cost = 0
        for index,node in enumerate(path):
            if index != 0:
                cost = cost + distBetween(path[index-1],path[index],adjMatrix,cities)
        print "Expanded nodes: " + str(sorted(list(expanded)))
        print "Number of Expanded nodes: " + str(len(expanded))
        print "Path: " + str(path)
        print "Length of solution path: " + str(len(path)-1)
        print "Cost of solution path: " + str(cost)
    else:
        print "Usage: python a1.py search_type source_city dest_city"

if __name__ == "__main__":
    SearchUSA()

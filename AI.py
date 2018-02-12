import math
import sys

global max_len
max_len = 1

class City:
    def __init__(self, name, latitude, longitude):
        self.name = name
        self.latitude = float(latitude)
        self.longitude = float(longitude)
        self.expanded = False
        self.roads = []


class Road:
    def __init__(self, city1, city2, length):
        self.city1 = city1
        self.city2 = city2
        if not length or length =='':
            self.length = -1
        else:
            self.length = float(length)

def heuristic(long1, lat1, long2, lat2):
    return math.sqrt((69.5 * (lat1 - lat2)) ** 2 + (69.5 * math.cos((lat1 + lat2)/360 * math.pi) * (long1 - long2)) ** 2)

def parse_city(filename, cities):
    for line in open(filename):
        tokens = line.split(',')
        city = City(tokens[0],tokens[1],tokens[2])
        cities[city.name] = city
    return cities

def parse_roads(filename, cities):
    global max_len
    for line in open(filename):
        tokens = line.split(',')
        city1 = tokens[0].strip()
        city2 = tokens[1].strip()
        dist = Road(city1,city2,tokens[2].strip())

        if city1 in cities:
            cities[city1].roads.append(dist)
        else:
            print("City not found")
        if city2 in cities:
            cities[city2].roads.append(dist)
        else:
            print("City not found")

    if max_len < dist.length:
        max_len = dist.length

    return cities

def dfs1(cities,src,dest):
    parent = {}
    stack = [(src,[src])]
    print("Stack = %s" % (stack))

    while stack:
        (city_name, path) = stack.pop()
        print("\nPopped = %s, %s" % (city_name, path))
        cities[city_name].expanded = True

        city_roads = cities[city_name].roads
        print("City Roads = %s" % (city_roads))
        neighbors = []
        for c in city_roads:
            neighbors = neighbors + [c.city2]

        n = set(neighbors)
        print("Neighbors = %s" % (n))
        for i in n:
            if cities[i].expanded == True:
                continue
            for r in cities[city_name].roads:
                if (r.city2 == i):
                    parent[i] = [cities[city_name], r]
            print("i = %s" % (i))
            if (i != dest):
                new_city = (i, path + [i])
                stack.append(new_city)
                print("Stack here = %s" % (stack))
            else:
                print("here")
                p = path + [i]
                #PrintPath(parent,cities[src],cities[dest])
                #return p
                print(path)

def dfs(cities,src,dest,visited=None):
    stack = [(src, [src])]
    while stack:
        print ("\nStack = %s" % (stack))
        (vertex, path) = stack.pop()
        cities[vertex].expanded = True
        print ("City popped = %s" % (vertex))

        city_roads = cities[vertex].roads
        print("City_Roads = ")
        for r in city_roads:
            print(r.city1)

        neighbors = []
        for c in city_roads:
            neighbors = neighbors + [c.city2]

        n = set(neighbors)
        print("Neighbors = %s" % (n))
        for next in n - set(path):
            print("Next = %s" % (next))
            if next == dest:
                yield path + [next]
            else:
                stack.append((next, path + [next]))

def SearchUSA():
    cities = {}

    searchtype = sys.argv[1]
    srccity = sys.argv[2]
    destcity = sys.argv[3]

    #f = open('roads.txt','r').read().split('%====================================== Roads ========================================')
    #names = ['city_info.txt','road_info.txt']
    #for num,file in enumerate(f):
    #    open(names[num],'w').write(file)

    cities = parse_city('city_info.txt',cities)
    cities = parse_roads('road_info.txt',cities)

    #for city, value in cities.items():
    #    print("%s\t%s\t%s\t%s" % (value.name, value.longitude, value.latitude, value.roads))

    #print(list(dfs(cities,srccity,destcity)))
    for city, value in cities.items():
        for r in value.roads:
            print("%s %s %s" % (city, r.city1, r.city2))


SearchUSA()

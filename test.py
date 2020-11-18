import sys
import math
#https://www.bogotobogo.com/python/python_Dijkstras_Shortest_Path_Algorithm.php
class Vertex:
    def __init__(self, node, x, y):
        self.x = x
        self.y = y
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxsize
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.previous = None

    def __lt__(self, other):
        #bug in python3, need to override the default lessthan function
        return self.distance < other.distance

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])


class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node, x, y):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node, x, y)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def calculate_weight(self, v1: Vertex, v2: Vertex):
        return math.sqrt(math.pow(v1.x - v2.x, 2) + math.pow(v1.y - v2.y, 2))

    def add_edge(self, frm, to):
        if frm not in self.vert_dict or to not in self.vert_dict:
            raise Exception("invalid vertice, {} {}".format(frm, to))

        weight = self.calculate_weight(self.vert_dict[frm], self.vert_dict[to])
        self.vert_dict[frm].add_neighbor(self.vert_dict[to], weight)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], weight)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous


def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return


import heapq


def dijkstra(aGraph, start, target):
    print('''Dijkstra's shortest path''')
    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(), v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        # for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)

            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                print('updated : current = %s next = %s new_dist = %s'
                      % (current.get_id(), next.get_id(), next.get_distance()))
            else:
                print('not updated : current = %s next = %s new_dist = %s'
                % (current.get_id(), next.get_id(), next.get_distance()))

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(), v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)


if __name__ == '__main__':
    g = Graph()
    with open("input6.txt", 'r') as fp:
        # tokens[0] = vertices, tokens[1] = edges
        totalVertices, totalEdges = list(map(lambda d: int(d), fp.readline().strip().split(" ")))
        # tokens = fp.readline().strip().split(" ")
        # tokens[0] = int(tokens[0])
        # tokens[1] = int(tokens[1])
        verticeNum = 0
        def removeSpaceAndEmpty(s):
            return s != ' ' and s != ''

        while verticeNum < totalVertices:
            # [ '', '1',' ', ' ', ' ', ' ', '2']
            v, x, y = list(map(lambda d: int(d), filter(removeSpaceAndEmpty, fp.readline().strip().split(" "))))

            g.add_vertex(v, x, y)

            verticeNum += 1

        fp.readline() #read empty line
        edgeNum = 0
        while edgeNum < totalEdges:
            v1, v2 = list(map(lambda d: int(d), filter(removeSpaceAndEmpty, fp.readline().strip().split(" "))))
            g.add_edge(v1, v2)
            edgeNum += 1

    # g = Graph()
    #
    # g.add_vertex('a')
    # g.add_vertex('b')
    # g.add_vertex('c')
    # g.add_vertex('d')
    # g.add_vertex('e')
    #
    # g.add_edge('a', 'b', 6)
    # g.add_edge('a', 'd', 1)
    # g.add_edge('b', 'c', 5)
    # g.add_edge('b', 'e', 2)
    # g.add_edge('d', 'b', 2)
    # g.add_edge('d', 'e', 2)
    # g.add_edge('e', 'c', 5)
    #
    #
    print('Graph data:')
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print('( %s , %s, %3f)' % (vid, wid, v.get_weight(w)))

    origin = 0
    target = 5
    dijkstra(g, g.get_vertex(origin), g.get_vertex(target))

    target = g.get_vertex(target)
    path = [target.get_id()]
    shortest(target, path)
    print('The shortest path : %s' % (path[::-1]))
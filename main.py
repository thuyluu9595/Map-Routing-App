import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt
import turtle
import sys


class IndexMinPQ:
    """
        Priority queue class for sorting weighted Edge
    """
    def __init__(self, num_of_element=0):
        self.pq = []  # storage for queue
        self.qp = [False] * num_of_element  # to check if a key is in the queue

    def contains(self, key):
        if (key < len(self.qp)):
            return self.qp[key]  # O(1)
        else:
            print("No such vertex in the map!")
            return False

    def insert(self, key, priority):
        # insert to queue only when the key is not in the queue
        if key > len(self.qp):
            return False
        if not self.qp[key]:
            self.pq.append([key, priority])
            self.qp[key] = True
            self.heap_sort()  # O(logN)

    def delMin(self):
        if not self.isEmpty():
            min = self.pq[0][0]
            self.qp[self.pq[0][0]] = False
            self.pq.pop(0)
            self.heap_sort()
            return min

    def change(self, key, priority):
        # need a better algorithm to run faster!
        if self.qp[key]:
            for index in range(len(self.pq)):
                if self.pq[index][0] == key:
                    self.pq[index][1] = priority
                    break
            self.heap_sort()
        return False

    def isEmpty(self):
        return len(self.pq) == 0

    def sink(self, n, k):
        # look for the max between root and its children
        largest = k  # assume max is the root
        left = 2 * k + 1  # location of left child
        right = 2 * k + 2  # location of right child

        # looking for the real max, start from right child
        if right < n and self.pq[largest][1] < self.pq[right][1]:
            largest = right
        # looking for the real max, check the left child
        if left < n and self.pq[largest][1] < self.pq[left][1]:
            largest = left

        #  if largest is not the root, fix it!
        if largest != k:
            self.pq[k], self.pq[largest] = self.pq[largest], self.pq[k]
            self.sink(n, largest)  # recursion

    def heap_sort(self):
        """
        Heapsort is an improved selection sort: it divides its input into a sorted
        and an unsorted region, and it iteratively shrinks the unsorted region by
        extracting the largest element and moving that to the sorted region.
        """
        # Heap construction
        n = len(self.pq)

        # arrange the  list
        for i in range(n // 2, -1, -1):
            # from leaf to root
            self.sink(n, i)

        #
        for i in range(n - 1, 0, -1):
            # exchange last leaf (n-1) with the root ( 0 )
            self.pq[i], self.pq[0] = self.pq[0], self.pq[i]
            # sink the new root to the right position
            self.sink(i, 0)

        return self.pq


class VERTEX:
    """
       Vertex class
       A vertex has x, y coordination and it may have a connection with other vertex
    """

    def __init__(self, v_id=0, x_coord=0, y_coord=0):
        # initial a vertex with coordination x and y
        self.id = v_id  # id or index of this vertex
        self.x = int(x_coord)
        self.y = int(y_coord)
        self.neighbor = []  # list of vertices

    def vertex(self, x_coord, y_coord):
        # to change its coordination
        self.x = x_coord
        self.y = y_coord

    # to add connected vertices to this vertex
    # neighbors are other vertices which are connected to this vertex
    def add_neighbor(self, nb_vertex):
        if nb_vertex not in self.neighbor:
            self.neighbor.append(nb_vertex)
            return True

        return False

    def del_neighbor(self, nb_vertex):
        if nb_vertex in self.neighbor:
            self.neighbor.pop(nb_vertex)
            return True

        return False

    def tostring(self):
        return f"({self.x}, {self.y})"


# ---------------------------------------------------------------------------------------------------------
class EDGE:
    """
          The directed edge class
          A directed edge has 2 vertex, from-vertex and to-vertex
          The edge's weight compute from the distance between its vertices
    """

    def __init__(self, v_from=VERTEX(0, 0), v_to=VERTEX(0, 0)):
        # an edge draw from  vertex v_from to vertex v_to, default for all initial vertex is (0,0)
        self.v_from = v_from
        self.v_to = v_to
        self.edge_weight = self.weight()

    def weight(self):
        return math.sqrt(math.pow((self.v_to.x - self.v_from.x), 2) + math.pow((self.v_to.y - self.v_from.y), 2))

    def either(self):
        # return the from_vertex
        return self.v_from

    def other(self, v_from):
        # return the to_vertex
        if v_from == self.v_from:  # check if is the data is correct
            return self.v_to

        return False

    def toString(self):
        return f"({self.v_from.id}, {self.v_to.id})"
        #return f"from {self.v_from.tostring()}, to {self.v_to.tostring()}"


# ---------------------------------------------------------------------------------------------------------
class GRAPH:
    """
        The graph class
        A graph is the collection of vertices, and a collection of edges or adjacent
        The graph also gives the number of Vertices and edges inside it.
    """

    def __init__(self, int_v=0):
        self.num_of_vertices = int_v  # number of vertices
        self.num_of_edges = 0  # number of edges
        self.vertices = []  # the collection of vertices
        self.edges = []  # the collection of edges for graphing in turtle
        self.adj = []  # the edges that connected  between this vertex to other vertices

    def getdata(self, filename: any):
        # import data for this graph from a text file
        try:
            with open(filename) as inputs:
                lines = inputs.readlines()  # read line to line in the text file
        except IOError as e:
            print(f"Couldn't open  the {filename} file (%s)." % e)

        # get number of vertices and edges for the map from the first line
        self.num_of_vertices, self.num_of_edges = int(lines[0].split()[0]), int(lines[0].split()[1])

        # initial list of adjacent for each vertex
        for i in range(self.num_of_vertices):
            self.adj.append([])

        # collect all vertices' coordination
        for line in lines[1:int(self.num_of_vertices) + 1]:
            v_id, v_x, v_y = int(line.split()[0]), int(line.split()[1]), int(
                line.split()[2])  # format in each life:  name    x   y
            self.vertices.append((VERTEX(v_id, v_x, v_y)))

        # collect all edges' coordination,
        for line in lines[int(self.num_of_vertices) + 2:]:
            v_from, v_to = int(line.split()[0]), int(line.split()[1])
            # collect all edges
            self.edges.append(EDGE(self.vertices[v_from], self.vertices[v_to]))

            # get adjacent from each vertex
            self.adj[v_from].append(EDGE(self.vertices[v_from], self.vertices[v_to]))


class DIJKSTRASP:
    """
        The Dijkstra Shortest Path class
        Use dijkstra's algorithm to find the shortest path from 1 vertex to others in a weighted digraph
        the start_node is the name of the vertex that staring to find the distance

    """

    def __init__(self, graph=GRAPH(), start_point=0):
        self.edge_to = [None] * graph.num_of_vertices  # show the edge to the index-name vertex
        # self.edge_to[start_point] = None  # the start point / vertex has no root

        self.dist_to = [sys.maxsize] * graph.num_of_vertices  # set the distance from vertex v to other is infinity
        self.dist_to[start_point] = 0  # set starting distance from the source to zero

        self.pq = IndexMinPQ(graph.num_of_vertices)  # create priority queue
        self.pq.insert(start_point, 0.0)  # set priority queue to 0 for the start vertex

        while not self.pq.isEmpty():
            # keep relaxing the max edge until there is no edge to visit
            self.relax(graph, self.pq.delMin())

    def relax(self, graph, v_from):
        for current_edge in graph.adj[v_from]:
            w = current_edge.v_to.id  # get vertex's index of vertex_to

            # check if the distance from vertex V_from to its neighbors smaller than saved data in dist_to
            if self.dist_to[w] > self.dist_to[v_from] + current_edge.edge_weight:
                self.dist_to[w] = self.dist_to[v_from] + current_edge.edge_weight
                self.edge_to[w] = current_edge
                if self.pq.contains(w):
                    self.pq.change(w, self.dist_to[w])  # update v_to priority
                else:
                    self.pq.insert(w, self.dist_to[w])  # add v_to priority

    def distTo(self, v_to):
        if 0 <= v_to <= len(self.dist_to):
            return self.dist_to[v_to]
        return -1

    # check if there is a path from v_from vertex to the v_to vertecx
    def hasPathTo(self, v_to):
        if 0 <= self.dist_to[v_to] < sys.maxsize:
            return True
        return False

    def pathtTo(self, v_to):
        if 0 <= v_to <= len(self.dist_to):
            path = [v_to]
            if self.hasPathTo(v_to):
                path_edge = self.edge_to[v_to]
                while path_edge is not None:
                    path.append(path_edge.v_from.id)
                    path_edge = self.edge_to[path_edge.v_from.id]
                return path       # use path.pop() to get the path from start vertex to destination / v_to vertex
            else:
                return None
        else:
            return None


def draw_this_graph(G, STP, color="blue"):
    screen = turtle.Screen()
    # screen.bgcolor("black")
    # sets window to 100% of screen by 100% of screen and centers
    screen.setup(width=1.0, height=1.0, startx=0, starty=0)

    my_pen = turtle.Turtle()
    turtle.screensize(1500, 1500)
    my_pen.shape("circle")

    re_pos = 300
    my_pen.pensize(1)
    my_pen.speed(0)

    for edge in G.edges:
        my_pen.color("grey")
        my_pen.pu()
        my_pen.hideturtle()
        my_pen.goto((edge.v_from.x / 10) - re_pos, (edge.v_from.y / 10) - re_pos)
        my_pen.color("red")
        my_pen.write(edge.v_from.id, font=("arial", 15))
        my_pen.color("grey")

        my_pen.pd()
        my_pen.showturtle()
        my_pen.dot(15)
        my_pen.goto((edge.v_to.x / 10) - re_pos, (edge.v_to.y / 10) - re_pos)
        my_pen.dot(15)
        my_pen.color("red")
        my_pen.write(edge.v_to.id, font=("arial", 15))

    # draw shortest part
    if STP:
        point = STP.pop()
        # my_pen.write(G.vertices[point].id, font=("arial", 15))
        my_pen.color("blue")
        my_pen.pu()
        my_pen.goto((G.vertices[point].x / 10) - re_pos, (G.vertices[point].y / 10) - re_pos)
        my_pen.speed(1)
        my_pen.pd()
    while STP:
        my_pen.write(G.vertices[point].id, font=("arial", 15))
        point = STP.pop()  # get next vertex / point
        my_pen.goto((G.vertices[point].x / 10) - re_pos, (G.vertices[point].y / 10) - re_pos)

    # my_pen.write(G.vertices[point].id, font=("arial", 15))

    my_pen.hideturtle()
    turtle.done()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    file_name = "input6.txt"

    my_graph = GRAPH()
    my_graph.getdata(file_name)

    print(my_graph.num_of_vertices, my_graph.num_of_edges)

    shortest_part = DIJKSTRASP(my_graph, 0)

    stp = shortest_part.pathtTo(10)
    for e in my_graph.edges:
        print(e.toString())

    draw_this_graph(my_graph, stp, "blue")

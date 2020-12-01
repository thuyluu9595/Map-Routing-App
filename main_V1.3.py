import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt
import turtle
import sys

"""
Last update: 18:56, Nov 30 2020
- update some bugs
- create plotting, test running time, and samples generator functions
- update new main ()

by Hoang Tran

"""


class IndexMinPQ:
    """
        Priority queue class for sorting weighted Edge
    """

    def __init__(self, num_of_element=0):
        self.pq = []  # storage for queue
        self.qp = [False] * num_of_element  # to check if a key is in the queue

    def contains(self, key):
        # check whether the key is in the queue or not
        if key < len(self.qp):
            return self.qp[key]  # complexity O(1)
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
        # pop the min key out of the queue
        if not self.isEmpty():
            this_min = self.pq[0][0]
            self.qp[self.pq[0][0]] = False
            self.pq.pop(0)
            self.heap_sort()
            return this_min

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
        # return f"from {self.v_from.tostring()}, to {self.v_to.tostring()}"


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
        lines = []
        try:
            with open(filename) as inputs:
                lines = inputs.readlines()  # read line to line in the text file
        except IOError as e:
            print(f"\nCouldn't open  the {filename} file (%s)." % e)

        if lines:
            # get number of vertices and edges for the map from the first line
            self.num_of_vertices, self.num_of_edges = int(lines[0].split()[0]), int(lines[0].split()[1])

            # initial list of adjacent for each vertex
            for i in range(self.num_of_vertices):
                self.adj.append([])

            # collect all vertices' coordination
            for line in lines[1:int(self.num_of_vertices) + 1]:
                v_id, v_x, v_y = int(line.split()[0]), int(line.split()[1]), int(
                    line.split()[2])  # format in each life:  name    x   y
                self.vertices.append((VERTEX(v_id, int(v_x / 2), int(v_y / 2))))

            # collect all edges' coordination,
            for line in lines[int(self.num_of_vertices) + 2:self.num_of_edges + self.num_of_vertices + 2]:
                v_from, v_to = int(line.split()[0]), int(line.split()[1])
                # collect all edges
                self.edges.append(EDGE(self.vertices[v_from], self.vertices[v_to]))

                # get adjacent from each vertex
                self.adj[v_from].append(EDGE(self.vertices[v_from], self.vertices[v_to]))

            inputs.close()
            del lines
            return True
        else:
            print(f"Check your input file!!\n.\n.")
            return False


class DIJKSTRASP:
    """
        The Dijkstra Shortest Path class
        Use dijkstra's algorithm to find the shortest path from 1 vertex to others in a weighted digraph
        the start_node is the name of the vertex that staring to find the distance

    """

    def __init__(self, graph=GRAPH(), start_point=0):
        if start_point > graph.num_of_vertices:
            start_point = 0
        self.start_point = start_point
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
        # print("in relax: ", v_from, "\n")

    def distTo(self, v_to):
        if 0 <= v_to <= len(self.dist_to):
            return self.dist_to[v_to]
        return -1

    # check if there is a path from v_from vertex to the v_to vertex
    def hasPathTo(self, v_to):
        # if the weight to this vertex is not infinity, there is a path to this vertex from the start_point
        if v_to >= len(self.edge_to):
            print(f"Vertex {v_to} is not in this map!")
            return False
        if 0 <= self.dist_to[v_to] < sys.maxsize:
            return True
        return False

    def pathtTo(self, v_to):
        if 0 <= v_to <= len(self.dist_to):
            path = [v_to]  # trace back to the start_point
            # if the weight of the checking vertex is not infinity, we can trace the road back to the root
            if self.hasPathTo(v_to):
                path_edge = self.edge_to[v_to]
                while path_edge is not None:
                    path.append(path_edge.v_from.id)
                    path_edge = self.edge_to[path_edge.v_from.id]
                return path  # use path.pop() to get the path from start vertex to destination / v_to vertex
            else:
                # there is no road connect between this vertex and the start_point vertex
                return None
        else:
            return None


def draw_stp(G, STP, pen, re_pos=300, color="green"):
    turtle.tracer(True)
    pen.speed(0)
    pen.pensize(4)
    pen.color(color)
    pen.shape("circle")
    draw_path = "-1"

    if STP:
        pen.pu()
        point = STP.pop()
        draw_path = f"Path: {point}"
        pen.hideturtle()
        pen.setpos((G.vertices[point].x / 10) - re_pos, (G.vertices[point].y / 10) - re_pos)
        pen.speed(5)
        pen.pd()
    pen.showturtle()
    while STP:
        pen.dot(12)
        pen.write(G.vertices[point].id, font=("arial", 12))
        point = STP.pop()  # get next vertex / point
        draw_path = draw_path + f"=>{point}"
        pen.goto((G.vertices[point].x / 10) - re_pos, (G.vertices[point].y / 10) - re_pos)
    # mark the destination point.
    pen.color("blue")
    pen.dot(20)

    pen.speed(0)
    pen.hideturtle()
    pen.pu()
    pen.setpos(-730, 350)
    pen.color("red")
    pen.pd()
    if draw_path != "-1":
        pen.write(draw_path, align="left", font=("arial", 22))
    else:
        pen.write("No Path!", align="left", font=("arial", 22))
    turtle.tracer(False)


def draw_roads(G, pen, re_pos):
    # draw full graph
    turtle.tracer(False)
    pen.pensize(2)
    for edge in G.edges:
        pen.pu()
        pen.color("grey")
        pen.hideturtle()
        pen.setpos((edge.v_from.x / 10) - re_pos, (edge.v_from.y / 10) - re_pos)
        pen.color("red")
        pen.write(edge.v_from.id, font=("arial", 12))

        pen.pd()
        pen.color("grey")
        pen.showturtle()
        pen.dot(12)
        pen.goto((edge.v_to.x / 10) - re_pos, (edge.v_to.y / 10) - re_pos)
        pen.dot(12)
        pen.color("red")
        pen.write(edge.v_to.id, font=("arial", 12))
    turtle.tracer(True)


def draw_this_graph(G, shortest_part):
    screen = turtle.Screen()
    screen.bgcolor("LightGoldenrodYellow")
    # sets window to 100% of screen by 100% of screen and centers
    screen.setup(width=1.0, height=1.0, startx=0, starty=0)
    turtle.screensize(1500, 1500)
    turtle.title(f"Map Routing - Find The Shortest Part from Vertex  {shortest_part.start_point}")
    re_pos = 300

    my_pen = turtle.Turtle()

    # draw full graph
    draw_roads(G, my_pen, re_pos)
    root_x = G.vertices[shortest_part.start_point].x
    root_y = G.vertices[shortest_part.start_point].y

    # draw shortest part from start_point to another point
    while True:
        my_pen.hideturtle()
        my_pen.pu()
        my_pen.setpos((root_x / 10) - re_pos, (root_y / 10) - re_pos)
        my_pen.color("blue")
        my_pen.pd()
        my_pen.dot(20)
        point = screen.numinput("Shortest Path", "Choose the  destination", 36, minval=0,
                                maxval=G.num_of_vertices)
        if point is not None:
            point = int(point)
            turtle.Screen().reset()
            draw_roads(G, my_pen, re_pos)
            STP = shortest_part.pathtTo(point)
            draw_stp(G, STP, my_pen, re_pos, "green")
        else:
            turtle.Screen().reset()
            my_pen.setpos(0, 0)
            my_pen.color("brown")
            bye_mes = "San Jose State University\nCMPE 130, Fall 2020\n\n" \
                      "Map Routing with Dijkstras’s Algorithm\n" \
                      "Thao Ton - Thuy Luu - Hoang Tran\n\n" \
                      "Dr. Gokay Saldamli"
            my_pen.write(bye_mes, font=("arial", 25), align="center", )
            break
    my_pen.hideturtle()
    turtle.done()


def create_data(num_of_ver, num_of_ed):
    """
        Create data for test, store in text file,
        File name and file's format is based on the number of vertices and number of edges
    """
    # create a txt file: name = test + number of vertices
    file_test = f"test{num_of_ver}.txt"
    connected_edge = [0] * num_of_ed
    with open(file_test, 'w') as my_file:
        my_file.write(f"{num_of_ver}\t {num_of_ed}\n")
        # get random number for (x, y) coordination
        for index in range(num_of_ver):
            line = f"{index}\t {random.randint(1, 10000)}\t {random.randint(1, 10000)}\n"
            my_file.write(line)
        my_file.write("\n")

        # these initial connection to make sure that the starting location has some connection to test
        # if the starting location has no connect, time testing is zero....Not a good sample
        my_file.write("0 \t 1\n")  # this is make sure the root connect to something to test
        my_file.write("0 \t 2\n")  # this is make sure the root connect to something to test
        my_file.write("0 \t 3\n")  # this is make sure the root connect to something to test
        my_file.write("0 \t 4\n")  # this is make sure the root connect to something to test

        # connect 2 vertices to be an edge, each vertex has 2 connections maximum
        for index in range(num_of_ed - 4):
            v_f = random.randint(0, num_of_ver - 1)  # get random vertex from
            # v_t = random.randint(0, num_of_ver - 1)  # get random vertex from
            v_t = v_f + random.randint(0, 3)  # this make sure all vertices have some connections
            while connected_edge[v_f] > 2:  # get other vertex if this vertex has more than 3 connections
                v_f = random.randint(1, num_of_ver)
            while v_t >= num_of_ver or v_t == connected_edge[v_t]:
                v_t = v_f + random.randint(0, 3)
            line = f"{v_f}\t {v_t}\n"
            my_file.write(line)
        print(f"file {file_test} created!")


def create_samples(p=[]):
    """
        create  samples data files to test the Distrak's algorithms
        Range from 0 to p, number of edge is double number of vertices
    """
    if p:
        for v in p:
            if v >= 5:
                create_data(v, 2 * v)
            else:
                print("The number of vertices need to be greater than 4!")
                break


def check_running_time(num_of_run_time=1):
    # the list of number of vertices will be run in the running-time test
    sample_sizes = [5, 50, 500, 1000, 5000, 10000, 20000, 40000, 60000, 80000, 100000, 200000, 400000, 500000]
    timing_stp = []  # for y-axis of shortest part
    timing_load = []  # for y-axis of loading data from file

    for i, size in enumerate(sample_sizes):
        sample_name = f"test{size}.txt"
        print("Sample file: ",sample_name)
        load_max = 0
        get_max = 0
        # Run each size of sample in num_of_run_time time
        for run in range(num_of_run_time):
            print(f"Test map with: {size} vertices, and {2*size} edges.")
            create_data(size, 2*size)    # create fresh data samples for each test time
            this_graph = GRAPH()            # a fresh map
            # start loading all data in file
            t0 = time.time()
            if this_graph.getdata(sample_name):  # load sample data from file
                t1 = time.time()
                total_time = t1 - t0
                if total_time > load_max:       # get the longest run time
                    load_max = total_time

                t0 = time.time()
                stb = DIJKSTRASP(this_graph, 0)  # compute the shortest part from starting location to others
                t1 = time.time()
                total_time = t1 - t0
                if total_time > get_max:        # get the longest run time
                    get_max = total_time
                time.sleep(1.0)                 # maybe this can help release memory....

            # save  the longest results
        timing_load.append(load_max)
        timing_stp.append(get_max)
        print("---------------------------\n")

    print("Num of ver: ", this_graph.num_of_vertices)
    print("Num of ed: ", this_graph.num_of_edges)
    print("loading time: ", timing_load)
    print("Shortest path time: ", timing_stp)
    # for i, e in enumerate(this_graph.edges):        print(i, "\t", e.v_from.id, "\t", e.v_to.id)
    # this plots things in log scale (pls google it), you need to add matplotlib to your virtualenv first!

    # Plot the graphs
    plt.xlim(0, sample_sizes[-1] + 1000)
    plt.ylabel('Running time (seconds)')
    plt.xlabel('Number of points')
    plt.legend(loc="upper left")
    plt.figure("Load data for map graph"), plt.plot(sample_sizes, timing_load, 'b-', label='Load data')
    plt.figure("Dijkstra shortest part "), plt.plot(sample_sizes, timing_stp, 'r-', label='Shortest path ')
    plt.figure("Compare graphs"), plt.plot(sample_sizes, timing_stp, 'r-', label='Shortest path '), plt.plot(sample_sizes, timing_load, 'b-', label='Load data')

    plt.show()


# ===================================================================================================
if __name__ == '__main__':
    menu  = -1
    while menu != 1 and menu !=2:
        try:
            menu = input(f"\nEnter 1 for run DEMO, 2 for Running-Time Test: ")
            menu = int(menu)
        except ValueError:
            print("Error! This is not a number. Try again. ")

    if menu == 2:
        num_test = 3        # will test each size of sample 3 times
        check_running_time(num_test)

    if menu == 1:
        file_name = "input60_nondirmap.txt"
        # file_name = "input60_dirmap.txt"
        start_point = -1
        my_graph = GRAPH()
        if my_graph.getdata(file_name):
            print("Load data successful!")
            print("================================================================================")
            print(f"Read from the ma:\n "
                  f"\t - Number of vertices: {my_graph.num_of_vertices} \n"
                  f"\t - Number of edges: {my_graph.num_of_edges}")
            print("================================================================================")
            while 0 > start_point or start_point > my_graph.num_of_vertices:
                try:
                    get_num = input(f"Enter the starting location, from 0 to {my_graph.num_of_vertices - 1}: ")
                    start_point = int(get_num)
                except ValueError:
                    print("Error! This is not a number. Try again. ")

            print(f"Create the shortest path from vertex: {start_point}\n")
            shortest_path = DIJKSTRASP(my_graph, start_point)
            draw_this_graph(my_graph, shortest_path)
        else:
            print("Fail to initial data!\n")


# ================ end of main
"""

    # test usa file
    sample_name = "usa.txt"
    this_graph = GRAPH()
    # start loading all data in file
    print(sample_name, "\n")
    t0 = time.time()
    if this_graph.getdata(sample_name):
        t1 = time.time()
        total_time = t1 - t0
        timing_load.append(total_time)

        # start finding the shortest path
        t0 = time.time()
        stb = DIJKSTRASP(this_graph, 0)
        t1 = time.time()
        total_time = t1 - t0
        timing_stp.append(total_time)
"""


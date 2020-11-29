from main1 import *

class Test_Main(object):
    @classmethod
    def setup_class(klass):
        print("\n###        Start Functions Tests         ###\n")

    def test_one(self):
        pass

    def test_num_of_vertices_edges_1(self):
        file_name = "input6.txt"
        my_graph = GRAPH()
        my_graph.getdata(file_name)
        actual = (my_graph.num_of_vertices,my_graph.num_of_edges)
        expected = (6,9)
        assert expected == actual

    def test_num_of_vertices_edges_2(self):
        file_name = "Points.txt"
        my_graph = GRAPH()
        my_graph.getdata(file_name)
        actual = (my_graph.num_of_vertices,my_graph.num_of_edges)
        expected = (50,86)
        assert expected == actual

    def test_path_1(self):
        file_name = "input6.txt"
        my_graph = GRAPH()
        my_graph.getdata(file_name)
        shortest_part = DIJKSTRASP(my_graph, 0)
        stp = shortest_part.pathtTo(5)
        arr = []
        for i in range(len(stp)):
            arr.append(stp.pop())
        expected = [0,1,2,5]
        assert expected == arr

    def test_path_2(self):
        file_name = "Points.txt"
        my_graph = GRAPH()
        my_graph.getdata(file_name)
        shortest_part = DIJKSTRASP(my_graph, 26)
        stp = shortest_part.pathtTo(30)
        arr = []
        for i in range(len(stp)):
            arr.append(stp.pop())
        expected = [26,38,6,4,11,13,43,29,30]
        assert expected == arr

    def test_path_3(self):
        file_name = "Points.txt"
        my_graph = GRAPH()
        my_graph.getdata(file_name)
        shortest_part = DIJKSTRASP(my_graph, 4)
        stp = shortest_part.pathtTo(23)
        arr = []
        for i in range(len(stp)):
            arr.append(stp.pop())
        expected = [4,11,13,43,29,30,25,23]
        assert expected == arr
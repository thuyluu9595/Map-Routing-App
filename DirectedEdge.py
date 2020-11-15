class DirectedEdge:
    # Initialize a directed edge from vertex v to vertex w
    def __init__(self, v, w, weight):
        self.v = v
        self.w = w
        self.weight = weight

    # Return the tail vertex
    def fromm(self):
        return self.v

    # Return the head vertex
    def to(self):
        return self.w

    # Return the weight of the directed edge
    def weight(self):
        return self.weight

    # def toString(self):

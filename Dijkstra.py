import IndexMinPQ


class Dijkstra:
    # INFINITY

    def __init__(self, G, s):

        self.distTo = [None] * G.V()
        self.edgeTo = [None] * G.V()
        for v in range(G.V()):
            self.distTo[v] = -1
        self.distTo[s] = 0

        self.pq = IndexMinPQ()
        self.pq.insert(s, self.distTo[s])
        while self.pq.isEmpty() is False:
            v = self.pq.delMin()
            for e in G.adj():
                self.relax(e)

    def relax(self, e):
        v = e.fromm()
        w = e.to()
        if self.distTo[w] > self.distTo[v] + e.weight():
            self.distTo[w] = self.distTo[v] + e.weight()
            self.edgeTo[w] = e
            if self.pq.contains(w):
                self.pq.decreaseKey(w, self.distTo[w])
            else:
                self.pq.insert(w, self.distTo[w])

    def distTo(self, v):
        return self.distTo[v]

    # def hasPathTo(self, v):

    def check(self, G, s):
        for e in G.edges():
            if e.weight() < 0:
                print("negative edge weight detected")
                return False
        if self.distTo[s] is not 0 or self.edgeTo[s] is not None:
            print("distTo[s] and edgeTo[s] inconsistent")
            return False
        for v in range(G.V()):
            if v == s: continue
            if self.edgeTo[v] is None and self.distTo[v] is not -1:
                print("distTo[] and edgeTo[] inconsistent")
                return False

        for v in range(G.V()):
            for e in G.adj(v):
                w = e.to()
                if self.distTo[v] + e.weight() < self.distTo[w]:
                    print("edge is not relaxed")
                    return False
        for w in range(G.V()):
            if self.edgeTo[w] is None: continue
            e = self.edgeTo[w]
            v = e.fromm()
            if w is not e.to(): return False
            if self.distTo[v] + e.weight() != self.distTo[w]:
                print("edge on shorter  path not tight")
                return False
        return True

    #def validateVertex(self,v):



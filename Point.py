import math
class Point:
    def __init__(self):
        self.x = 0
        self.y = 0

    def init_point(self,x,y):
        self.x = x
        self.y = y

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def distanceTo(self, that):
        dx = self.x - that.getX()
        dy = self.y - that.getY()
        return math.sqrt(dx*dx + dy*dy)


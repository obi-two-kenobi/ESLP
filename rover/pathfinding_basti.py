import math
import numpy as np


## Class for obstacles
class Obstacle:
    def __init__(self, x:int, y:int, width:int, height:int):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.sides = []
        ## Genereate vertex matrix (vertexes are the outer points of the obstacle). 
        ## Width and height have to be choosen according to obstacle size + safety margin (e.g.35cm each)
        self.navMatrix = [(x-width/2, y-height/2), (x+width/2, y-height/2), (x+width/2, y+height/2), (x-width/2, y+height/2)]
        self.collisionMatrix = [(x-width/2 + 1, y-height/2 + 1), (x+width/2 -1 , y-height/2 +1), (x+width/2 -1, y+height/2 -1 ), (x-width/2 +1 , y+height/2 -1)]
        
        ## generate side vectors
        for i in range(0,4):
            self.sides.append(PathFinding.getVector(self,self.collisionMatrix[i], self.collisionMatrix[(i+1)%4]))
            # print(self.sides[i-1])

class PathFinding:
    ## Returns the two-dimensional vector from start to end
    def getVector(self, start, end):
        return (end[0]-start[0], end[1]-start[1])

    ## Returns the length of a vector
    def getVectorLength(self, vector):
        return math.sqrt(vector[0]**2 + vector[1]**2)

    ## Returns False if obstacles are in the way, True if there is a clear path
    def checkLineOfSight(self, vector, obstacle:Obstacle):
        ''' Checks if there is a clear path from start to end. Returns False if obstacles are in the way, True if there is a clear path '''
        ## check for intersection with each side of the obstacle
        for side in obstacle.sides:
            if self.checkIntersection(vector, side):
                print("Intersection detected" + str(side))
                return False
        return True

    ## check for intersection of two vectors
    def checkIntersection(self,vector1, vector2):
        ''' Checks if two vectors intersect. Returns True if they intersect, False if they don't '''
        xdiff = (vector1[0], -vector2[0])
        ydiff = (vector1[1], -vector2[1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return False
        else:
            return True 

    def findPath(self, start:tuple, end:tuple, obstacles:list):
        ''' Returns a path from start to end. If there is no clear path, the path will be around the obstacles '''
        path = []
        ## check if there is a clear path
        for obstacle in obstacles:
            if not self.checkLineOfSight(self.getVector(start, end), obstacle):
                print("obstacle in the way")
                ## if there is a clear path add only endpoint to path
                for index, vertex in enumerate(obstacle.collisionMatrix):
                    ## check if there is a clear path to each vertex of the obstacle
                    ## Here the checking does not work properly, since only one sie of the obstacle is checked.
                    ## NEEDS WORK
                    if self.checkIntersection(self.getVector(start,obstacle.navMatrix[index]), self.getVector(vertex, obstacle.collisionMatrix[(index+1)%4])) and self.checkIntersection(self.getVector(start,obstacle.navMatrix[index]), self.getVector(obstacle.collisionMatrix[(index-1)%4] ,vertex)):
                        print("clear path to vertex")
                        ## if there is a clear path to a vertex, add it to the path
                        path.append(obstacle.navMatrix[index])
                        ## set new start to the vertex
                        start = vertex
                        ## break loop
                        break
        path.append(end)
        return path


def main():
    ## create obstacles
    obstacle1 = Obstacle(3, 3, 4, 4)
    obstacles = [obstacle1]

    ## create pathfinding object
    pathfinding = PathFinding()

    ## find path
    path = pathfinding.findPath((0,0), (3,7), obstacles)

    ## print path
    print(path)

if __name__ == "__main__":
    main()


 
    

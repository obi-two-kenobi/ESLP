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
        ## The navMatrix is used to navigate around the obstacle. The collisionMatrix is used to check for collisions
        ## The navMatrix is the same as the collisionMatrix, but with a safety margin of 1cm
        ## 
        ## 3                    2
        ##  x-----------------x
        ##  |                 |
        ##  |                 | 
        ##  |                 | 
        ##  |                 |
        ##  x-----------------x 
        ## 0                    1

        self.navMatrix = [(x-width/2, y-height/2), (x+width/2, y-height/2), (x+width/2, y+height/2), (x-width/2, y+height/2)]
        self.collisionMatrix = [(x-width/2 + 1, y-height/2 + 1), (x+width/2 -1 , y-height/2 +1), (x+width/2 -1, y+height/2 -1 ), (x-width/2 +1 , y+height/2 -1)]
        
        ## generate side vectors
        for i in range(0,4):
            sideStraight = Straight(PathFinding.getVector(self,self.collisionMatrix[i], self.collisionMatrix[(i+1)%4]),self.collisionMatrix[i], PathFinding.getVectorLength(self, PathFinding.getVector(self,self.collisionMatrix[i], self.collisionMatrix[(i+1)%4])))
            self.sides.append(sideStraight)
            # print(self.sides[i-1])

class Straight:
    def __init__(self, vector, origin, length):
        self.vector = vector
        self.origin = origin
        self.length = length

class PathFinding:
    ## Returns the two-dimensional vector from start to end
    def getVector(self, start, end):
        return (end[0]-start[0], end[1]-start[1])

    ## Returns the length of a vector
    def getVectorLength(self, vector):
        return math.sqrt(vector[0]**2 + vector[1]**2)

    ## Returns False if obstacles are in the way, True if there is a clear path
    def checkLineOfSight(self, straight, obstacle:Obstacle):
        ''' Checks if there is a clear path from start to end. Returns False if obstacles are in the way, True if there is a clear path '''
        ## check for intersection with each side of the obstacle
        for side in obstacle.sides:
            if self.checkIntersection(straight, side)[0]:
                print("Intersection detected" + str(side))
                return False, side
        return True

    ## check for intersection of two vectors
    def checkIntersection(self, straight1:Straight, straight2:Straight):
        ''' Checks if two straights intersect. Returns True if they intersect, False if they don't '''
        ## calculate intesection point of the two straights
        a = np.array([[straight1.vector[0], straight2.vector[0]],[straight1.vector[1], straight2.vector[1]]])
        b = np.array([straight2.origin[0]-straight1.origin[0], straight2.origin[1]-straight1.origin[1]])
        intersection = np.linalg.solve(a,b)
        if intersection is None:
            return False, intersection
        ## check if intersection point is on both straights
        if intersection[0] >= 0 and intersection[0] <= straight1.length and intersection[1] >= 0 and intersection[1] <= straight2.length:
            print(intersection)
            return True, intersection
        else:
            return False, intersection




    def findPath(self, start:tuple, end:tuple, directLine:Straight, obstacles:list):
        ''' Returns a path from start to end. If there is no clear path, the path will be around the obstacles '''
        path = []
        ## check if there is a clear path
        for obstacle in obstacles:
            if not self.checkLineOfSight(directLine, obstacle)[0]:
                print("obstacle in the way, side:" + str(self.checkLineOfSight(directLine, obstacle)[1].origin))
                ## check start and end vertex of side for clear path:

                
                
                
                
                
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
    start = (0,0)
    target = (3,7)
    directConnection = Straight(PathFinding.getVector(PathFinding, start, target), start, PathFinding.getVectorLength(PathFinding, PathFinding.getVector(PathFinding, start, target)))

    ## create pathfinding object
    pathfinding = PathFinding()

    ## find path
    path = pathfinding.findPath((0,0), (3,7),directConnection, obstacles)

    ## print path
    print(path)

if __name__ == "__main__":
    main()


 
    

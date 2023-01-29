import math
import numpy as np


## Class for obstacles
class Obstacle:
    def __init__(self, x:int, y:int, width:int, height:int):
        ## Coordinates
        self.x = x
        self.y = y
        ## Size of obstacle (later refered to as "collision Matrix")
        self.width = width
        self.height = height
        self.sides = []
        ## Genereate vertex matrix (vertexes are the outer points of the obstacle). 
        ## Width and height have to be choosen according to obstacle size (e.g.35cm each)
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

        self.collisionMatrix = [(x-width/2, y-height/2), (x+width/2, y-height/2), (x+width/2, y+height/2), (x-width/2, y+height/2)]
        self.navMatrix = [(x-width/2 - 1, y-height/2 - 1), (x+width/2 +1 , y-height/2 -1), (x+width/2 +1, y+height/2 +1 ), (x-width/2 -1 , y+height/2 +1)]
        
        ## generate side vectors + origins
        for i in range(0,4):
            sideStraight = Straight(PathFinding.getVector(self,self.collisionMatrix[i], self.collisionMatrix[(i+1)%4]),self.collisionMatrix[i], PathFinding.getVectorLength(self, PathFinding.getVector(self,self.collisionMatrix[i], self.collisionMatrix[(i+1)%4])))
            self.sides.append(sideStraight)


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
    def hasClearPath(self, straight, obstacle:Obstacle):
        ''' Checks if there is a clear path from start to end. Returns False if obstacles are in the way, True if there is a clear path and the intersectionpoint of the path'''
        ## check for intersection with each side of the obstacle
        intersections = []
        for side in obstacle.sides:
            result = self.calculateIntersectionPoint(straight, side)
            if result[0]:
                distanceToIntersection = self.getVectorLength(self.getVector(straight.origin, result[1]))
                intersections.append((distanceToIntersection, result[1]))
        if len(intersections) == 0:
            return True, None
        else:
            return False, min(intersections)[1]

    ## calculate intesection point of the two straights
    def calculateIntersectionPoint(self, straight1:Straight, straight2:Straight):
        ''' Checks if two straights intersect. Returns True if they intersect, False if they don't + the point of intersection'''
        intersectionPoint = None

        ## transform the two straights into a linear equation system and solve it
        a = np.array([[straight1.vector[0], -1*straight2.vector[0]],[straight1.vector[1], -1*straight2.vector[1]]])
        b = np.array([straight2.origin[0]-straight1.origin[0], straight2.origin[1]-straight1.origin[1]])
        intersection = np.linalg.solve(a,b)

        ## check if the intersection point is on both straights
        if(np.allclose(np.dot(a, intersection), b)):
            intersectionPointX = intersection[0] * straight1.vector[0] + straight1.origin[0]
            intersectionPointY = intersection[0] * straight1.vector[1] + straight1.origin[1]
            intersectionPoint = (intersectionPointX, intersectionPointY)
            if intersection is None:
                return False, intersectionPoint
            ## check if the intersection point is in the range of both straights
            if intersection[0] >= 0 and intersection[0] <= straight1.length and intersection[1] >= 0 and intersection[1] <= straight2.length:
                return True, intersectionPoint
            else:
                return False, intersectionPoint
        else:
            return False, intersectionPoint

    ## actual pathfinding method
    def findPath(self, start:tuple, end:tuple, directLine:Straight, obstacles:list):
        ''' Returns a path from start to end. If there is no clear path, the path will be around the obstacles '''
        path = []
        ## check if there is a clear path by iterating through all obstacles
        for obstacle in obstacles:
            if not self.hasClearPath(directLine, obstacle)[0]:
                intersectionPoint = self.hasClearPath(directLine, obstacle)[1]
                print("obstacle in the way, intersection at location:" + str(self.hasClearPath(directLine, obstacle)[1]))
                vertexScores = []
                ## find the optimal vertex to navigate around the obstacle by comparing the distance from the start to the vertex and the distance from the vertex to the end
                for index, vertex in enumerate(obstacle.collisionMatrix):
                    if vertex[0] == intersectionPoint[0] or vertex[1] == intersectionPoint[1]:
                        if self.hasClearPath(Straight(self.getVector(obstacle.navMatrix[index], end),obstacle.navMatrix[index], self.getVectorLength(self.getVector(obstacle.navMatrix[index], end)) ), obstacle)[0]:
                            distanceFromOrigin = self.getVectorLength(self.getVector(start, obstacle.navMatrix[index]))
                            distanceToTarget = self.getVectorLength(self.getVector(obstacle.navMatrix[index], end))
                            vertexScores.append((distanceFromOrigin + distanceToTarget, index))
                bestVertex = min(vertexScores)[1]
                print("optimal vertex to navigate around obstacle is:" + str(obstacle.navMatrix[bestVertex]))

                ## add the optimal vertex to the path
                path.append(obstacle.navMatrix[bestVertex])

                ## update the direct line to the end
                start = obstacle.navMatrix[bestVertex]
                directLine = Straight(self.getVector(start, end), start, self.getVectorLength(self.getVector(start, end)))

        ## after all obstacles have been cleared, add the end to the path
        path.append(end)
        return path

def main():
    obstacle1 = Obstacle(3, 3, 2, 2)
    obstacle2 = Obstacle(6, 7.5, 1, 1)
    obstacles = [obstacle1, obstacle2]
    start = (0,0)
    target = (9,10)
    directConnection = Straight(PathFinding.getVector(PathFinding, start, target), start, PathFinding.getVectorLength(PathFinding, PathFinding.getVector(PathFinding, start, target)))

    ## create pathfinding object
    pathfinding = PathFinding()

    ## find path
    path = pathfinding.findPath(start, target,directConnection, obstacles)

    ##TODO: compare different path solutions and choose the shortest one

    ## print path
    print(path)

if __name__ == "__main__":
    main()


 
    

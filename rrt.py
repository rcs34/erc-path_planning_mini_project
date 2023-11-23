import shapely
from shapely.geometry import Point,Polygon,LineString
import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Node():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.children = []
        self.point = [self.x,self.y]

class RRT():
    def __init__(self,start,goal,Iterations,stepsize,obstacles):
        self.start= Node(start.x,start.y)
        self.goal = Node(goal.x,goal.y)
        self.iterations = Iterations
        self.d = stepsize
        self.nearestdist = 500
        self.nearestnode = None
        self.obstacles = obstacles
        self.nodes = [self.start]
        self.new_point = None
    

    def sample_random_point(self):
        self.sample_point = [random.uniform(0,100), random.uniform(0,100)]

    def add_child(self,xcoord,ycoord):
        newnode = Node(xcoord,ycoord)
        self.nearestnode.children.append(newnode)
        newnode.parent = self.nearestnode
        self.nodes.append(newnode)
        if self.distance([newnode.x, newnode.y], [self.goal.x, self.goal.y]) < self.d:
            self.goal.parent = newnode
            newnode.children.append(self.goal)
        ax.plot(np.array([self.nearestnode.x,newnode.x]),np.array([self.nearestnode.y,newnode.y]), color="blue", linewidth = 2, linestyle = 'dashed')
    
    def find_nearest_node(self,node):
        self.nearestdist = 500
        self.nearestnode = None

        stack = [node]

        while stack:
            current_node = stack.pop()

            x = self.distance([current_node.x, current_node.y], self.sample_point)

            if x <= self.nearestdist:
                self.nearestdist = x
                self.nearestnode = current_node

            stack.extend(current_node.children)

    def distance(self,point1,point2):
        dist = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        return dist
    
    def get_coord_of_new_node(self):
        offset = self.d * self.unit_vector(self.nearestnode)
        self.new_point = [self.nearestnode.x + offset[0], self.nearestnode.y + offset[1]]
        
    
    def unit_vector(self,p1):
        v = np.array([self.sample_point[0]- p1.x, self.sample_point[1] - p1.y])
        return v/np.linalg.norm(v)
    
    def in_obstacle(self):
        line = LineString([self.nearestnode.point,self.new_point])
        for obstacle in self.obstacles:
            if line.intersects(obstacle):
                return True
        return False


    def plan(self):
        goal_reached = False

        for i in range(self.iterations):
            self.sample_random_point()
            self.find_nearest_node(self.start)
            self.get_coord_of_new_node()

            if not self.in_obstacle():
                self.add_child(self.new_point[0],self.new_point[1])
                plt.scatter(self.new_point[0],self.new_point[1], color = 'blue')
                plt.pause(1)
                
            if self.goal.parent is not None:
                path = []
                current_node = self.goal
                while current_node is not self.start:
                    path.append([current_node.x, current_node.y])
                    current_node = current_node.parent
                path.append([self.start.x,self.start.y])
                path = np.array(path)
                path = path[::-1]
                ax.plot(path[:, 0], path[:, 1], color="red", linewidth=2)


fig, ax = plt.subplots()
plt.title("RRT Path planning")
spoint = Point(10,73)
gpoint = Point(84,22)
circle_spoint = plt.Circle((spoint.x,spoint.y),7,color = 'r',fill = False)
circle_gpoint = plt.Circle((gpoint.x,gpoint.y),7,color = 'g',fill = False)

obstacle1 = Polygon([(10,20),(20,20),(20,10)])
obstacle2 = Polygon([(87,65),(80,80),(80,95),(95,95),(95,80),])
obstacle3 = Polygon([(30,30),(30,60),(60,60),(60,30)])

obstacles = [obstacle1,obstacle2,obstacle3]

ax.plot(spoint.x, spoint.y, "ro")
ax.plot(gpoint.x, gpoint.y, "go")

for obstacle in obstacles:
    exterior_coords = list(zip(obstacle.exterior.xy[0], obstacle.exterior.xy[1]))
    ax.add_patch(plt.Polygon(exterior_coords, facecolor="black", edgecolor="black"))

ax.set_xlim([0,100])
ax.set_ylim([0,100])

rrt = RRT(spoint,gpoint,200,5,obstacles)
rrt.plan()

plt.show()



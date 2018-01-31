# CS 440 Fall 2017 Assignment 1
# Ryan Anastasia

import sys,copy
from collections import deque
import heapq

# 
# maze_file_name.txt search_type
#

num_args = len(sys.argv)
arg_list = []
for _ in range(num_args):
       arg_list.append(_) 

#DFS, BFS, Greedy, A*


"""
    Maze Solver Class



"""
class mazeSolver:
    # default base class constructor - 
    def __init__(self, filename):
        #init members before setting values
        self.maze = []  #2d array for maze parsing - allows for dealing w/ single characters + neighbors
        self.dots = []  #array of tuples, (x,y) coords of dots set while parsing maze
        self.found = False  #found current dot? might not need to be a member tbh
        self.visited = []   #current array of visited nodes
        self.expanded = []
        self.cost = []   #this is bad for multiple dots
        self.frontier = []  #will function as stack/queue etc for separate searches based on derived class add/remove methods
        self.steps = 0
        self.nodes = 0

        self.cur_pos = (0,0)    #not really necessary in python tbh..
        self.start_pos = (0,0)  #save for potential redo on non-optimal paths/algos that may differ
        
        #call necessary functions to parse maze, set values etc

        #self.read_in_maze(filename)  #set maze, dots, visited arrays
        #self.cur_pos = self.dots[0] #first (or only) dot
        self.read_in_maze(filename)
        
    def pickLowest(self,pos):
        pass
        #get cost of node
        #find lowest neighbor
        #return lowest neighbor?

    """
        Doesn't need to return steps, just mark each 

    """
    def tracePath(self,pos):    #from largest to 0, ignore infinity
        curr_pos = end_pos
        while self.level[curr_pos[0]][curr_pos[1]] != 0: #until reach start point again
            pass        #always take lowest path from current node

    def resetCost(self):   #mUST BE BEFORE RESET VISITED
        for pos in self.visited:
            self.level[pos[0]][pos[1]] = 100000

    def clearFrontier(self):
        while len(self.frontier) != 0:
            self.remove()

    def resetNodes(self):
        pass
        #char = False    #maybe?


    def pathFound(self,pos):    #clean up, prepare for next node
        self.resetLevel(pos)
        self.resetVistied(pos)
        self.clarFrontier()

    def findDistance(self,pos,end_pos): #manhattan distance between 2 points
        return abs(end_pos[0]-pos[0]) + abs(end_pos[1]-pos[1])

    def nextNode(self,pos):
        pass
    """
    read_in_maze(filename)
        Description:
            Read in and parse input maze line by line
            Sets maze, dots, cur_pos vars
            % - Wall
            P - Start Pos
            . - dots

        Arguments:



    """
    def read_in_maze(self, filename):
        with open(filename,'r') as f:
            for line_count,line in enumerate(f):          #iterate over lines of maze
                self.maze.append([])        #create new line, every time but first
                self.visited.append([])
                self.cost.append([])
                self.maze[line_count] = []           #next line of input to parse as a list of chars, creates 2nd dimension
                self.visited[line_count] = []   #this line sucks *******
                self.cost[line_count] = []
                for char_count,char in enumerate(line):   #iterate over chars of line
                    self.maze[line_count].append(char)
                    self.visited[line_count].append(False)
                    self.cost[line_count].append(100000) #infinity
                    if char == '%': #wall
                        pass
                    elif char == 'P':   #set start position
                        self.start_pos = (line_count,char_count)
                        self.add(self.start_pos) #not necessary if infinite loop during
                    elif char == '.':   #add new dot coords
                        self.dots.append((line_count,char_count))

    """

        Description: Return true if node is NOT a wall and NOT yet visited


        Arguments
            pos - node to check validity of; tuple (line,character)

    """
    def validNode(self,pos):
        if self.maze[pos[0]][pos[1]] != '%' and self.visited[pos[0]][pos[1]] == False: #wall
            return True
        return False

    """

        Description: mark node as visited

        Arguments:
            pos - node to mark visited; tuple (line,character)

    """
    def markVisited(self,pos):
        self.visited[pos[0]][pos[1]] = True #visited
        self.maze[pos[0]][pos[1]] = '*'     #mark on maze

    def setCost(self,pos,curr_cost):   #should probably also check own level?
        lowest = 10000000
        if self.cost[pos[0]-1][pos[1]+1] < lowest:
            lowest = self.cost[pos[0]][pos[1]+1] + 1

        if self.cost[pos[0]+1][pos[1]] < lowest:
            lowest = self.cost[pos[0]+1][pos[1]] + 1
        if self.cost[pos[0]][pos[1]-1] < lowest:
            lowest = self.cost[pos[0]][pos[1]-1] + 1

        if self.cost[pos[0]-1][pos[1]] < lowest:
            lowest = self.cost[pos[0]-1][pos[1]] +1
        if lowest < curr_cost:
            return lowest   #if this ever returns infinity...??
        return curr_cost
        
    """

        Description: Add neighbors to frontier in order Right, Top, Left, Bottom

        Arguments: 
            pos - node to add neighbors of; tuple (line,character)

    """
    def addNeighbors(self, pos):    #might be obsolete if recursive DFS
        self.add((pos[0]+1,pos[1])) #bottom
        self.add((pos[0],pos[1]-1)) #left
        self.add((pos[0],pos[1]+1)) #right
        self.add((pos[0]-1,pos[1])) #top

    """
        Description: Mark current dot as found
    """
    def markFound(self):
        self.found = True

    """
        Description:
            Given tuple, will find path from frontier to coordinate

        Arguments:
            curr_pos - expanded node
            end_pos - tuple coord of current dot
            visited - array of tuples of nodes visited

    """
    def findPath(self,end_pos):
        path_cost = 0
        while(1):       #loop until dot is found, should always be found
            curr_pos = self.remove()    #take first element off
        
            if self.validNode(curr_pos): #have not been to node yet, not a wall
                #self.cost[curr_pos[0]][curr_pos[1]] = path_cost
                self.nodes += 1    #expanded node, includes end
                self.markVisited(curr_pos)      #mark visited
                self.expanded.append(curr_pos)  #keep coords of visited nodes
                self.cost[curr_pos[0]][curr_pos[1]] = self.setCost(curr_pos,path_cost)         #set level
                #BAD

                if curr_pos == end_pos:    #if done
                    #done, generate path cost? or just return
                    #self.tracePath(curr_pos)    #
                    #self.pathFound()
                    #self.cost = self.level[end_pos[0]][end_pos[1]]  #this is kind of a hack
                    print("Found Path! Dot at {} found. {} nodes expanded".format(end_pos,self.nodes))
                    print("Path Cost: {}".format(self.cost[end_pos[0]][end_pos[1]]))
                    #print("Solution takes {} steps".format(self.cost))
                    return

                self.addNeighbors(curr_pos) #add neighbors to frontier, only if not done to save one single case
                path_cost += 1

    """
    solveMaze()
        Description:
            Wrapper function for solving multiple dot search case
            Only iterates once for single dot mazes
            Calls findPath helper method on each dot
    """
    def solveMaze(self):
        print("Solve Maze!")
        print("Start at {}".format(self.start_pos))


        for dot in self.dots:
            print("End at {}".format(dot))
            self.findPath(dot)


"""
    Depth-First Search Solver Class



"""
class DFSSolver(mazeSolver):
    """
    def __init__(self):
        pass
        #super().__init__()  #calls base class constructor,
        #^^ is that necessary? i don't think so
    """

    def add(self, pos):
        self.frontier.append(pos)

    def remove(self):
        return self.frontier.pop()

"""
    Breadth-First Search

        Need queue for frontier, so separate constructor required.
        Maybe do separate DFS ctor also

"""
class BFSSolver(mazeSolver):
    def __init__(self,filename):
        super().__init__(filename)
        self.frontier = deque()
        self.add(self.start_pos)

    def add(self,pos):  
        self.frontier.append(pos)

    def remove(self):
        return self.frontier.popleft()

"""
    Greedy-Search

"""
class greedySolver(mazeSolver):
    def add(self,pos):  #will add to queue based on distance
        if self.maze[pos[0]][pos[1]] != "%":    #totally a hack, but don't want to add any walls because i'm bad at things
            heapq.heappush(self.frontier,(self.findDistance(pos,self.dots[0]),pos)) #bad

    def remove(self):   #should always remove element with lowest distance in unvisited queue
        pos = heapq.heappop(self.frontier)
        print(pos)
        return pos[1]

"""
    A*-Search

"""
class starSolver(mazeSolver):
    def add(self,pos):
        pass

    def remove(self):
        pass


def main():

    #maze = mazeSolver(sys.argv[1])    #new maze instance, filename from arguments
    #maze = DFSSolver(sys.argv[1])    #new maze instance, filename from arguments
    #maze = BFSSolver(sys.argv[1])    #new maze instance, filename from arguments
    maze = greedySolver(sys.argv[1])
    
    maze.solveMaze()
    #print(maze.findDistance(maze.start_pos,maze.dots[0]))

    #print("Nodes Expanded: {}".format(maze.nodes))
    #print("Path Cost: {}".format(maze.steps))


    #write to file
    with open("1.1bfs.txt","w") as f:
        for line in maze.maze:
            temp = str.join('',line)
            f.write(temp)

    #dfs_maze = 
    #bfs_maze = 
    #greedy_maze =
    #a_star_maze = 

if __name__ == "__main__":
    main()






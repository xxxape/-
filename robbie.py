import argparse as ap

import re

import platform



######## RUNNING THE CODE ####################################################

#   You can run this code from terminal by executing the following command

#   python planpath.py <INPUT/input#.txt> <OUTPUT/output#.txt> <flag>

#   for example: python planpath.py INPUT\input1.txt OUTPUT\output1.txt 0

#   NOTE: THIS IS JUST ONE EXAMPLE INPUT DATA

###############################################################################





################## YOUR CODE GOES HERE ########################################
#   Created by Zixuan Zhu 

class robbie():
    
    def __init__(self, map):
        # get map information
        self.N = int(map[0].replace('\n', ''))
        self.map = map[1:]
        for i in range(self.N):
            self.map[i] = self.map[i].replace('\n', '')
        # define operator list, it stores all the operators
        self.operators = ['U', 'L', 'R', 'D', 
                          'LU', 'RU', 'LD', 'RD']
        # define cost list, it stores costs of all operators
        self.cost = [2, 2, 2, 2,
                     1, 1, 1, 1]
        # edge decides angle
        self.relation = ((4, 5), (4, 6), (5, 7), (6, 7))
        # define open and closed list, they store the id and position of the nodes
        self.openList = []
        self.closedList = []
        # define countNode, it show the total nodes of the current state
        self.countNode = 1
        # define expansion-order
        self.expansion_order = 1
        # find goal node flag
        self.findGoal = False
        # find startNode and goalNode
        self.startX, self.startY = self.findSGNodePos('S')
        self.goalX, self.goalY = self.findSGNodePos('G')
        self.startNode = mapNode('N0', 'S', 0, self.startX, self.startY, self.goalX, self.goalY, None)
        self.goalNode = mapNode(None, 'G', 0, self.goalX, self.goalY, self.goalX, self.goalY, None)
        # caculate the h and f of startNode
        self.startNode.h = 2 * (self.startNode.dx + self.startNode.dy) - 3 * self.startNode.min_dx_dy(self.startNode.dx, self.startNode.dy)
        self.startNode.f = self.startNode.h
        self.startNode.operators = 'S'
        
    # find start and goal node    
    def findSGNodePos(self, node):
        for i in range(self.N):
            for j in range(self.N):
                if self.map[i][j] == node:
                    x = i
                    y = j
        return x, y
    
    # check node is in openList or not by x and y
    # if in return index, else return -1
    def inOpenlist(self, node):
        for i in range(len(self.openList)):
            if self.openList[i].x == node.x and self.openList[i].y == node.y:
                return i
        return -1
    
    # check node is in closedList or not by x and y
    
    # if in return index, else return -1
    def inClosedlist(self, node):
        for i in range(len(self.closedList)):
            if self.closedList[i].x == node.x and self.closedList[i].y == node.y:
                return i
        return -1
    
    def findFInList(self, elem):
        return elem.f
    
    def findGInList(self, elem):
        return elem.g
    
    def countOperators(self, elem):
        return len(elem.operators.split('-'))
    
    def sortOpenList(self):
        # sort openList
        # tie-rule: f > g > number_of_operators
        if len(self.openList) > 1:
            self.openList.sort(key=self.countOperators)
            self.openList.sort(key=self.findGInList)            
            self.openList.sort(key=self.findFInList)
            
    # main loop
    def findSolution(self, flag):
        self.openList.append(self.startNode)
        while (not self.findGoal):
            # check goal is in the openList or not
            # if in, find goal and finish
            if self.inOpenlist(self.goalNode) != -1:
                self.findGoal = True
                # output solution
                goalNodeFound = self.openList[self.inOpenlist(self.goalNode)]
                return self.getSolution(goalNodeFound)
                        
            # if openList is not empty, expand the node
            elif self.openList:
                # sort and get the min f in openList
                self.sortOpenList()       
                # expand the best node, take it from openList and put into closedList
                self.openList[0].order = self.expansion_order
                self.expansion_order += 1
                current_node = self.openList[0]
                self.openList.pop(0)
                self.closedList.append(current_node)
                # find children nodes
                # 8 surronding node
                # 1-4: 4 edges
                # 5-8: 4 angles
                tempChildNode = [(current_node.x-1, current_node.y),   
                                 (current_node.x,   current_node.y-1), 
                                 (current_node.x,   current_node.y+1), 
                                 (current_node.x+1, current_node.y),
                                 (current_node.x-1, current_node.y-1), 
                                 (current_node.x-1, current_node.y+1), 
                                 (current_node.x+1, current_node.y-1), 
                                 (current_node.x+1, current_node.y+1)]
                # children node list
                childrenList = []
                # check 4 edges first
                for i in range(4):
                    # not over boundary and not none
                    if (tempChildNode[i] is not None and tempChildNode[i][0] >= 0
                        and tempChildNode[i][1] >= 0 and tempChildNode[i][0] < self.N 
                        and tempChildNode[i][1] < self.N):
                        # if is mountain node, set the node and both sides nodes as None
                        if self.map[tempChildNode[i][0]][tempChildNode[i][1]] == 'X':
                            tempChildNode[i] = None
                            tempChildNode[self.relation[i][0]] = None
                            tempChildNode[self.relation[i][1]] = None
                        # if not mountain node, set the node as childrenNode and evaluate it
                        else:
                            tempNode = mapNode(None, self.operators[i], self.cost[i], 
                                               tempChildNode[i][0], tempChildNode[i][1],
                                               self.goalX, self.goalY, current_node)
                            tempNode.evaluate(self.cost[i])
                            # if in openList
                            if self.inOpenlist(tempNode) != -1:
                                # set nodeID as existed ID
                                tempNode.nodeID = self.openList[self.inOpenlist(tempNode)].nodeID
                                # compare two nodes'f and use the smaller one
                                if tempNode.f < self.openList[self.inOpenlist(tempNode)].f:
                                    self.openList[self.inOpenlist(tempNode)] = tempNode
                                #childrenList.append(self.openList[self.inOpenlist(tempNode)])
                                childrenList.append(tempNode)
                            elif self.inClosedlist(tempNode) != -1:
                                tempChildNode[i] = None
                            # not in open and closed list
                            else:
                                tempNode.nodeID = 'N' + str(self.countNode)
                                self.countNode = self.countNode + 1
                                self.openList.append(tempNode)
                                childrenList.append(tempNode)
                    else:
                        tempChildNode[i] = None
                # check 4 angles
                for i in range(4, 8):
                    if (tempChildNode[i] is not None and tempChildNode[i][0] >= 0
                        and tempChildNode[i][1] >= 0 and tempChildNode[i][0] < self.N 
                        and tempChildNode[i][1] < self.N):
                        if self.map[tempChildNode[i][0]][tempChildNode[i][1]] == 'X':
                            tempChildNode[i] = None
                        else:
                            tempNode = mapNode(None, self.operators[i], self.cost[i], 
                                               tempChildNode[i][0], tempChildNode[i][1],
                                               self.goalX, self.goalY, current_node)
                            tempNode.evaluate(self.cost[i])
                            # if in openList
                            if self.inOpenlist(tempNode) != -1:
                                # set nodeID as existed ID
                                tempNode.nodeID = self.openList[self.inOpenlist(tempNode)].nodeID
                                # compare two nodes'f and use the smaller one
                                if tempNode.f < self.openList[self.inOpenlist(tempNode)].f:
                                    self.openList[self.inOpenlist(tempNode)] = tempNode
                                #childrenList.append(self.openList[self.inOpenlist(tempNode)])
                                childrenList.append(tempNode)
                            elif self.inClosedlist(tempNode) != -1:
                                tempChildNode[i] = None
                            # not in open and closed list
                            else:
                                self.countNode = self.countNode + 1
                                tempNode.nodeID = 'N' + str(self.countNode)
                                self.openList.append(tempNode)
                                childrenList.append(tempNode)
                    else:
                        tempChildNode[i] = None    
                # if diagnostic mode, print details
                if flag >= 1:
                    self.printOutput(current_node, childrenList)
            
            # if openList is empty, there is no solution for the map
            else:
                print('NO-PATH')
                return 'NO-PATH'
                break

                
    # diagnostic mode print function
    def printOutput(self, current_node, childrenList):
        # print current node
        current_node.printNodeInfo(1)
        print()
        # print children
        if childrenList:
            print('Children: {', end = '')
            childrenList[0].printNodeInfo(4)
            for node in childrenList[1:]:
                print(', ', end = '')
                node.printNodeInfo(4)
            print('}')
        else:
            print('Children: {}')
        # print openList
        if self.openList:
            print('OPEN: {(', end = '')
            self.openList[0].printNodeInfo(3)
            for node in self.openList[1:]:
                print('), (', end = '')
                node.printNodeInfo(3)
            print(')}')
        else:
            print('OPEN: {}')
        # print closedList
        if self.closedList:
            print('CLOSED: {(', end = '')
            self.closedList[0].printNodeInfo(2)
            for node in self.closedList[1:]:
                print('), (', end = '')
                node.printNodeInfo(2)
            print(')}')
        else:
            print('CLOSED: {}')
        # print a blank line
        print()
        
    # get solution node
    def getSolution(self, goal):
        solutionList = []
        solution = ''
        solutionList.append(goal.parentNode)
        # get all the nodes
        while solutionList[0].nodeID != 'N0':
            solutionList.insert(0, solutionList[0].parentNode)
        for node in solutionList:
            solMap = self.map
            # print map
            for i in range(self.N):
                if i == node.x:
                    solution = solution + solMap[i][:node.y] + '*' + solMap[i][node.y+1:]
                    solution = solution + '\n'
                else:
                    solution = solution + solMap[i]
                    solution = solution + '\n'
            solution = solution + '\n'
            # print operators
            solution = solution + node.operators + ' ' + str(node.g)
            solution = solution + '\n'
            solution = solution + '\n'
        # print the final map
        for i in range(self.N):
            solution = solution + self.map[i]
            solution = solution + '\n'
        # print the last operator
        finalOperator = goal.operators + '-G '
        solution = solution + finalOperator + str(goal.g)
        return solution
        

# store node information
class mapNode():
    
    def __init__(self, nodeID, operator, cost, x, y, goalX, goalY, parentNode):
        self.nodeID = nodeID
        self.operator = operator
        self.cost = cost
        self.x = x
        self.y = y
        self.goalX = goalX
        self.goalY = goalY
        self.dx = abs(self.x - self.goalX)
        self.dy = abs(self.y - self.goalY)
        self.parentNode = parentNode
        self.g = 0
        self.h = 0
        self.f = 0
        self.order = 0
        self.operators = ''
        
    def evaluate(self, cost):
        self.operators = self.parentNode.operators + '-' + self.operator
        self.g = self.parentNode.g + cost
        # Chebyshev distance
        self.h = 2 * (self.dx + self.dy) -3 * self.min_dx_dy(self.dx, self.dy)
        self.f = self.g + self.h

    def min_dx_dy(self, dx, dy):
        if dx < dy:
            return dx
        return dy
    
    # print node information with order
    # flag: 1: print all operators with order
    #       2: print single operator with order
    #       3: print without order
    #       4: print only id and operators
    def printNodeInfo(self, flag):
        if flag == 1:
            print(self.nodeID + ':' + self.operators, self.order, self.g, self.h, self.f, end = '')
        elif flag == 2:
            print(self.nodeID + ':' + self.operator, self.order, self.g, self.h, self.f, end = '')
        elif flag == 3:
            print(self.nodeID + ':' + self.operators, self.g, self.h, self.f, end = '')
        elif flag == 4:
            print(self.nodeID + ':' + self.operators, end = '')


def graphsearch(map, flag):

    #solution = "S-R-RD-D-D-LD-G"
    
    # WRITE YOUR CODE HERE 
    solution = robbie(map).findSolution(flag)

    return solution



def read_from_file(file_name):

    # You can change the file reading function to suit the way

    # you want to parse the file

    file_handle = open(file_name)

    map = file_handle.readlines()

    return map




###############################################################################

########### DO NOT CHANGE ANYTHING BELOW ######################################

###############################################################################



def write_to_file(file_name, solution):

    file_handle = open(file_name, 'w')

    file_handle.write(solution)



def main():

    # create a parser object

    parser = ap.ArgumentParser()



    # specify what arguments will be coming from the terminal/commandline

    parser.add_argument("input_file_name", help="specifies the name of the input file", type=str)

    parser.add_argument("output_file_name", help="specifies the name of the output file", type=str)

    parser.add_argument("flag", help="specifies the number of steps that should be printed", type=int)

    # parser.add_argument("procedure_name", help="specifies the type of algorithm to be applied, can be D, A", type=str)





    # get all the arguments

    arguments = parser.parse_args()



##############################################################################

# these print statements are here to check if the arguments are correct.

#    print("The input_file_name is " + arguments.input_file_name)

#    print("The output_file_name is " + arguments.output_file_name)

#    print("The flag is " + str(arguments.flag))

#    print("The procedure_name is " + arguments.procedure_name)

##############################################################################



    # Extract the required arguments



    operating_system = platform.system()



    if operating_system == "Windows":

        input_file_name = arguments.input_file_name

        input_tokens = input_file_name.split("\\")

        if not re.match(r"(INPUT\\input)(\d)(.txt)", input_file_name):

            print("Error: input path should be of the format INPUT\input#.txt")

            return -1



        output_file_name = arguments.output_file_name

        output_tokens = output_file_name.split("\\")

        if not re.match(r"(OUTPUT\\output)(\d)(.txt)", output_file_name):

            print("Error: output path should be of the format OUTPUT\output#.txt")

            return -1

    else:

        input_file_name = arguments.input_file_name

        input_tokens = input_file_name.split("/")

        if not re.match(r"(INPUT/input)(\d)(.txt)", input_file_name):

            print("Error: input path should be of the format INPUT/input#.txt")

            return -1



        output_file_name = arguments.output_file_name

        output_tokens = output_file_name.split("/")

        if not re.match(r"(OUTPUT/output)(\d)(.txt)", output_file_name):

            print("Error: output path should be of the format OUTPUT/output#.txt")

            return -1



    flag = arguments.flag

    # procedure_name = arguments.procedure_name





    try:

        map = read_from_file(input_tokens[1]) # get the map

    except FileNotFoundError:

        print("input file is not present")

        return -1

    # print(map)

    

    solution_string = "" # contains solution



    solution_string = graphsearch(map, flag)

    write_flag = 1

    

    # call function write to file only in case we have a solution

    if write_flag == 1:

        write_to_file(output_tokens[1], solution_string)



if __name__ == "__main__":

    main()


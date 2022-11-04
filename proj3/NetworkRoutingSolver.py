#!/usr/bin/python3


from dis import dis
from math import dist
from re import L
from CS312Graph import *
import time

class NetworkRoutingSolver:
    def __init__(self):
        self.distances = None
        self.previous = None
        self.source = None
        self.destination = None
        self.network = None

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def getShortestPath(self, destIndex):
        self.destination = destIndex
        edges = []
        length = self.distances[self.destination]
        index = self.destination

        # Checking it the node is unreachable
        if self.previous[destIndex] is None:
            return {'cost': float('inf'), 'path': edges}

        # Finding all the edges
        while self.previous[index] is not None:
            prevNode = self.network.nodes[self.previous[index]]
            for edge in prevNode.neighbors:
                if edge.dest.node_id == index:
                    edges.append((edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)))
            index = self.previous[index]

        return {'cost': length, 'path': edges}

    def computeShortestPaths(self, srcIndex, use_heap=False):
        t1 = time.time()
        
        self.source = srcIndex

        # Choosing either heap or array implementaion
        if use_heap:
            PQ = HeapPQ()
        else:
            PQ = ArrayPQ()

        self.distances, self.previous = self.dijkstra(PQ)

        t2 = time.time()
        return t2 - t1

    def dijkstra(self, PQ):
        distances = []
        previous = []

        for i in range(len(self.network.nodes)):
            distances.insert(i, float('inf'))
            previous.insert(i, None)
        PQ.makeQueue(distances)
        distances[self.source] = 0
        PQ.decreaseKey(self.source, 0)

        # Looping until the queue is empty
        while not PQ.isEmpty():
            # Getting the minimum node from the queue
            minIndex = PQ.deleteMin(distances)

            # Checking if the node is unreachable
            if distances[minIndex] == float('inf'):
                break

            # Updating distances
            for edge in self.network.nodes[minIndex].neighbors:
                newDistance = distances[minIndex] + edge.length
                if distances[edge.dest.node_id] == float('inf') or newDistance < distances[edge.dest.node_id]:
                    distances[edge.dest.node_id] = newDistance
                    previous[edge.dest.node_id] = minIndex
                    PQ.decreaseKey(edge.dest.node_id, newDistance)      
        
        return distances, previous

# Array priority queue implementaion class
class ArrayPQ:
    def __init__(self):
        self.set = set()

    def makeQueue(self, distances):
        for i in range(len(distances)):
            self.insertNode(i)

    # Inserting a node into the array
    def insertNode(self, index):                                                            # Total Time: O(1), Total Space: O(1)
        self.set.add(index)                                                                 # Time: O(1), Space; O(1)

    # Decreasing the distance
    def decreaseKey(self, node, distance):                                                  # Total Time: O(1), Total Space: O(1)
        pass

    # Removing the smallest distance node from the list and returning it
    def deleteMin(self, distance):                                                          # Total Time: O(|V|), Total Space: O(1)
        # Getting first node from the set
        minIndex = next(iter(self.set))
        # Getting the minimum from the set
        for num in self.set:                                                                # Time: O(|V|), Space; O(1)
            if distance[minIndex] is float('inf') or distance[num] < distance[minIndex]:    # Time: O(1), Space; O(1)
                minIndex = num
        # Removing the minimum from the set
        self.set.remove(minIndex)                                                           # Time: O(1), Space; O(1)
        return minIndex                                                                     # Time: O(1), Space; O(1)

    # Checking if the tree is empty
    def isEmpty(self):
        if len(self.set) == 0:
            return True
        return False

# Heap priority queue implementation class
class HeapPQ:
    def __init__(self):
        self.tree = []
        self.pointers = []

    def makeQueue(self, distances):
        for i in range(len(distances)):
            self.insertNode(i)

    # Inserting a node into the heap
    def insertNode(self, node_id):                                                          # Total Time: O(1), Total Space: O(1)
        self.pointers.append(len(self.tree))                                                # Time: O(log|V|), Space: O(1)
        self.tree.append((node_id, float('inf')))                                           # Time: O(1), Space: O(1)

    # Decreasing the distance
    def decreaseKey(self, node, distance):                                                  # Total Time: O(log|V|), Total Space: O(1)
        self.tree[self.pointers[node]] = (node, distance)                                   # Time: O(1), Space: O(1)
        self.bubbleUp(node)                                                                 # Time: O(log|V|), Space: O(1)

    # Removing the smallest distance node from the list and returning it
    def deleteMin(self, dist):                                                              # Total Time: O(log|V|), Total Space: O(1)
        # Checking if there is only one node left in the tree
        if len(self.tree) == 1:                                                             # Time: O(1), Space: O(1)    
            return self.tree.pop()[0]                                                       # Time: O(1), Space: O(1)

        # Getting the top node and removing it
        topNode = self.tree[0][0]
        self.pointers[topNode] = None

        # Moving the last node to the top
        self.tree[0] = self.tree[-1]
        self.tree.pop()

        # Bubbling the top node down
        bottomNode = self.tree[0][0]
        self.pointers[bottomNode] = 0
        self.bubbleDown(bottomNode)                                                         # Time: O(log|V|), Space: O(1)

        return topNode                                                                      # Time: O(1), Space: O(1)

    # Bubbling a value up in the tree
    def bubbleUp(self, node):                                                               # Total Time: O(1), Total Space: O(1)
        currNode = node
        while True:                                                                         # Time: O(log|V|), Space: O(1)
            currLocation = self.pointers[currNode]
            # Checking to see if the current location is the top
            if currLocation == 0:                                                           # Time: O(1), Space: O(1)
                break

            # Calculating the parent node
            parentLocation = (currLocation - 1) // 2                                        # Time: O(1), Space: O(1)
            currNode, currValue = self.tree[currLocation]
            parent, parentValue = self.tree[parentLocation]
            
            # Checking if the current value is greater than or equal to the parent value
            if currValue >= parentValue:                                                    # Time: O(1), Space: O(1)
                break

            # Swapping the current node and the parent node
            self.tree[parentLocation] = self.tree[currLocation]
            self.pointers[currNode] = parentLocation
            self.tree[currLocation] = (parent, parentValue)
            self.pointers[parent] = currLocation

    # Bubbling a value down in the tree
    def bubbleDown(self, node):                                                             # Total Time: O(log|V|), Total Space: O(1)
        currNode = node
        while True:                                                                         # Time: O(log|V|), Space: O(1)
            # Checking if there are children
            currLocation = self.pointers[currNode]
            firstChildLocation = round((currLocation + 0.5) * 2)
            secondChildLocation = (currLocation + 1) * 2
            currNode, cur_value = self.tree[currLocation]

            maxLocation = len(self.tree) - 1
            if firstChildLocation > maxLocation and secondChildLocation > maxLocation:      # Time: O(1), Space: O(1)
                break

            if firstChildLocation > maxLocation:                                            # Time: O(1), Space: O(1)
                firstChildValue = float('inf')
                secondChild, secondChildValue = self.tree[secondChildLocation]
            elif secondChildLocation > maxLocation:                                         # Time: O(1), Space: O(1)
                firstChild, firstChildValue = self.tree[firstChildLocation]
                secondChildValue = float('inf')
            else:                                                                           # Time: O(1), Space: O(1)
                firstChild, firstChildValue = self.tree[firstChildLocation]
                secondChild, secondChildValue = self.tree[secondChildLocation]

            # Creating the correct child node
            if firstChildValue <= secondChildValue:                                         # Time: O(1), Space: O(1)
                child = firstChild
                childValue = firstChildValue
                childLocation = firstChildLocation
            else:                                                                           # Time: O(1), Space: O(1)
                child = secondChild
                childValue = secondChildValue
                childLocation = secondChildLocation

            # Checking if the current value is less than or equal to the parent value
            if cur_value <= childValue:                                                     # Time: O(1), Space: O(1)
                break
            
            # Swapping the current node and the child node
            self.tree[childLocation] = self.tree[currLocation]
            self.pointers[currNode] = childLocation
            self.tree[currLocation] = (child, childValue)
            self.pointers[child] = currLocation

    # Checking if the tree is empty
    def isEmpty(self):
        if len(self.tree) == 0:
            return True
        return False
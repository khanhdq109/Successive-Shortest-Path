import heapq

class link():
    def __init__ (self, o, d, capacity = 0, cost = 0):
        self.o = o                  # Origin
        self.d = d                  # Destination
        self.capacity = capacity    # Capacity
        self.cost = cost            # Cost
    
    def update(self, newCapacity = -1):
        if newCapacity == -1:
            return
        self.capacity = newCapacity
        
class network():
    def __init__(self, linkArray = []):
        self.linkArray = linkArray  # Array of links
        
    def update(self):
        self.linkArray = [link for link in self.linkArray if link.capacity > 0]
            

# Support functions
def minFlow(net, path):
    min_capacity = float('inf')
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
        for link in net.linkArray:
            if link.o == current_node and link.d == next_node:
                if link.capacity < min_capacity:
                    min_capacity = link.capacity 
                break
    return min_capacity

def dijkstra(net, src, des):
    # Extract all unique nodes (vertices) from linkArray
    nodes = list(set([l.o for l in net.linkArray] + [l.d for l in net.linkArray]))
    
    # Initialize distances and predecessors
    distances = {node: float('inf') for node in nodes}
    distances[src] = 0
    predecessors = {node: None for node in nodes}
    
    # Priority queue (min-heap)
    priority_queue = [(0, src)] # (distance, node)
    
    while priority_queue:
        # Extract node with the smallest distance from the priority queue
        current_distance, current_node = heapq.heappop(priority_queue)
        
        # If the extracted node has already been processed, skip it
        if current_distance > distances[current_node]:
            continue
        
        # Explore neighbors of the current node
        for link in net.linkArray:
            if link.o == current_node: # Check outgoing edges from current_node
                neighbor_node = link.d
                edge_cost = link.cost
                
                # Calculate new distance to the neighbor node
                new_distance = current_distance + edge_cost
                
                # Update distance and predecessor if a shorter path is found
                if new_distance < distances[neighbor_node]:
                    distances[neighbor_node] = new_distance
                    predecessors[neighbor_node] = current_node
                    # Push updated distance and neighbor node to the priority queue
                    heapq.heappush(priority_queue, (new_distance, neighbor_node))
    
    # Reconstruct the shortest path from source to destination
    path = []
    current = des
    while current is not None:
        path.append(current)
        current = predecessors[current]
    path.reverse() # Reverse to get path from source to destination
    
    # Return the shortest path cost and the path list
    return distances[des], path

def successive_shortest_path(net, source, sink, v):
    totalSaved = 0 # Number of people have been sent to the sink
    flowInfo = {(link.o, link.d): 0 for link in net.linkArray} # Contains flow information of each link
    
    while True:
        # Use Dijkstra's algorithms to find shortest path from source to sink in residual graph
        (distance, path) = dijkstra(net, source, sink)
        
        # Exit loop if no path available or every people reached the destination
        if distance == float('inf') or totalSaved == v: break
        
        # Number of people passed through found path
        passed = minFlow(net, path)
        if totalSaved + passed > v: passed = v - totalSaved
        
        # Update total saved people
        totalSaved += passed
        
        # Update network
        for l in net.linkArray:
            l.update(l.capacity - passed) # Update the link capacity
            net.update() # Remove all links with capacity = 0
            flowInfo[(l.o, l.d)] += passed # update the flow info
    
    return flowInfo

# Example usage
if __name__ == "__main__":
    # Create a sample network
    links = [
        link('0', '1', capacity = 4, cost = 4),
        link('0', '7', capacity = 2, cost = 8),
        link('1', '7', capacity = 4, cost = 11),
        link('1', '2', capacity = 8, cost = 8),
        link('7', '8', capacity = 6, cost = 7),
        link('7', '6', capacity = 10, cost = 1),
        link('2', '8', capacity = 4, cost = 2),
        link('8', '6', capacity = 2, cost = 6),
        link('2', '5', capacity = 4, cost = 4),
        link('2', '3', capacity = 8, cost = 7),
        link('6', '5', capacity = 6, cost = 2),
        link('3', '5', capacity = 10, cost = 14),
        link('3', '4', capacity = 4, cost = 9),
        link('5', '4', capacity = 2, cost = 10),
        link('10', '11', capacity = 2, cost = 10),
    ]
    net = network(links)
    
    src_node = '0'
    des_node = '5'
    shortest_cost = dijkstra(net, src_node, des_node)
    print("Shortest path cost from", src_node, "to", des_node, "is:", shortest_cost)
import heapq
import networkx as nx 
import matplotlib.pyplot as plt
from tabulate import tabulate

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
        
    def draw_network(self, source=None, sink=None):
        # Create a directed graph
        G = nx.DiGraph()
        
        # Add edges to the graph from linkArray
        for link in self.linkArray:
            G.add_edge(link.o, link.d, capacity = link.capacity, cost = link.cost)
        
        # Set node colors based on source and sink
        node_colors = ['skyblue' for _ in G.nodes()]
        if source is not None:
            node_colors[list(G.nodes()).index(source)] = 'red'
        if sink is not None:
            node_colors[list(G.nodes()).index(sink)] = '#90ee90'  # Light green color
        
        # Draw the graph with improved layout
        plt.figure(figsize=(12, 10))
        pos = nx.spring_layout(G, k = 0.3, iterations = 50)  # Adjust 'k' and 'iterations' for better layout
        nx.draw(G, pos, with_labels = True, node_size = 700, node_color = node_colors, font_size = 12, font_color = 'black', edge_color = 'gray', arrows = True)
        
        # Customize edge labels to display (capacity, cost)
        edge_labels = {(u, v): f"({d['capacity']}, {d['cost']})" for u, v, d in G.edges(data = True)}
        nx.draw_networkx_edge_labels(G, pos, edge_labels = edge_labels, font_color = 'black', label_pos = 0.5)
        
        # Show the plot
        plt.title('Directed Network Graph with (capacity, cost) on edges')
        plt.show()
            
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

def print_dict(flow_dict):
    print('--------------')
    print('--FLOW INFOR--')
    print('--------------')
    # Prepare data for tabulate
    table_data = []
    for key, value in flow_dict.items():
        link = f"{key[0]} -> {key[1]}"  # Format link as "O -> D"
        table_data.append([link, value])
    
    # Print table with headers "Link" and "Flow"
    print(tabulate(table_data, headers = ["Link", "Flow"]))
    print('--------------')
    
def print_list(path_list):
    print('')
    intro_lines = ['--------------', '--FOUND PATH--', '--------------']
    
    # Calculate the maximum length across all printed lines
    max_length = max(len(line) for line in intro_lines)
    max_length = max(max_length, max(len('{}. {}'.format(i + 1, ' -> '.join(path))) for i, path in enumerate(path_list)))
    
    # Print introductory lines with aligned dashes
    for line in intro_lines:
        if line == '--FOUND PATH--':
            # Calculate padding for center alignment
            padding_length = max_length - len(line)
            left_padding = padding_length // 2
            right_padding = padding_length - left_padding
            print('-' * left_padding + line + '-' * right_padding)
        else:
            print(line.ljust(max_length, '-'))
    
    for i in range(len(path_list)):
        print('{}. {}'.format(i + 1, ' -> '.join(path_list[i])))
    
    # Print concluding line with aligned dashes
    print('--------------'.ljust(max_length, '-'))

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
    paths = [] # Array to save all found paths
    
    while True:
        # Use Dijkstra's algorithms to find shortest path from source to sink in residual graph
        (distance, path) = dijkstra(net, source, sink)
        
        # Exit loop if no path available or every people reached the destination
        if distance == float('inf') or totalSaved >= v: break
        
        # Number of people passed through found path
        passed = minFlow(net, path)
        if totalSaved + passed > v: passed = v - totalSaved
        
        # Update total saved people and found path
        totalSaved += passed
        paths.append(path)
        
        # Update network
        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]
            for l in net.linkArray:
                if l.o == current_node and l.d == next_node:
                    l.update(l.capacity - passed) # Update the link capacity
                    flowInfo[(l.o, l.d)] += passed # update the flow info
                    break
        net.update() # Remove all links with capacity = 0
    
    return flowInfo, paths

# Example usage
if __name__ == "__main__":
    # Create a sample network
    links = [
        link('A', 'B', capacity = 4, cost = 10),
        link('A', 'C', capacity = 2, cost = 20),
        link('A', 'E', capacity = 6, cost = 5),
        link('B', 'D', capacity = 4, cost = 10),
        link('C', 'D', capacity = 8, cost = 15),
        link('E', 'D', capacity = 2, cost = 10),
        link('D', 'F', capacity = 8, cost = 5),
        link('E', 'F', capacity = 10, cost = 25),
    ]
    net = network(links)
    
    src_node = 'A'
    des_node = 'F'
    
    v = 10
    
    flowInfo, paths = successive_shortest_path(net, src_node, des_node, 10)
    
    print_dict(flowInfo)
    print_list(paths)
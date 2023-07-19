

import networkx as nx
import matplotlib.pyplot as plt
import heapq
import heapq

def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    pq = [(0, start)]
    prev = {start: None}
    while pq:
        curr_distance, curr_node = heapq.heappop(pq)
        if curr_distance > distances[curr_node]:
            continue
        for neighbor, weight in graph[curr_node].items():
            distance = curr_distance + weight['weight']
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                prev[neighbor] = curr_node
                heapq.heappush(pq, (distance, neighbor))
    shortest_paths = {}
    for node in graph:
        if node == start:
            shortest_paths[node] = [start]
        elif prev.get(node) is None:
            shortest_paths[node] = []
        else:
            path = [node]
            while path[-1] != start:
                path.append(prev[path[-1]])
            shortest_paths[node] = list(reversed(path))
    return shortest_paths

    
# Take input for the number of nodes and edges
n, m = map(int, input().split(","))

# Create an empty graph
G = nx.Graph()

# Take input for each edge and add it to the graph
for i in range(m):
    src_node, dest_node, weight = input().split(",")
    G.add_edge(src_node, dest_node, weight=int(weight))

# Compute the forwarding table for each node
for node in sorted(G.nodes()):
    # Use Dijkstra's algorithm to compute the shortest path to all other nodes in the graph
    #shortest_paths = nx.single_source_dijkstra_path(G, node, weight='weight')# node: starting node  , shortest_paths: dictionary keys: Destination node, value: shortest path from source to that destination
    shortest_paths=dijkstra(G,node)
    print(shortest_paths)
    print(f"Forwarding table for node {node}:")
    print("------------------------------------")
    print("Destination     Link")
    print("------------------------------------")
    for dest_node in sorted(G.nodes()):
        if dest_node != node:
             
             print(f" {dest_node}     ({shortest_paths[dest_node][0]},{shortest_paths[dest_node][1]})")

    # Print the forwarding table for the current node
    print(f"Forwarding table for node {node}:")
    for dest_node in sorted(G.nodes()):
        if dest_node != node:
            path = shortest_paths[dest_node]
            next_hop = path[1]
            cost = nx.shortest_path_length(G, source=node, target=dest_node, weight='weight')
            print(f"Destination: {dest_node} Next hop: {next_hop} Cost: {cost}")
    # Draw the graph
    pos2 = nx.spring_layout(G)
    nx.draw(G, pos2, with_labels=True, font_weight='bold')

    # Draw the shortest paths
    for dest_node in sorted(G.nodes()):
        if dest_node != node:
            path_edges = [(shortest_paths[dest_node][i], shortest_paths[dest_node][i+1]) for i in range(len(shortest_paths[dest_node])-1)]
            nx.draw_networkx_edges(G, pos2, edgelist=path_edges, width=3, alpha=0.5, edge_color='r')

        # Show the plot
    plt.title('Second Graph')
    plt.show()

# Visualize the topology
pos1 = nx.spring_layout(G)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_nodes(G, pos1, node_size=500)
nx.draw_networkx_edges(G, pos1)
nx.draw_networkx_labels(G, pos1, font_size=20, font_family='sans-serif')
nx.draw_networkx_edge_labels(G, pos1, edge_labels=labels, font_size=10, font_family='sans-serif')


plt.axis('off')
plt.title('Original Topology')
plt.show()

















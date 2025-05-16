import matplotlib.pyplot as plt
import networkx as nx
import math
from queue import PriorityQueue

class Graph:
    def __init__(self):
        # Define coordinates for each node
        self.coordinates = {
            "1": (0, 0),
            "A": (1, 2),
            "B": (4, 2),
            "C": (1, 5),
            "D": (3, 6),
            "E": (6, 6),
            "F": (5, 8),
            "Z": (7, 10)
        }

        # Define graph structure (edges without weights)
        self.graph = {
            "1": [("A")],
            "A": ["B", "C", "D"],
            "B": ["A", "E"],
            "C": ["A"],
            "D": ["A", "F"],
            "E": ["B", "Z"],
            "F": ["D", "Z"],
            "Z": ["E", "F"]
        }

        self.edges = []
        self.weights = self.calculate_weights()
        self.heristics = {
            "1": 0,
            "A": 0,
            "B": 0,
            "C": 0,
            "D": 0,
            "E": 0,
            "F": 0,
            "Z": 0
        }
        self.populate_edges()

    def calculate_weights(self):
        """Calculate weights (distances) for all edges."""
        weights = {}
        for from_node, neighbors in self.graph.items():
            for to_node in neighbors:
                weights[(from_node, to_node)] = self.euclidean_distance(from_node, to_node)
        return weights

    def euclidean_distance(self, from_node, to_node):
        """Calculate Euclidean distance between two nodes."""
        x1, y1 = self.coordinates[from_node]
        x2, y2 = self.coordinates[to_node]
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def populate_edges(self):
        """Populate edges with weights for visualization."""
        for (from_node, to_node), weight in self.weights.items():
            self.edges.append((from_node, to_node, weight))

    def neighbors(self, node):
        """Get neighbors for a given node."""
        return self.graph.get(node, [])

    def get_cost(self, from_node, to_node):
        """Get the cost (weight) between two nodes."""
        return self.weights.get((from_node, to_node), float('inf'))

    def get_heuristic(self, node):
        """Get the heuristic value for a node."""
        return self.heristics.get(node, float('inf'))

def astar(graph, start, goal):
    visited = []
    queue = PriorityQueue()
    came_from = {}
    queue.put((0, start))
    cost_so_far = {start: 0}

    while not queue.empty():
        _, current = queue.get()

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.get_cost(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + graph.get_heuristic(neighbor)
                queue.put((priority, neighbor))
                came_from[neighbor] = current

    # Reconstruct the path
    path = []
    node = goal
    while node != start:
        path.append((came_from[node], node))
        node = came_from[node]
    path.reverse()

    return path

# Create the graph object
graph = Graph()
edges = graph.edges
weights = graph.weights

# Run A* algorithm
optimal_path = astar(graph, "A", "Z")
print("Optimal Path:", optimal_path)

# Create a directed graph for visualization
G = nx.DiGraph()

# Add edges and weights to the graph
for from_node, to_node, weight in edges:
    G.add_edge(from_node, to_node, weight=round(weight, 2))

# Get positions for nodes using their coordinates
pos = graph.coordinates

# Draw the graph
plt.figure(figsize=(10, 8))
nx.draw(G, pos, with_labels=True, node_size=200, node_color='lightblue', font_size=10, font_weight='bold')

# Draw edge labels (weights)
edge_labels = {(u, v): f"{d['weight']}" for u, v, d in G.edges(data=True)}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10)

# Highlight the optimal path
optimal_edges = [(u, v) for u, v in optimal_path]
nx.draw_networkx_edges(G, pos, edgelist=optimal_edges, edge_color='red', width=2)

# Show the graph
plt.title("Graph Visualization with Optimal Path")
plt.grid(1)
plt.show()

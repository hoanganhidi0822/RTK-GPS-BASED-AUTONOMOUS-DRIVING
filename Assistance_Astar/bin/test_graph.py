segments = [("A", "B"),("A", "E"), ("B", "C"), ("C", "D"), ("D", "E"), ("E", "F"), ("F", "A")]

# Chuyển đổi segments sang cấu trúc đồ thị
graph = {}
for start, end in segments:
    if start not in graph:
        graph[start] = []
    graph[start].append(end)

print("Graph structure:")
print(graph)



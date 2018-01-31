import prm
from matplotlib import pyplot as plt


X = (0, 10)
Y = (0, 5)
state_space = prm.StateSpace([X, Y])

graph = prm.Graph(state_space, prm.euclid)
graph.load('../data/')

start = 100
goal = 201
shortest_path = prm.astar(start, goal, graph, prm.euclid)

print('shortest_path: ', shortest_path)


fig = plt.figure()
ax = fig.add_subplot(111)

annotate = False
dot_size = 6
lwidth = 3

for node in graph.nodes:
    s = graph.nodes[node][0]
    if node == start:
        color = 'blue'
    elif node == goal:
        color = 'red'
    else:
        color = 'yellow'
    ax.plot([s[0]], [s[1]], marker='o', markersize=dot_size, color=color)
    if annotate:
        ax.annotate(
            node,
            xy=(s[0], s[1]), xytext=(-20, 20),
            textcoords='offset points', ha='right', va='bottom',
            bbox=dict(boxstyle='round,pad=0.5', fc=color, alpha=0.5),
            arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))

for edge in graph.edges:
    s1, s2 = graph.nodes[edge[0]][0], graph.nodes[edge[1]][0]
    dot1, dot2 = (s1[0], s2[0]), (s1[1], s2[1])
    ax.plot(dot1, dot2, color='gray', linewidth=lwidth//2, linestyle='--')

for i in range(len(shortest_path)-1):
    n1, n2 = shortest_path[i], shortest_path[i+1]
    s1, s2 = graph.nodes[n1][0], graph.nodes[n2][0]
    dot1, dot2 = (s1[0], s2[0]), (s1[1], s2[1])
    ax.plot(dot1, dot2, color='green', linewidth=lwidth)


# To plot a and made state path
# state_path = []

# for i in range(len(state_path)-1):
#     s1, s2 = state_path[i], state_path[i+1]
#     dot1, dot2 = (s1[0], s2[0]), (s1[1], s2[1])
#     ax.plot(dot1, dot2, color='red', linewidth=lwidth)


plt.show()

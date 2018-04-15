# Astar
This project utulizes two pathfinding algorithms built in python; Astar (A*) and Dijkstras.

The A* algorithm has a weighted heuristic allowing for a dynamic balance between Greedy First-Search and Dijkstra.

```sh
f(n) = g(n) + h(n)
```
where **f(n)** is total cost, **g(n)** is cost so far from node n to the start, **h(n)** is our heuristic function below.
### Heuristic

```sh
def Heuristic(h, w1, w2, c):
    h += h(w1)
    c = c(w2)
    return h+c
```

where **h** is the distance from node n to the goal, **w1** is the Greedy First-Search weight, **w2** is the Dijkstra weight, and **c** is the cost from the current node **n** to the next node **n'**

### Usage
```sh
$   python3 pathfinding.py
```
| Input | Result |
| ------ | ------ |
| 0 (zero) | Execute pathfinding with Dijkstras Algrothim  |
| 1 | Execute pathfinding with the Astar Algrothim |
| 2 | Set the w1 Greed First-Search weight |
| 3 | Set the w2 Dijkstra weight |

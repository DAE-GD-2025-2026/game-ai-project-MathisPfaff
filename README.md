# Game AI Project

This project is a **school assignment** for the subject **Gameplay Programming**.

# Assignments

## Assignment 1: Flocking
This focuses on **flocking behavior**.  
Flocking is built from three classic group behaviors:

- **Cohesion**: agents move toward nearby group members to stay together.
- **Separation**: agents keep distance to avoid crowding and collisions.
- **Alignment**: agents steer to match the average direction of neighbors.

### Steering Approaches

The project uses both **blended steering** and **priority steering**:

- **Blended Steering** combines multiple behaviors (cohesion, separation, alignment) using weights, producing smooth and natural flock movement.
- **Priority Steering** prioritizes certain behaviors, in this case it prioritizes the evade behavior when the evade target is to close, when the evade target is far enough it switches back to the blended steering.

### Spatial Partitioning

To improve performance with many agents, the flocking system can use **partitioning** (flat spatial partitioning).  
This reduces neighbor checks by searching only nearby partitions instead of all agents, making flocking more efficient and scalable.

## Assignment 2: A* Pathfinding + Navigation Meshes
This focuses on **pathfinding** within a navmesh

### Pathfinding Algorithms

Two algorithms are implemented to find a path through the graph:

- **BFS (Breadth-First Search)**: explores nodes level by level, finding the shortest path in terms of number of steps.
- **A***: uses a heuristic to guide the search toward the goal more efficiently, taking node costs into account to find the optimal path.

### Tile Types

The graph supports different tile types that affect pathfinding:

- **Mud tiles**: traversable but cost twice as much as a normal node, making the algorithms prefer paths around them when possible.
- **Water tiles**: completely impassable, forcing the algorithms to find an alternative route around them.

### Navigation Mesh (NavMesh)

The navmesh divides the walkable area into triangles and uses **portals** for path planning:

- **Portal Generation**: a portal is placed at the midpoint of each shared edge between triangles, and portals are connected to the other portals within the same triangle, forming a graph of traversable connections.
- **SSFA Optimization (Simple Stupid Funnel Algorithm)**: once a path through the portals is found, SSFA optimizes it by pulling the path as straight as possible through the portal funnel, making the resulting movement path smooth and direct.

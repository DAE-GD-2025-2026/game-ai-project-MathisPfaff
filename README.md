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

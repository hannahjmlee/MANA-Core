# Multi-Agent Navigation Algorithms Core 

This is Hannah's Multi-Agent Navigation Algorithm (MANA) Core library. This is a C++ library with code developed for testing and performing experiments. For my development library in Python, please see MANA-Labs. To start,
begin by running:
```
chmod +x init.sh
./init.sh
```
This will load all of the map and scenario files necessary for running
experiments. Then, to build the library, run:
```
chmod +x build.sh
./build.sh
```

## Multi-Agent Pathfinding Algorithms
Currently, we support the following multi-agent pathfinding algorithms:
- Conflict-Based Search (CBS)
- Priority-Based Search (PBS)
- Hierarchical Composition Conflict-Based Search (HC-CBS)

We will be adding these algorithms in future releases: 
- Parallel Hierarchical Composition Conflict-Based Search (PHC-CBS)
- Dynamic Parallel Hierarchical Composition Conflict-Based Search (DPHC-CBS)
- Pathfinding with Rapid Information Sharing using Motion constraints (PRISM) 

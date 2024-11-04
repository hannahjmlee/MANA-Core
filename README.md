# Multi-Agent Navigation Algorithms Core - Constraint Analysis

This library, known as Hannah's Multi-Agent Navigation Algorithm (MANA) Core, is a C++ library for testing and conducting experiments on multi-agent pathfinding algorithms. This branch is specifically tailored for the study _"An Analysis on Constraint-Based Search Multi-Agent Pathfinding Algorithms."_

## Usage
To reproduce our results, start by loading the benchmark maps and scenario files:
```
chmod +x init.sh
./init.sh
```
Next, build the library:
```
chmod + x build.sh
./build.sh
```
This will produce an executable named `constraint_analysis.exe`, which can be run as follows:

    ./constraint_analysis.exe [map name] [resolution] [scenario number] [number of agents] [algorithm]

-   **map_name**: Any benchmark map filename
-   **resolution**: Must be one of `[1, 2, 4]`
-   **scenario_number**: Ranges from `1` to `25`
-   **number_of_agents**: For the most part, can be arbitrarily set
-   **algorithm**: Specify either `CBS` or `PBS`

The output will be formatted as:
        `[map_name],[scenario_number],[algorithm],[number_of_agents],[solve_time],[solution_cost],[explored_nodes],[total_nodes]`

To run multiple instances, consider scripting calls to this executable for batch processing.

# SweepAI-Intelligent-Cleaning-System
Engineered an intelligent cleaning agent leveraging a variety of search algorithms using Python, enhancing the efficiency and effectiveness of the cleaning process in a dynamic environment
# Vacuum Cleaning Agent with Search Algorithms

This project implements a vacuum cleaning agent that operates in a 2D grid environment containing rooms that can be either dirty or clean, as well as blocked rooms. The goal of the agent is to use various search algorithms to navigate the grid and clean all the dirty rooms.

## Project Overview

The project includes the following components and features:

1. **Environment**: The environment is represented as a 2D grid of rooms, with some rooms being dirty and others being blocked. The agent starts from the middle of the grid.

2. **Search Algorithms**: Four search algorithms are implemented:
   - Breadth-First Graph Search (BF Graph)
   - Depth-First Graph Search (DF Graph)
   - Uniform Cost Search (UC)
   - A* Search (A*)

3. **Visualization**: The visual components of the project include:
   - The explored area is colored in pink during the search process (for all algorithms).
   - Number-of-steps and Performance labels at the top of the interface are updated as the agent progresses through the grid.
   - The chosen path is rendered in a different color (e.g., orange or light blue) than the explored area.

4. **Path Computation**: When a search algorithm is selected, the agent computes a path to the closest dirty room based on the chosen search method. The path is returned, and the agent can step through it one move at a time or run automatically.

## Getting Started

To run the project, follow these steps:

1. Clone the repository to your local machine:

   ```bash
   git clone <repository_url>
   ```

2. Navigate to the project directory:

   ```bash
   cd vacuum-cleaning-agent
   ```

3. Run the main script:

   ```bash
   python xy_vacuum_environment.py
   ```

4. Use the interface to select a search algorithm and control the agent's actions.

## Project Structure

The project directory is structured as follows:

- `xy_vacuum_environment.py`: The main script that contains the implementation of the vacuum cleaning agent and the user interface for selecting search algorithms and controlling the agent.

- `search_algorithms.py`: This module contains the implementations of the four search algorithms: BF Graph, DF Graph, UC, and A*.

- `environment.py`: Defines the grid environment, including the layout of rooms, dirty rooms, and blocked rooms.

- `utils.py`: Contains utility functions used in the project.

- `README.md`: This file, which provides an overview of the project and instructions for running it.

## Customized Functions

Two functions, `path_cost()` and `heuristic h()`, have been modified from the base class `Problem` for use in Uniform Cost Search (UC) and A* Search (A*). The changes are described in the comment sections of these functions in `xy_vacuum_environment.py`.

## Analysis

In this project, the agent does not consider turning when choosing a direction to move. It simply moves in the selected direction without changing its orientation. This simplification aligns with the previous assignment's requirements.

Please feel free to explore and modify the project as needed. Enjoy experimenting with different search algorithms for the intelligent vacuum cleaning agent!

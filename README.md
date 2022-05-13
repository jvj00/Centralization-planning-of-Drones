# Centralization-planning-of-Drones
##Description
This repository allows you to create a central planning of drones, with planner tools such as popf (or plansys2 in ros): it contains domains for multiple drones,
mix drones/agv (and others in development), and a tool to create problem by simple giving it some parameters (instead of creating all points and links).
## Requirements
* Ubuntu Linux OS
* [GCC Compiler](https://gcc.gnu.org/)
* [POPF](https://github.com/roveri-marco/popf)
## Installation
1. Compile map_tool.cpp with command `gcc map_tool.cpp -o map_tool`
2. Execute map_tool with parameters (see `example_of_use_map_tool.txt` or type `./map_tool --help` for the guide)
3. Than execute popf in the popf directory by typing `./plan <path_to_domain.pddl> <path_to_generated_problem.pddl> sol.res`
## Contributors
[jvj00](https://github.com/jvj00)

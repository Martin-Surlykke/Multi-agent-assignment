/*******************************************************\
|                AI and MAS: Searchclient               |
|                        README                         |
\*******************************************************/

*** INTRODUCTION *** 
This is a boilerplate platform for running the hospital environment for the AIMAS project. 

Currently, it can run 5 seperate algorithms: 
- BFS
- DFS
- ASTAR 
- WASTAR    
- GREEDY

It functioned well enough for the warmup assignment, handling most levels with reasonable solving times.
It can both handle movement and pushing the boxes. 

For the final project, the current algorithms are insufficient for getting a adequate score. Thus we need to implement
Something more efficient. 

I (Martin) am currently working on CBS, but many other algorithms are possible.
As i have understood the goal is to experiment, and show "good ideas" to the teacher. As such, implementing any
algorithm you find interesting would be a possibility. 

That said, from reading the assignment there are a few other areas we could work on: 

1. This program uses the teachers setup for handling memory, states actions etc. He has himself stated that 
it is not made to be efficient, so we could probably improve our score by reducing unneccessary iterative processees. 

2. CBS handles conflicts, some of the papers we found sought to build on top of CBS, it might be possible to do this. 

3. We need to produce a competition level, the guide for creating a level can be found in the project (or warmup) 
description. I think it gives extra points if we produce a nice level. 

4. If we manage to make a decent algorithm, i might port it to JAVA afterwards for a performance boost. 

5. We might be able to create a more efficient data management system. 


*** GETTING STARTED ON THE PROJECT *** 

The project consists of 8 files contained in the "searchclient" folder, these files are: 
Action
Color
Frontier
Graphsearch
Heuristic
Memory
Searchclient
state

Of these 8 files, not all of them are necessary to produce an algorithm. 3 of the files i haven't personaly
touched yet, namely: 
1. color (mainly a cosmetic item)
2. searchclient (handles sending your algorithmic results to the java server)
3. memory (handles the memory structure of the algorithm) 

The other 5 files are responsible for running the program / algorithm, but 2 files were mostly "set and forget"

State - This file is used to handle which states an agent can be in, 
and which actions are available to them given their state

Action - This file defines which actions an agent can do

** As far as i remember, i don't think there are any new actions or states which need to be implemented leaving only 
3 files which actually handle the agents behaviour. 

1. Graphsearch - This file currently runs a simple Graphsearch. It then creates and gathers children 
based on the specified frontier chosen. 

2. Frontier - This file defines the "frontiers" for different algorithms i.e. their priority queues. 

3. heuristc - This is the general file for defining any behaviour used to define any logic besides the priority.
I.e it handles the heuristic of the ASTAR algorithm. 


*** Getting started *** 

the program can be run from the root directory using the command: 
java -jar server.jar -l levels/{CHOSEN LEVEL} -c "python -m searchclient.searchclient {CHOSEN ALGORITHM}" 
-g -s 150 -t 180  

Levels should be denoted as their file is written, for example: SAD01.lvl
There are currently 5 commands for algortihms: 
-bfs
-dfs
-astar
-wastar
-greedy

If you add a new algortihm, be sure to add another label so we can run them seperately. 

An example of a function command would be: 
java -jar server.jar -l levels/SAD1.lvl -c "python -m searchclient.searchclient -astar" -g -s 150 -t 180 


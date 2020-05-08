The following three Matlab codes exhibits the behaviour of three path finding algorithms.
Potential Fields, Probabilistic Roadmap (PRM), and Rapidly-exploring random tree (RRT)
Each file pulls a map with obstacles (sim_map.pgm) for the robot to navigate through.

The robot is a simple 2 wheel holonomic robot with kinematic model given as:

x + T*v*cos(theta)   
y + T*v*sin(theta)
theta + w*T


The Matlab code is broken into sections

***FOR BEST RESULTS, RUN THE CODE IN SECTIONS***

**run clear section before starting code for new question


*There are a lot of graphs, they show the entire process and pretty much all path and generation in between

*comments should guide what is being done

*speed of Potential_Fields can be controlled by changing velocity and angular velocity gain values, too high causes bad stuff though

*the number of points and range of "closest" points can be changed in PRM to get different density maps
*if the number of points is too small or range is too short, some essential paths might not be created
*check the plot right after creating PRM to ensure map is good before continuing 
*PRM uses Dijkstra's algorithm to find shortest path

*in RRT, the distance away from walls that the points can be can be adjusted
*also, how close the point must be to end can also be changed
*decreasing this distance will create more branches 
*these parameter are indicated in the code

*if a comment doesnt make sense, ignore it, i probably forgot to change it
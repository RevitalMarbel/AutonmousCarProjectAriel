# AutonmousCarProjectAriel
obstacle avoidance using fast slam- python project. poject for autonomous car course Ariel university

Project goal:
In this project , we designed and implemented an autonomous car with the ability to track , and avoid obstacles.
Smart course plan -save the obstacles location in a predefined greed around the car location
Fast slam. (using the car GPS location)
Save the obstacles coordinates on the map in order to avoid them on the next visit.  

Algorithm:
We used  the A* algorithm – a heuristic path finding algorithm.
Extension of Dijkstra's algorithm – uses guided search.
A* first use was on Shaley the robot- the first general-purpose mobile robot to be able to reason about its own actions

Implementation:

Grid: we created a virtual  grid of cells using the GPS coordinates, each cell was 2 meter  wide and 2 meter long.  ( this number will allow us to get a safe distance from protentional obstacles)
Obstacles: we used the Lidar data ,witch is a set of distances and angles, as well as the car location to place the obstacles on the grid. This is done several times during the car (operation???).
Move cost : we gave lower grades to cells closest to the car location and in front of the car.

Technical Challenges:

1. Power (!!!!) , each component need to have enough power to work .
Raspberry Pi , we used RP to control the car commands and receive data such as GPS coordination's and Lidar results .
GPS fix: we needed to wait several minutes before starting the algorithm to get GPS data.
Guided mode: for safety – we added an option to manually stop the car in case of a bug. 


Created by :
Vlad Landa
Saed Ashley
Revital marbel

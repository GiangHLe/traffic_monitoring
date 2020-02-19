# Traffic monitoring system by computer vision and machine learning
The system is in a jupyter notebook in "source code" directory. For details, please take a look on [our paper](paper.pdf).

It is divide into two mode, and need a lot of pre-processing step for different position of a camera. 

Our system can solve three problem:
+ Overspeed
+ Wrong lane departure
+ Cross the red line

We combined "overspeed" and "wrong lane deparute" into one mode, because it can solve on highway and called that mode is "speed". Another mode is "crossRedLine", this mode work well with the intersection road where has the traffic light.

We success in catching the violation situations

<center><img src="./results/speed.jpg" alt="Overspeed" width="500"/><figcaption>Fig 1. Assume speed limited is 60km/h, two vehicle get violation</figcaption></center>

<center><img src="./results/lane_cross.png" alt="Wrong lane" width="500"/><figcaption>Fig 2. System know that vehicle in green lane is moving in purple lane</figcaption></center>

<center><img src="./results/violation_red_light.jpg" alt="Cross red line" width="500"/><figcaption>Fig 3. While the light is red, two vehicle cross over the white line</figcaption></center>

However, there are a lots of problem we need to face with. For example, we use SORT as tracking algorithm, so when the vehicles get overlap, it will reset the ID of that vehicle and assume that is the new one. 

<center><img src="./results/cross_red_result.png" alt="Issue" width="500"/><figcaption>Fig 4. A frame in result video from "crossRedLine" mode</figcaption></center>

In Fig 4, in yellow area, there are 7 vehicle and all of them get violation. However, system can only recognize 2 vehicles and one of them will get the ticket.

This system can take the input is local video or even stream link. We use twitch server for streaming. While processing with stream video, we realized that 720p video (resolution: 1080x720 pixels) returned better results with YOLOv3 and SORT algorithm. 

We will focus on the resolution and other algorithm such as ***Deep Sort*** in the future.

## How to run this system?
To run this system, we need to prepare a lot of parameters.

* ***Mode***. This system divide into two mode, ***speed*** for measure speed and lane of each vehicle and ***crossRedLine*** for catch the cross red line violation (only work when YOLOv3 can catch the traffic light). 
* ***Three vanishing points*** for different angles of camera, in this project we use diamond space method which follow by [a project of Brno University of Technology in Czech Republic](http://www.fit.vutbr.cz/research/groups/graph/pclines/papers/2013-BMVC-Dubska-VanishingPointsDetection.pdf).
* ***The principle point*** is the center of each frame $pp=(W//2,H//2)$
* ***Scene Scale***: we need to convert two points from image to Road plane coordinate, then find the distance by Euclidean's distance, finally find that distance in real life and let it divide with the Euclidean's distance. You can get the distance in real life by google map or measure by other methods.
* ***Best performance range***. The camera calibration is not good on all positions of each frame. You will need to test many case to find the region of interest (RoI) for each different angle.
* ***All lines***, it is necssary to find all lines on a frame of video. We haven't found the automatical way to find these lines so we measure them by the simple line equation of 2D coordinate: $y=ax+b$ with $x$ and $y$ is two random points on each line.
* ***Mask***. A path lead to binary image, divide the special area in ***crossRedLine*** mode, read paper for more information. 
* ***deadLine***. A list with three parameter [a,b,c] where a,b and c is the parameter of line equation $ax+by+c=0$
* ***allowed_line*** is a list contain indexs of allowed lanes.

The way how to run code in notebook named ***Systems.ipynb***, take a look for more details.
# Bot Ross Robot

This project utilized the [X3Plus](https://category.yahboom.net/products/rosmaster-x3-plus) robot running [ROS](https://wiki.ros.org/) to make the robot replicate drawings inputted into reality
This was a semester-long project completed within the Siena College CSIS-370 Robotics course.

To run, ensure that roscore and bringup commands are running in the background (part of ROS), 
and run the paint.py script inside /scripts

* `rosrun ros-whiteboard ./paint.py`

The main GUI application along with the graphing portion and calculations towards
how the robot should move are within paint.py. The commands to actually move the robot are
within main.py and are imported into and called from paint.py. Both of these files are inside
the scripts folder. Otherwise, other files are included as reference material for our main project.

This project was featured in a [Siena College news article](https://www.siena.edu/news/story/turn-the-robots-loose/) on December 20th, 2024. 

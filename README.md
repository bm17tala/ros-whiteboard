# Bot Ross Robot

This project utilized the X3Plus robot to make it replicate drawings inputted into reality.

To run, ensure that roscore and bringup commands are running in the background, 
and run the paint.py script inside /scripts

* `rosrun ros-whiteboard ./paint.py`

The main GUI application along with the graphing portion and calculations towards
how the robot should move are within paint.py. The commands to actually move the robot are
within main.py and are imported into and called from paint.py. Both of these files are inside
the scripts folder. Otherwise, other files are included as reference material for our main project.
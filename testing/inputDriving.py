from robotFunctions import *
options = [robot.Forward,robot.Back,robot.TurnRight,robot.TurnLeft,robot.Stop]
while True:
    opt = int(input("""0.) Forward
1.) Backward
2.) TurnRight
3.) TurnLeft
4.) Stop
: """))
    options[opt]()

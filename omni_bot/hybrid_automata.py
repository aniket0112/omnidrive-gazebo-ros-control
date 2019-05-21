import numpy as np

# Configurable constants of the Hybrid Automata
R = 0.05 #Radius of omnibot chassis

KP = 5
DISTANCE_TOL = 0.1 #meters
HEADING_TOL = 0.1 #radians
VELOCITY = 3 #m/s

class Position():
    def __init__(self,x,y,yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw
    def mag(self):
        return np.sqrt(self.x**2+self.y**2)
class uniCycleState:
    def __init__(self,v,w):
        self.v = v
        self.w = w
class laserSensor:
    def __init__(self,mag,angle,size):
        self.mag = mag
        self.angle = angle
        self.size = size
def threelWheelKinematics(distance,phi,error,alpha):
    robot = uniCycleState(0,0)
    if abs(error)>HEADING_TOL:
        robot.w = KP*error
    else:
        robot.w = 0
    if distance>DISTANCE_TOL:
        robot.v = VELOCITY
    else:
        robot.v = 0
    rpmFront = (robot.v * ((np.cos(alpha + 1.57) * np.cos(phi))
                        +(np.sin(alpha + 1.57) * np.sin(phi)))
                        + robot.w*R)
    rpmLeft = (robot.v * ((np.cos(alpha + (2.0933+1.57)) * np.cos(phi))
                        +(np.sin(alpha + (2.0933+1.57)) * np.sin(phi)))
                        + robot.w*R)
    rpmRight = (robot.v * ((np.cos(alpha + (4.1866+1.57)) * np.cos(phi))
                        +(np.sin(alpha + (4.1866+1.57)) * np.sin(phi)))
                        + robot.w*R)
    return (robot,rpmLeft,rpmRight,rpmFront)
def angleBetweenTwoVector(a_,b_):                                               # Signed angle between two vectors
    a = Position(np.cos(a_),np.sin(a_))
    b = Position(np.cos(b_),np.sin(b_))
    angle = np.arccos((a.x*b.x+a.y*b.y)/(a.mag()*b.mag()))
    crossproduct = a.x*b.y-a.y*b.x
    if crossproduct > 0:
        return angle
    else:
        return -angle
def GTG(currPosition,desiredPosition,desiredHeading):                                          # Mode: Go To Goal
    phi = np.arctan2(desiredPosition.y-currPosition.y,
                                desiredPosition.x-currPosition.x)
    distance = np.sqrt(np.square((desiredPosition.x-currPosition.x))
                      +np.square((desiredPosition.y-currPosition.y)))
    error = angleBetweenTwoVector(currPosition.yaw,desiredHeading)
    return threelWheelKinematics(distance,phi,error,currPosition.yaw)
def findObstacleAngle(obstacle_sense):                                          # Conversion of LaserScan Message to Obstacle Angle
    obstaclePosition = Position(0,0)
    min = 100000
    for i in range(len(obstacle_sense.mag)):
        if obstacle_sense.mag[i] is not 0:
            if min > obstacle_sense.mag[i]:
                min = obstacle_sense.mag[i]
            w = [(1/obstacle_sense.mag[i])*np.cos(obstacle_sense.angle[i]),
                (1/obstacle_sense.mag[i])*np.sin(obstacle_sense.angle[i])]
            obstaclePosition.x = obstaclePosition.x + w[0]
            obstaclePosition.y = obstaclePosition.y + w[1]
    return (min,np.arctan2(obstaclePosition.y,obstaclePosition.x))
def followCurve(currPosition,y,FINAL_TIME,t,desiredHeading=0):
    robot = uniCycleState(0,0)
    if t <= FINAL_TIME:
        phi = np.arctan2(y,1)
        distance = 10
        error = angleBetweenTwoVector(currPosition.yaw,desiredHeading)
        return threelWheelKinematics(distance,phi,error,currPosition.yaw)
    else:
        return (robot,0,0,0)

#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()
left_drive_smart = Motor(Ports.PORT7, 1.0, True)
right_drive_smart = Motor(Ports.PORT12, 1.0, False)

drivetrain = SmartDrive(left_drive_smart, right_drive_smart, brain_inertial, 200)
distance_11 = Distance(Ports.PORT11)
touchled_8 = Touchled(Ports.PORT8)
optical_9 = Optical(Ports.PORT9)
motor_10 = Motor(Ports.PORT10, True)
motor_4 = Motor(Ports.PORT4, True)



# generating and setting random seed
def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 
    
# Initialize random seed 
initializeRandomSeed()


# Color to String Helper
def convert_color_to_string(col):
    if col == Color.RED:
        return "red"
    if col == Color.GREEN:
        return "green"
    if col == Color.BLUE:
        return "blue"
    if col == Color.WHITE:
        return "white"
    if col == Color.YELLOW:
        return "yellow"
    if col == Color.ORANGE:
        return "orange"
    if col == Color.PURPLE:
        return "purple"
    if col == Color.CYAN:
        return "cyan"
    if col == Color.RED_VIOLET:
        return "red_violet"
    if col == Color.VIOLET:
        return "violet"
    if col == Color.BLUE_VIOLET:
        return "blue_violet"
    if col == Color.BLUE_GREEN:
        return "blue_green"
    if col == Color.YELLOW_GREEN:
        return "yellow_green"
    if col == Color.YELLOW_ORANGE:
        return "yellow_orange"
    if col == Color.RED_ORANGE:
        return "red_orange"
    if col == Color.BLACK:
        return "black"
    if col == Color.TRANSPARENT:
        return "transparent"
    return ""
vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    brain_inertial.calibrate()
    while brain_inertial.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


# Calibrate the Drivetrain
calibrate_drivetrain()

#endregion VEXcode Generated Robot Configuration
gridspace = 28 #cm
drivetrain.set_heading(0,DEGREES)
motor_4.set_heading(0,DEGREES)
modegrid = [[0,9],[13,12],[25,9]]
grid = [(0,0),(0,1),(0,2),(0,3),
        (1,0),(1,1),(1,2),(1,3),
        (2,0),(2,1),(2,2),(2,3)]

class Navigator:
    def __init__(self,cur,grid):
        self.cur = cur
        self.grid = grid
        self.mode = 0

    def close(self, pos, grid):
        valid = []
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for dx, dy in directions:
            # Access the elements of the cur tuple using indices [0] and [1]
            neighbor = (pos[0] + dx, pos[1] + dy)
            if neighbor in grid:
                valid.append(neighbor)
        return valid


    def dikstras(self,dest,grid):
        #generates adjecsemnt graph
        graph = {}
        for i in grid:
            graph.update({i:self.close(i,grid)})
        #set inital distances
        dis = {val: float('inf') for val in graph}
        dis[self.cur] = 0

        prev = {val:None for val in graph}

        unvis = set(graph.keys())

        while unvis:
            curNode = None
            for node in unvis:
                if curNode is None:
                    curNode = node
                elif dis[node] < dis[curNode]:
                    curNode = node

            if dis[curNode] == float('inf'):
                break

            unvis.remove(curNode)

            for near in graph[curNode]:
                alt = dis[curNode] + 1
                if alt < dis[near]:
                    dis[near] = alt
                    prev[near] = curNode

        path = []
        target = dest
        while target is not None:
            path.insert(0, target)
            target = prev[target]

        return path

    def move(self,dir,drec=FORWARD,final=False):
        if not final:
            power = 280 #mm
        else:
            power= 220 #mm
        
        dire = drec
        if dir == (0,1):
            drivetrain.turn_to_heading(90,DEGREES)
            drivetrain.drive_for(dire,power,MM,wait=True)
        elif dir == (0,-1):
            drivetrain.turn_to_heading(270,DEGREES)
            drivetrain.drive_for(dire,power,MM,wait=True)
        elif dir == (1,0):
            drivetrain.turn_to_heading(0,DEGREES)
            drivetrain.drive_for(dire,power,MM,wait=True)
        elif dir ==(-1,0):
            drivetrain.turn_to_heading(180,DEGREES)
            drivetrain.drive_for(dire,power,MM,wait=True)
        elif dir == (0,0):
            pass

    def shift(self,mode):
        if mode == 0:
            motor_4.spin_to_position(0,DEGREES,wait=True)
            if self.mode == 2:
                drivetrain.drive_for(FORWARD,30,MM)
            self.mode = 0
        elif mode == 1:
            motor_4.spin_to_position(105,DEGREES,wait=True)
            if self.mode != 2:
                drivetrain.drive_for(REVERSE,30,MM)
            self.mode = 1
        elif mode == 2:
            motor_4.spin_to_position(180,DEGREES,wait=True)
            if self.mode == 2:
                drivetrain.drive_for(FORWARD,30,MM)
            self.mode = 2


    def goTo(self,dest,remove=None,switchTo=0,release=False,grab=False):
        curpos = self.cur
        if grab:
            motor_10.spin(REVERSE)
        removedGrid = list(self.grid)
        if release:
            motor_10.spin(FORWARD)
        if remove:
            for i in remove:
                removedGrid.remove(i)
        path = self.dikstras(dest,removedGrid)
        if len(path) > 1:
            path.pop(0)
        prev = path[0]
        for i in path:
            final = False
            if i == dest and grab:
                final = True
            try:
                if len(i) >= 2 and i == path[-2]:
                    self.shift(switchTo[0])
            except IndexError:
                pass
            self.move((tuple(x - y for x, y in zip(i, curpos))),final=final)

            if final:
                if grab:
                    motor_10.spin(FORWARD)
                if release:
                    motor_10.spin(REVERSE)
                self.move(tuple((x - y) for x, y in zip(i, curpos)),drec=REVERSE,final=final)
                self.shift(0)
            wait(500,MSEC)
            curpos = i
        
        self.cur = curpos



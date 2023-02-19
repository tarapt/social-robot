from AriaPy import *
from ArNetworkingPy import *
import sys
import math
import logging
import requests
from cursed import CursedApp, CursedWindow, CursedMenu

SERVER_PORT = 7272
PERSON_LOCATION_SERVICE_URL = "http://localhost:5100/detections"
SAFE_DISTANCE = 0.5  # in meters
logging.basicConfig(
    filename='robot_client.log',
    format='%(levelname)s: %(message)s',
    filemode='w',
    level=logging.DEBUG)


def get_detections():
    response = requests.get(PERSON_LOCATION_SERVICE_URL)
    position = response.json()
    x, z, th = float(
        position['x']), float(
        position['z']), float(
            position['th'])
    return x, z, th


class MainWindow(CursedWindow):
    X, Y = (0, 0)
    WIDTH, HEIGHT = 'max', 23

    # To create a menubar, you create a CursedMenu instance in a MENU
    # class variable.
    # You add a menu to it with the add_menu function, and specify the
    # title, optional hotkey, and a list of menu items in the form of
    # (name, hotkey, callback) or (name, callback).
    # Menu items can be chosen by hotkeys or by the arrow keys and enter.
    MENU = CursedMenu()
    MENU.add_menu('Movement', key='m', items=[
        ('Goto', 'g', 'goto'),
        ('Current Position', 'p', 'currentPosition'),
        ('Quit', 'q', 'quit'),
    ])
    MENU.add_menu('Vision', key='v', items=[
        ('Detections', 'd', 'detections'),
        ('Approach', 'a', 'approach')
    ])

    @classmethod
    def detections(cls):
        def float_formatter(x): return "%.1f" % x
        x, z, th = get_detections()
        logging.debug(
            "Person detected at %.2f %.2f, at heading %.2f",
            x,
            z,
            th)
        cls.addstr("Person detected at x={}m, y={}m, at heading {}degrees".format(str(
            float_formatter(x)), str(float_formatter(z)), str(float_formatter(th))), 0, 10)

    @classmethod
    def currentPosition(cls):
        updates.lock()
        str1 = "Mode: {} Status: {} Pos: ({},{},{})".format(
            updates.getMode(),
            updates.getStatus(),
            updates.getX(),
            updates.getY(),
            updates.getTh())
        updates.unlock()
        cls.addstr(str1, 0, 21)

    @classmethod
    def approach(cls):
        def float_formatter(x): return "%.1f" % x
        id = cls.getstr(10, 4, "index = ")
        cls.addstr('Approaching person {}.'.format(id), 0, 4)
        x, z, th = get_detections()
        cls.addstr("Person was last detected at x={}m, y={}m, at heading {}degrees".format(
            str(float_formatter(x)), str(float_formatter(z)), str(float_formatter(th))), 0, 5)
        currPose = get_position(updates)
        goto_position(currPose, x, z, th, client)

    @classmethod
    def goto(cls):
        x = float(cls.getstr(10, 12, "x = "))
        y = float(cls.getstr(10, 13, "y = "))
        th = float(cls.getstr(10, 14, "th = "))
        currPose = get_position(updates)
        goto_position(currPose, x, y, th, client)

    @classmethod
    def quit(cls):
        Aria.exit(0)

    @classmethod
    def update(cls):
        # The update function will be looped upon, so this is where you
        # want to put the main logic. This is what will check for key
        # presses, as well as trigger other functions through
        # cls.trigger.
        # Handle input here, other than what the menu will handle through
        # triggering callbacks itself.
        cls.trigger('currentPosition')


class FooterWindow(CursedWindow):
    # This window will appear on the bottom, print the string
    # "Press f then q to exit", then quit. The message will stay on the
    # screen.
    # All windows must have called 'quit' to exit out of the program, or
    # simply ctrl-C could be pressed.
    X, Y = (0, 23)
    WIDTH, HEIGHT = 'max', 1

    @classmethod
    def init(cls):
        cls.addstr('press m and then q to quit')
        cls.refresh()
        cls.trigger('quit')


def get_safe_position(x, y, th):
    # if(math.fabs(th) > 90):
        # TODO: check if the formulaes change
    r = SAFE_DISTANCE
    del_x = r * math.sin(math.radians(th))
    del_y = r * math.cos(math.radians(th))
    return x - del_x, y - del_y, th


def get_position_in_world_coordinates(
        robot_pose, position_in_robot_coordinates):
    x1, y1, theta1 = robot_pose[0], robot_pose[1], robot_pose[2]
    x2, y2 = position_in_robot_coordinates[0], position_in_robot_coordinates[1]

    r = math.hypot(x2, y2)
    theta1 = math.radians(theta1)
    theta2 = math.atan2(x2, y2)

    x = r * math.cos(theta1 + theta2)
    y = r * math.sin(theta1 + theta2)
    return x1 + x, y1 + y


def goto_position(currPose, x, z, th, client):
    # these coordinates are in robot's POV
    goal_x, goal_y, delta_heading = get_safe_position(x, z, th)
    # print("Safe position: " + str(goal_x) + " " + str(goal_y))
    scale = 1000  # 1 meter = 1000 steps for the robot, 1023 seems to be a experimental mistake, didn't take into account the deceleration time
    goal_x = goal_x * scale
    goal_y = goal_y * scale

    goal_X, goal_Y = get_position_in_world_coordinates(
        currPose, [goal_x, goal_y])

    goal_heading = currPose[2] + delta_heading
    goal_X, goal_Y, goal_heading = int(goal_X), int(goal_Y), int(goal_heading)

    logging.info(
        "[goto_position]: Changing goal to [%d, %d, %d].",
        goal_X,
        goal_Y,
        goal_heading)
    posePacket = ArNetPacket()
    posePacket.byte4ToBuf(goal_X)
    posePacket.byte4ToBuf(goal_Y)
    posePacket.byte4ToBuf(goal_heading)
    client.requestOnce("gotoPose", posePacket)


def get_position(updates):
    updates.lock()
    x = updates.getX()
    y = updates.getY()
    th = updates.getTh()
    logging.info(
        "Mode: %s Status: %s Pos: (%d,%d,%d)",
        updates.getMode(),
        updates.getStatus(),
        updates.getX(),
        updates.getY(),
        updates.getTh())
    updates.unlock()
    return [x, y, th]


def modeCallback(mode):
    print 'Mode changed'
    print mode


def statusCallback(status):
    print 'Status changed: ', status


Aria.init()
client = ArClientBase()

# You can change the hostname or ip address below to connect to a removve
# # server:
if not client.blockingConnect("localhost", SERVER_PORT):
    print "Could not connect to server at localhost port 7272, exiting"
    Aria.exit(1)
client.runAsync()

updates = ArClientHandlerRobotUpdate(client)
# updates.addStatusChangedCB(statusCallback)
# updates.addModeChangedCB(modeCallback)
updates.requestUpdates()

# while True:
#   currPose = get_position(updates)
#   ArUtil.sleep(2000)

app = CursedApp()
result = app.run()
logging.debug("Result of app.run is {}".format(result))
# This checks if ctrl-C or similar was pressed to kill the application.
if result.interrupted():
    Aria.exit(0)
    print('Ctrl-C pressed.')
else:
    # This will reraise exceptions that were raised in windows.
    result.unwrap()

import sys
import math
import logging
import requests
import argparse
import traceback
import signal
import ttk
from enum import Enum
from Tkinter import *
from PIL import ImageTk, Image
import tkMessageBox as messagebox
from goal_manager import GoalManager
from arnl_client import ArnlClient

# GUI Parameters
CHECKBOXES_UPDATE_INTERVAL = 500
DETECTIONS_UPDATE_INTERVAL = 1000
# Increase ROBOT_INFO_UPDATE_INTERVAL if it causes the robot_location_service to run slowly
# 100ms interval shouldn't be a problem as, genarally it takes 0.000028s to get pose, mode and status info even from ssh connection
STATUS_BARS_UPDATE_INTERVAL = 10
COMMAND_STATE_UPDATE_INTERVAL = 1000
NEIGHBOURHOOD_PREVIEW_IMAGE_FILE = '../data/neighbourhood_preview.jpg'
PATH_REPLAN_MIN_DISTANCE_LONG = 500
PATH_REPLAN_MIN_DISTANCE_SHORT = 100
LONG_DISTANCE = 2500


PERSON_LOCATION_SERVICE_URL = "http://localhost:5100/detections"


def configureLoggerAndArgparse():
    logging.basicConfig(
        # filename='robot_client.log',
        # filemode='w',
        format='%(levelname)s: %(message)s')
    _LOG_LEVEL_STRINGS = ['CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG']

    def _log_level_string_to_int(log_level_string):
        if log_level_string not in _LOG_LEVEL_STRINGS:
            message = 'invalid choice: {0} (choose from {1})'.format(
                log_level_string, _LOG_LEVEL_STRINGS)
            raise argparse.ArgumentTypeError(message)
        log_level_int = getattr(logging, log_level_string, logging.INFO)
        # check the logging log_level_choices have not changed from our expected values
        assert isinstance(log_level_int, int)
        return log_level_int

    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--host",
                    help="host ip of the arnl server")

    ap.add_argument("-l", "--log-level",
                    default='INFO',
                    dest='log_level',
                    type=_log_level_string_to_int,
                    nargs='?',
                    help='Set the logging output level. {0}'.format(_LOG_LEVEL_STRINGS))

    raw_args = ap.parse_args()
    args = vars(raw_args)

    logger = logging.getLogger()
    logger.setLevel(raw_args.log_level)

    if args.get('host'):
        ArnlClient.SERVER_HOST = args['host']


def get_formatted_distance(location):
    def float_formatter(x):
        return "%.1f" % x
    distance = math.sqrt(location[0] * location[0] +
                         location[1] * location[1] +
                         location[2] * location[2])
    return str(float_formatter(distance / 1000)) + ' meters'


def getFormattedPosition(position):
    def float_formatter(x):
        return "%.1f" % x
    return [float_formatter(x) for x in position]


class UserCommand(Enum):
    NO_COMMAND = 1
    APPROACH_POINT = 2
    APPROACH_PERSON = 3
    ABOUT_TO_APPROACH_PERSON_NAMED = 4
    APPROACH_MOVING_PERSON = 5
    APPROACH_MOVING_PERSON_NAMED = 6
    APPROACHING_PERSON_NAMED = 7


class GUI:
    def __init__(self, goalManager, arnlClient):
        self.arnlClient = arnlClient
        self.goalManager = goalManager
        try:
            self.USER_COMMAND_STATE = UserCommand.NO_COMMAND
            self.LAST_FRAME_RECEIVED_BY_UPDATE_DETECTED_PERSONS_LISTBOX = -1
            self.LAST_FRAME_RECEIVED_BY_UPDATE_COMMAND_STATE = -1
            self.setupGui()
            self.win.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.updateCheckBoxes()
            self.robotInfoUpdateCount = 0
            self.updateStatusBars()
            self.updateDetections()
            self.updateCommandState()
            self.win.mainloop()
        except Exception as e:
            traceback.print_exc(e)
            arnlClient.close()

    def updateStatusBars(self):
        mode, status, pose = self.arnlClient.getRobotInfo()
        robotInfoString = "Mode: {} Status: {} Pos: ({},{}) Heading: {}".format(
            mode,
            status,
            pose.getX(),
            pose.getY(),
            pose.getTh())
        logging.debug(
            "{} - {}".format(self.robotInfoUpdateCount, robotInfoString))
        self.robotInfoUpdateCount += 1
        self.robotInfoLabel.configure(text=robotInfoString)

        if self.USER_COMMAND_STATE == UserCommand.ABOUT_TO_APPROACH_PERSON_NAMED:
            # currentRobotPose = self.arnlClient.getPosition()
            # if (self.ROBOT_POSE_BEFORE_PATH_PLANNING != currentRobotPose):
            if status == ArnlClient.GOING_TO_GOAL_STRING:
                self.USER_COMMAND_STATE = UserCommand.APPROACHING_PERSON_NAMED
                logging.info("USER_COMMAND_STATE changed to {}".format(self.USER_COMMAND_STATE))

        if self.USER_COMMAND_STATE == UserCommand.APPROACHING_PERSON_NAMED and (status == ArnlClient.GOAL_REACHED_STRING or
                                                                                status == ArnlClient.GOAL_FAILED or
                                                                                status == ArnlClient.CANNOT_FIND_PATH_STRING):
            # path planning has finished its task, the arnl client may report a failure or a success, in both cases we need to stop giving commands
            logging.info("Resetting the state to NO_COMMAND")
            self.USER_COMMAND_STATE = UserCommand.NO_COMMAND
        self.statusLabel.configure(
            text="Status: {}".format(self.USER_COMMAND_STATE.name))
        self.win.update()
        self.win.after(STATUS_BARS_UPDATE_INTERVAL, self.updateStatusBars)

    def updateCommandState(self):
        def euclidean(p1, p2):
            return math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))
        if self.USER_COMMAND_STATE == UserCommand.APPROACHING_PERSON_NAMED:
            detections = getDetections()
            if (detections is not None):
                locations, names = detections
                for name, location in zip(names, locations):
                    if name == self.PERSON_TO_APPROACH:
                        robotPose, position, frameId = location
                        if frameId > self.LAST_FRAME_RECEIVED_BY_UPDATE_COMMAND_STATE:
                            self.LAST_FRAME_RECEIVED_BY_UPDATE_COMMAND_STATE = frameId
                            worldCoordinates = self.goalManager.getPositionInWorldCoordinates(
                                robotPose, position)
                            toReplan = False
                            if(euclidean(robotPose, worldCoordinates) > LONG_DISTANCE):
                                if euclidean(worldCoordinates, self.LAST_PERSON_COORDINATES) > PATH_REPLAN_MIN_DISTANCE_LONG:
                                    toReplan = True
                            else:
                                if euclidean(worldCoordinates, self.LAST_PERSON_COORDINATES) > PATH_REPLAN_MIN_DISTANCE_SHORT:
                                    toReplan = True
                            if toReplan:
                                logging.info("Frame[{}]: Changing goal for {}: {}(robot) or {}(world)".format(
                                    frameId, name, getFormattedPosition(position), worldCoordinates))
                                safeSpot = self.goalManager.findNearestSafeSpot(
                                    worldCoordinates)
                                if safeSpot is not None:
                                    heading = self.goalManager.getHeadingInDegrees(
                                        worldCoordinates, safeSpot)
                                    self.arnlClient.sendPathPlanRequest(
                                        safeSpot + (heading,))
                                    self.LAST_PERSON_COORDINATES = worldCoordinates
                                else:
                                    logging.warning("[UpdateCommandState] No safe spot found for the position {}.".format(
                                        personPositionInWorldCoordinates))
        self.win.after(COMMAND_STATE_UPDATE_INTERVAL, self.updateCommandState)

    def updateCheckBoxes(self):
        if self.personPresentCheckVar.get() == 0:
            self.movingPersonCheckButton.pack_forget()
            self.personNameCheckButton.pack_forget()
        else:
            self.movingPersonCheckButton.pack(side=LEFT)
            self.personNameCheckButton.pack(side=LEFT)
        if self.personNameCheckVar.get() == 0:
            self.personNameLabel.pack_forget()
            self.personNameEntry.pack_forget()
        else:
            self.personNameLabel.pack(side=LEFT)
            self.personNameEntry.pack(side=LEFT)
        self.win.after(CHECKBOXES_UPDATE_INTERVAL, self.updateCheckBoxes)

    def fillDetectedPersonsListbox(self, locations):
        self.detectedPersonsListbox.delete(0, END)
        for name, location in locations.iteritems():
            robotPose, position, frameId = location
            if frameId > self.LAST_FRAME_RECEIVED_BY_UPDATE_DETECTED_PERSONS_LISTBOX:
                self.LAST_FRAME_RECEIVED_BY_UPDATE_DETECTED_PERSONS_LISTBOX = frameId
                worldCoordinates = self.goalManager.getPositionInWorldCoordinates(
                    robotPose, position)
                logging.info("Detections: Robot at: {}, {} at: {}(robot) or {}(world) in frame {}".format(
                    robotPose, name, getFormattedPosition(position), worldCoordinates, frameId))
                self.detectedPersonsListbox.insert(END, "{0}: {1}".format(
                    name, get_formatted_distance(position)))

    def updateDetections(self):
        detections = getDetections()
        self.locations = {}
        if (detections is not None):
            locations, names = detections
            self.locations = {name: location for name,
                              location in zip(names, locations)}
        self.fillDetectedPersonsListbox(self.locations)
        self.win.update()
        self.win.after(DETECTIONS_UPDATE_INTERVAL, self.updateDetections)

    def sendRobot(self):
        x = None
        y = None
        try:
            x = int(self.xCoordinateStringVar.get())
            y = int(self.yCoordinateStringVar.get())
        except:
            messagebox.showerror(
                "Invalid input", "The coordinates x and y should be integers.")
            return

        coordinateFrame = self.coordinateFrameStringVar.get()
        goalString = ""
        isGoalAPerson = self.personPresentCheckVar.get()
        if isGoalAPerson:
            goalString += "a person"
            isPersonInMotion = self.movingPersonCheckVar.get()
            personHasName = self.personNameCheckVar.get()
            if personHasName:
                personName = self.personNameStringVar.get()
                goalString += " named {}".format(personName)
            if isPersonInMotion:
                goalString += ", who is in motion"
            else:
                goalString += ", who is stationary"
            if not personHasName:
                goalString += " (since, no name is provided, the robot will approach the person who appears nearest to that position)"
        else:
            goalString += "an empty spot"

        if messagebox.askokcancel("Confirmation", "Send the robot to ({},{}) expressed in {}, where the goal is {}?".format(x, y, coordinateFrame, goalString)):
            logging.info("Sending the robot to ({},{}) expressed in {}, where the goal is {}.".format(
                x, y, coordinateFrame, goalString))
            personPositionInWorldCoordinates = (x, y)
            currentRobotPose = None
            if coordinateFrame == self.COORDINATE_FRAMES[0]:
                currentRobotPose = self.arnlClient.getPosition()
                personPositionInWorldCoordinates = self.goalManager.getPositionInWorldCoordinates(
                    currentRobotPose, (x, y))
            if isGoalAPerson:
                isPersonInMotion = self.movingPersonCheckVar.get()
                personHasName = self.personNameCheckVar.get()
                self.LAST_PERSON_COORDINATES = personPositionInWorldCoordinates
                if currentRobotPose is None:
                    self.ROBOT_POSE_BEFORE_PATH_PLANNING = self.arnlClient.getPosition()
                if isPersonInMotion:
                    pass
                elif personHasName:
                    self.PERSON_TO_APPROACH = self.personNameStringVar.get()
                else:
                    self.USER_COMMAND_STATE = UserCommand.APPROACH_PERSON
                safeSpot = self.goalManager.findNearestSafeSpot(
                    personPositionInWorldCoordinates)
                if safeSpot is not None:
                    heading = self.goalManager.getHeadingInDegrees(
                        personPositionInWorldCoordinates, safeSpot)
                    self.arnlClient.sendPathPlanRequest(
                        safeSpot + (heading,))
                    logging.info("USER_COMMAND_STATE changed to {}".format(UserCommand.ABOUT_TO_APPROACH_PERSON_NAMED.name))
                    self.USER_COMMAND_STATE = UserCommand.ABOUT_TO_APPROACH_PERSON_NAMED
                else:
                    logging.warning("No safe spot found for the position {}.".format(
                        personPositionInWorldCoordinates))
            else:
                self.USER_COMMAND_STATE = UserCommand.APPROACH_POINT
                self.arnlClient.sendPathPlanRequest(
                    personPositionInWorldCoordinates + (0,))

    def findSafePosition(self):
        x = None
        y = None
        try:
            x = int(self.xCoordinateStringVar.get())
            y = int(self.yCoordinateStringVar.get())
        except:
            messagebox.showerror(
                "Invalid input", "The coordinates x and y should be integers.")
            return
        coordinateFrame = self.coordinateFrameStringVar.get()
        personPositionInWorldCoordinates = (x, y)
        if coordinateFrame == self.COORDINATE_FRAMES[0]:
            currentRobotPose = self.arnlClient.getPosition()
            personPositionInWorldCoordinates = self.goalManager.getPositionInWorldCoordinates(
                currentRobotPose, (x, y))
        safeSpots = self.goalManager.findSafeSpots(
            personPositionInWorldCoordinates)
        if (len(safeSpots) != 0):
            messagebox.showinfo("Find safe spots", "{} at {}, of which the nearest one to the robot is {}".format(
                safeSpots, personPositionInWorldCoordinates, self.goalManager.findNearestSafeSpot(personPositionInWorldCoordinates, safeSpots=safeSpots)))
        else:
            messagebox.showinfo(
                "Find safe spots", "No safe spot found around the person present at {}, to place the robot".format(personPositionInWorldCoordinates))

    def safeToPlaceCheck(self):
        x = None
        y = None
        try:
            x = int(self.xCoordinateStringVar.get())
            y = int(self.yCoordinateStringVar.get())
        except:
            messagebox.showerror(
                "Invalid input", "The coordinates x and y should be integers.")
            return
        coordinateFrame = self.coordinateFrameStringVar.get()
        positionInWorldCoordinates = (x, y)
        if coordinateFrame == self.COORDINATE_FRAMES[0]:
            currentRobotPose = self.arnlClient.getPosition()
            positionInWorldCoordinates = self.goalManager.getPositionInWorldCoordinates(
                currentRobotPose, (x, y))
        result = self.goalManager.gridManager.isRobotSafeToPlace(
            positionInWorldCoordinates)
        messagebox.showinfo("Safe to place?", "{} at {}".format(
            result, positionInWorldCoordinates))

    def setupPreviewPointFrame(self, previewImage):
        previewWindow = Toplevel(self.win)
        previewWindow.title("Preview Neighbourhood")
        neighbourhoodPreviewImageLabel = Label(
            previewWindow, image=previewImage, borderwidth=5, relief=SOLID)
        neighbourhoodPreviewImageLabel.image = previewImage
        neighbourhoodPreviewImageLabel.pack(
            side=BOTTOM, fill="both", expand="yes")

    def showNeighbourhoodPreview(self):
        x = None
        y = None
        try:
            x = int(self.xCoordinateStringVar.get())
            y = int(self.yCoordinateStringVar.get())
        except:
            messagebox.showerror(
                "Invalid input", "The coordinates x and y should be integers.")
            return
        coordinateFrame = self.coordinateFrameStringVar.get()
        positionInWorldCoordinates = (x, y)
        if coordinateFrame == self.COORDINATE_FRAMES[0]:
            currentRobotPose = self.arnlClient.getPosition()
            positionInWorldCoordinates = self.goalManager.getPositionInWorldCoordinates(
                currentRobotPose, (x, y))
        if self.goalManager.gridManager.saveNeighbourhoodImage(positionInWorldCoordinates, 500, NEIGHBOURHOOD_PREVIEW_IMAGE_FILE):
            previewImage = ImageTk.PhotoImage(
                Image.open(NEIGHBOURHOOD_PREVIEW_IMAGE_FILE))
            self.setupPreviewPointFrame(previewImage)
        else:
            messagebox.showerror(
                "Invalid input", "Region does not lie inside grid.")

    def setupDetectedPersonsFrame(self, parentComponent):
        # TODO make it a table with the last detected time, position, orientation for each person
        self.detectedPersonsFrame = Frame(parentComponent)
        self.detectedPersonsFrame.pack()
        Label(self.detectedPersonsFrame,
              text="Detected Persons").pack(side=TOP, fill=X)
        scroll = Scrollbar(self.detectedPersonsFrame, orient=VERTICAL)
        self.detectedPersonsListbox = Listbox(self.detectedPersonsFrame, yscrollcommand=scroll.set,
                                              height=6, activestyle='none', selectmode='browse')
        scroll.config(command=self.detectedPersonsListbox.yview)
        scroll.pack(side=RIGHT, fill=Y)
        self.detectedPersonsListbox.pack(side=LEFT, fill=BOTH, expand=1)

    def setupSendRobotFrame(self, parentComponent):
        # coordinateEntryFrame
        self.coordinateEntryFrame = Frame(parentComponent)
        self.coordinateEntryFrame.pack()

        Label(self.coordinateEntryFrame, text="X (in mm):").pack(
            side=LEFT, ipadx=10)
        self.xCoordinateStringVar = StringVar()
        self.xCoordinateEntry = Entry(
            self.coordinateEntryFrame, textvariable=self.xCoordinateStringVar)
        self.xCoordinateEntry.pack(side=LEFT)

        Label(self.coordinateEntryFrame, text="Y (in mm):").pack(
            side=LEFT, ipadx=10)
        self.yCoordinateStringVar = StringVar()
        self.yCoordinateEntry = Entry(
            self.coordinateEntryFrame, textvariable=self.yCoordinateStringVar)
        self.yCoordinateEntry.pack(side=LEFT)

        # coordinateOptionsFrame
        self.coordinateOptionsFrame = Frame(parentComponent)
        self.coordinateOptionsFrame.pack()

        Label(self.coordinateOptionsFrame,
              text="Select coordinate frame").pack(side=LEFT)
        self.COORDINATE_FRAMES = [
            "Robot Coordinate Frame", "World Coordinate Frame"]

        self.coordinateFrameStringVar = StringVar(self.coordinateOptionsFrame)
        self.coordinateFrameStringVar.set(self.COORDINATE_FRAMES[1])
        self.coordinateFrameSelectorMenu = OptionMenu(
            self.coordinateOptionsFrame, self.coordinateFrameStringVar, *self.COORDINATE_FRAMES)
        self.coordinateFrameSelectorMenu.pack(side=LEFT)

        # personOptionsFrame
        self.personOptionsFrame = Frame(parentComponent)
        self.personOptionsFrame.pack()

        self.personPresentCheckVar = IntVar()
        self.personPresentCheckButton = Checkbutton(self.personOptionsFrame, text="Is goal a person?", variable=self.personPresentCheckVar,
                                                    onvalue=1, offvalue=0)
        self.personPresentCheckButton.pack(side=TOP)

        self.movingPersonCheckVar = IntVar()
        self.movingPersonCheckButton = Checkbutton(self.personOptionsFrame, text="Is person in motion?", variable=self.movingPersonCheckVar,
                                                   onvalue=1, offvalue=0)
        self.movingPersonCheckButton.pack(side=LEFT)

        self.personNameCheckVar = IntVar()
        self.personNameCheckButton = Checkbutton(self.personOptionsFrame, text="Does person have a name?", variable=self.personNameCheckVar,
                                                 onvalue=1, offvalue=0)
        self.personNameCheckButton.pack(side=LEFT)

        self.personNameLabel = Label(
            self.personOptionsFrame, text="Person Name: ")
        self.personNameLabel.pack(side=LEFT)
        self.personNameStringVar = StringVar()
        self.personNameEntry = Entry(
            self.personOptionsFrame, textvariable=self.personNameStringVar)
        self.personNameEntry.pack(side=LEFT)

        # buttonsFrame
        self.sendRobotTabButtonsFrame = Frame(parentComponent)
        self.sendRobotTabButtonsFrame.pack()

        self.showPreviewButton = Button(self.sendRobotTabButtonsFrame, text="Show Neighbourhood Preview",
                                        borderwidth=2, relief=RAISED, command=self.showNeighbourhoodPreview)
        self.showPreviewButton.pack(side=TOP)

        self.sendRobotButton = Button(
            self.sendRobotTabButtonsFrame, text="Send", borderwidth=2, relief=RAISED, command=self.sendRobot)
        self.sendRobotButton.pack(side=LEFT)

        self.safeToPlaceButton = Button(self.sendRobotTabButtonsFrame, text="Safe to place?",
                                        borderwidth=2, relief=RAISED, command=self.safeToPlaceCheck)
        self.safeToPlaceButton.pack(side=LEFT)

        self.findSafePositionButton = Button(
            self.sendRobotTabButtonsFrame, text="Find safe position", borderwidth=2, relief=RAISED, command=self.findSafePosition)
        self.findSafePositionButton.pack(side=LEFT)

    def setupStatusFrame(self):
        self.statusFrame = Frame(self.win)
        self.statusFrame.pack()
        self.statusLabel = Label(self.statusFrame, text="Status: " + self.USER_COMMAND_STATE.name,
                                 bd=1, relief=SOLID, anchor=W)
        self.statusLabel.pack(side=BOTTOM, fill=X)

        # TODO make a label for each of the robot's information, to make updating easier, and put them all in a frame
        self.robotInfoLabel = Label(self.statusFrame, text="Robot: ...",
                                    bd=1, relief=SOLID, anchor=W)
        self.robotInfoLabel.pack(side=BOTTOM, fill=X)

    def setupControlsFrame(self, parentComponent):
        # buttonsFrame
        self.controlsTabButtonsFrame = Frame(parentComponent)
        self.controlsTabButtonsFrame.pack()

        self.stopRobotButton = Button(
            self.controlsTabButtonsFrame, text="Stop Robot", borderwidth=2, relief=RAISED)
        self.stopRobotButton.pack(side=LEFT)

    def setupGui(self):
        self.win = Tk()
        self.win.title("Robot Controller")

        self.tabControl = ttk.Notebook(self.win)

        self.sendRobotTab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.sendRobotTab, text='Send Robot')
        self.tabControl.pack(expand=1, fill="both")
        self.setupSendRobotFrame(self.sendRobotTab)

        self.detectedPersonsTab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.detectedPersonsTab, text='Detected Persons')
        self.tabControl.pack(expand=1, fill="both")
        self.setupDetectedPersonsFrame(self.detectedPersonsTab)

        self.robotControlsTab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.robotControlsTab, text='Controls')
        self.tabControl.pack(expand=1, fill="both")
        self.setupControlsFrame(self.robotControlsTab)

        self.setupStatusFrame()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            arnlClient.close()
            self.win.destroy()


def getDetections():
    try:
        response = requests.get(PERSON_LOCATION_SERVICE_URL)
        # Handle error if deserialization fails (because of no text or bad format)
        try:
            data = response.json()
            # check that .json() did NOT return an empty dict
            if data:
                try:
                    positions = data['locations']
                    names = data['names']
                    return positions, names
                except (IndexError, KeyError, TypeError):
                    # data does not have the inner structure you expect
                    logging.error("data does not have the expected structure")
            else:
                logging.warning("response.json() returned an empty dict")

        except ValueError:
            # no JSON returned
            logging.error("no JSON received...")
    except requests.exceptions.ConnectionError:
        logging.error("Can't connect to " + PERSON_LOCATION_SERVICE_URL)
    return None


# To handle Ctrl + C
def signal_handler(sig, frame):
    arnlClient.close()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    configureLoggerAndArgparse()
    goalManager = GoalManager()
    arnlClient = ArnlClient()
    gui = GUI(goalManager, arnlClient)

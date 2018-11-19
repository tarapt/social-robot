#include <iostream>
#include "Aria.h"

void printStatus(ArRobot& robot) {
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();
}

void stop(ArRobot& robot) {
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);
}

/*
 * speed in mm/s,
 * time in seconds.
 */
void move(ArRobot& robot, double speed, double time) {
  // Set forward velocity to 50 mm/s
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at %.2f mm/s for %.2f sec...", speed, time);
  robot.lock();
  robot.enableMotors();
  robot.setRotVel(0);
  robot.setVel(speed);
  robot.unlock();
  ArUtil::sleep(int(time * 1000)); // in milliseconds

  stop(robot);
}

/*
 * speed in mm/s,
 * distance in mm.
 */
void moveDistance(ArRobot& robot, int distance, double speed) {
  double time = distance / speed;
  move(robot, speed, time);
}

void rotate(ArRobot& robot, double rotVel, double time) {
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at %.2f deg/s for %.2f sec...", rotVel, time);
  robot.lock();
  robot.setRotVel(rotVel);
  robot.unlock();
  ArUtil::sleep(int(time * 1000));

  stop(robot);
}

void rotateAngle(ArRobot& robot, double rotVel, double angle) {
  double time = fabs(angle/rotVel);
  if(angle < 0)
    rotVel *= -1;
  rotate(robot, rotVel, time);
}

// transVel = translationalVelocity
void moveTowardsTarget(ArRobot& robot, ArPose target, double transVel, double rotVel) {
  double delta = robot.findDeltaHeadingTo(target);
  double distance = (int)robot.findDistanceTo(target);
  rotateAngle(robot, rotVel, delta);
  moveDistance(robot, distance, transVel);
}

int main(int argc, char **argv)
{
  int x, y; 
  std::cout << "x: "; std::cin >> x;
  std::cout << "y: "; std::cin >> y;
  Aria::init();
  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  ArLog::log(ArLog::Terse, "WARNING: this program does no sensing or avoiding of obstacles, the robot WILL collide with any objects in the way! Make sure the robot has approximately 3 meters of free space on all sides.");

  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }
  
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Connected.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  

  // We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  // Make sure you unlock before any sleep() call or any other code that will
  // take some time; if the robot remains locked during that time, then
  // ArRobot's background thread will be blocked and unable to communicate with
  // the robot, call tasks, etc.
  
  printStatus(robot);

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Will start driving in 3 seconds...");
  ArUtil::sleep(3000);

  moveTowardsTarget(robot, ArPose(x, y, 0), 300, 10);
  
  printStatus(robot);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}

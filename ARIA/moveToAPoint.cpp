#include "Aria.h"
#include <iostream>

void printStatus(ArRobot& robot) {
    robot.lock();
    ArLog::log(ArLog::Normal, "moveToAPoint: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV", robot.getX(), robot.getY(), robot.getTh(), 
    robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
    robot.unlock();
}

void moveTo(ArRobot& robot, ArPose target, int speed) {
  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "moveToAPoint: Will start driving in 3 seconds...");
  ArUtil::sleep(3000);

  // Rotate towards the target
  ArLog::log(ArLog::Normal, "moveToAPoint: Rotating desired degrees towards the target...");
  robot.lock();
  robot.enableMotors();
  int delta = robot.findDeltaHeadingTo(target) * 1000;
  ArLog::log(ArLog::Normal, "moveToAPoint: delta = %d", delta);
  robot.setRotVel(10);
  robot.unlock();
  ArUtil::sleep(delta/10);
  printStatus(robot);

  ArLog::log(ArLog::Normal, "moveToAPoint: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);
  printStatus(robot);

  // Move towards the target
  ArLog::log(ArLog::Normal, "moveToAPoint: Driving towards the target at %d mm/s...", 100);
  robot.lock();
  int distance = (int)robot.findDistanceTo(target) * 1000;
  ArLog::log(ArLog::Normal, "moveToAPoint: distance = %d, time = %d milliseconds", distance, distance/100);
  robot.setRotVel(0);
  robot.setVel(100);
  ArUtil::sleep(distance/100);
  printStatus(robot);

  ArLog::log(ArLog::Normal, "moveToAPoint: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);
  printStatus(robot);
}

int main(int argc, char **argv)
{

  int x = 900, y = 400, speed = 150; 
  // std::cout << "X: "; std::cin >> x;
  // std::cout << "Y: "; std::cin >> y;
  // std::cout << "Speed: "; std::cin >> speed;

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
    ArLog::log(ArLog::Terse, "moveToAPoint: Could not connect to the robot.");
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
  
  ArLog::log(ArLog::Normal, "moveToAPoint: Connected.");

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

  moveTo(robot, ArPose(x, y, 0), speed);

  ArLog::log(ArLog::Normal, "moveToAPoint: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "moveToAPoint: Exiting.");
  Aria::exit(0);
  return 0;
}

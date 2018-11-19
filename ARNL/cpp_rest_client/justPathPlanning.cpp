#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"

/** @example justPathPlanning.cpp Example which does path planning and
following only, but provides servers for remote operation with MobileEyes.
 
This is provided as an example 
to build on for developing your own localization techniques. Note, the 
path planning and following will become innacurate without correction
of the robot pose from a localization process (use ArRobot::moveTo() to
correct the robot pose).
*/



/** Main function */
int 
main(int argc, char *argv[])
{
  // Initialize location of Aria, Arnl and their args.
  Aria::init();
  Arnl::init();
  
  // To initialize, but log to a file, do this instead of the above:
  //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);
  //ArLog::init(ArLog::File, ArLog::Verbose);
 
  // The robot object
  ArRobot robot;

  // Our server
  ArServerBase server;
  
  // Parse the command line arguments.
  ArArgumentParser parser(&argc, argv);

  // Set up our simpleConnector
  ArSimpleConnector simpleConnector(&parser);

  // Set up our simpleOpener
  ArServerSimpleOpener simpleOpener(&parser);

  // Set up our client for the central server
  ArClientSwitchManager clientSwitch(&server, &parser);
    
  // Load default arguments for this computer (from /etc/Aria.args, environment
  // variables, and other places)
  parser.loadDefaultArguments();

  // set up a gyro
  ArAnalogGyro gyro(&robot);
  
  // Parse arguments for the simple connector.
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    ArLog::log(ArLog::Normal, "\nUsage: %s -map mapfilename\n", argv[0]);
    Aria::logOptions();
    Aria::exit(1);
  }


  // The laser object, will be used if we have one
  ArSick sick;

  // Add the laser to the robot
  robot.addRangeDevice(&sick);

  // Sonar, must be added to the robot, used by teleoperation and wander to
  // detect obstacles, and for localization if SONARNL
  ArSonarDevice sonarDev;

  // Add the sonar to the robot
  robot.addRangeDevice(&sonarDev);
  
  // Set up where we'll look for files
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "examples");
  ArLog::log(ArLog::Normal, "Installation directory is: %s\nMaps directory is: %s\n", Aria::getDirectory(), fileDir);
  
  // Set up the map, this will look for files in the examples
  // directory (unless the file name starts with a /, \, or .
  // You can take out the 'fileDir' argument to look in the current directory
  // instead
  ArMap arMap(fileDir);
  // set it up to ignore empty file names (otherwise the parseFile
  // on the config will fail)
  arMap.setIgnoreEmptyFileName(true);

  // Make the path task planning task
  ArPathPlanningTask pathTask(&robot, &sick, &sonarDev, &arMap);

  // Set up things so data can be logged (only do it with the laser
  // since it can overrun a 9600 serial connection which the sonar is
  // more likely to have)
  ArDataLogger dataLogger(&robot);
  dataLogger.addToConfig(Aria::getConfig());

  // add our logging to the config
  ArLog::addToConfig(Aria::getConfig());

  // First open the server 
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    if (simpleOpener.wasUserFileBad())
      ArLog::log(ArLog::Normal, "Bad user file");
    else
      ArLog::log(ArLog::Normal, "Could not open server port");
    exit(2);
  }

  // Connect the robot
  if (!simpleConnector.connectRobot(&robot))
  {
    ArLog::log(ArLog::Normal, "Could not connect to robot... exiting");
    Aria::exit(3);
  }

  // Set up a class that'll put the movement and gyro parameters into ArConfig
  ArRobotConfig robotConfig(&robot);
  robotConfig.addAnalogGyro(&gyro);

  robot.enableMotors();
  robot.clearDirectMotion();

  // if we are connected to a simulator, reset it to its start position
  robot.comInt(ArCommands::RESETSIMTOORIGIN, 1);

  // Set up laser using connector (command line arguments, etc.)
  simpleConnector.setupLaser(&sick);

  // Start the robot thread.
  robot.runAsync(true);
  
  // Start the laser thread.
  sick.runAsync();

  // Try to connect the laser
  if (!sick.blockingConnect())
    ArLog::log(ArLog::Normal, "Warning: Couldn't connect to SICK laser, it won't be used");
  else
    ArLog::log(ArLog::Normal, "Connected to laser.");


  // Add additional range devices to the robot and path planning task.
  // IRs if the robot has them.
  robot.lock();
  ArIRs irs;
  robot.addRangeDevice(&irs);
  pathTask.addRangeDevice(&irs, ArPathPlanningTask::CURRENT);

  // Bumpers.
  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);
  pathTask.addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);

  // Forbidden regions from the map
  ArForbiddenRangeDevice forbidden(&arMap);
  robot.addRangeDevice(&forbidden);
  pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);

  // This is the place to add a range device which will hold sensor data
  // and delete it appropriately to replan around blocked paths.
  ArGlobalReplanningRangeDevice replanDev(&pathTask);

  // Create objects that add network services:
  
  // Drawing in the map display:
  ArServerInfoDrawings drawings(&server);
  drawings.addRobotsRangeDevices(&robot);
  drawings.addRangeDevice(&replanDev);

  /* If you want to draw the destination put this code back in:
  ArServerDrawingDestination destination(
	  &drawings, &pathTask, "destination",
	  500, 500,
	  new ArDrawingData("polyDots",
			    ArColor(0xff, 0xff, 0x0),
			    800, // size
			    49), // just below the robot
  */

  /* If you want to see the local path planning area use this 
    (You can enable this particular drawing from custom commands 
    which is set up down below in ArServerInfoPath) 
  ArDrawingData drawingDataP("polyLine", ArColor(200,200,200), 1, 75);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> 
  drawingFunctorP(pathTask, &ArPathPlanningTask::drawSearchRectangle);
  drawings.addDrawing(&drawingDataP, "Local Plan Area", &drawingFunctorP); 
  */

  /* If you want to see the points making up the local path in addition to the
   * main path use this. 
  ArDrawingData drawingDataP2("polyDots", ArColor(0,128,0), 100, 70);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> 
  drawingFunctorP2(pathTask, &ArPathPlanningTask::drawPathPoints);
  drawings.addDrawing(&drawingDataP2, "Path Points", &drawingFunctorP2);
  */

  // Misc. simple commands:
  ArServerHandlerCommands commands(&server);


  // These provide various kinds of information to the client:
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoPath serverInfoPath(&server, &robot, &pathTask);
  serverInfoPath.addSearchRectangleDrawing(&drawings);
  serverInfoPath.addControlCommands(&commands);

  // Provide the map to the client (and related controls):
  // This uses both lines and points now, since everything except
  // sonar localization uses both (path planning with sonar still uses both)
  ArServerHandlerMap serverMap(&server, &arMap);

  

  // Add some simple (custom) commands for testing and debugging:
  ArServerSimpleComUC uCCommands(&commands, &robot);                   // Send any command to the microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // configure logging
  ArServerSimpleComGyro gyroCommands(&commands, &robot, &gyro);        // monitor the gyro
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot);   // trigger logging of the robot config parameters
  ArServerSimpleServerCommands serverCommands(&commands, &server);     // monitor networking behavior (track packets sent etc.)


  /* Set up the possible modes for remote control from a client such as
   * MobileEyes:
   */

  // Mode To go to a goal or other specific point:
  ArServerModeGoto modeGoto(&server, &robot, &pathTask, &arMap, ArPose(0,0,0));

  // Add a simple (custom) command that allows you to give a list of 
  // goals to tour, instead of all. Useful for testing and debugging.
  modeGoto.addTourGoalsInListSimpleCommand(&commands);

  // Mode To stop and remain stopped:
  ArServerModeStop modeStop(&server, &robot);

  // cause the sonar to turn off automatically
  // when the robot is stopped, and turn it back on when commands to move
  // are sent. (Note, this should not be done if you need the sonar
  // data to localize, or for other purposes while stopped)
  ArSonarAutoDisabler sonarAutoDisabler(&robot);

  // Teleoperation modes To drive by keyboard, joystick, etc:
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  // New, improved mode
  ArServerModeDrive modeDrive(&server, &robot);            // Older mode for compatability

  // Drive mode's configuration and custom (simple) commands:
  modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings");
  modeDrive.addControlCommands(&commands);
  modeRatioDrive.addControlCommands(&commands);

  // Wander mode 
  ArServerModeWander modeWander(&server, &robot);


  // This provides a small table of interesting information for the client
  // to display to the operator:
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  
  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(&robot, 
					       &ArRobot::getMotorPacCount));


  // Make Stop mode the default (If current mode deactivates without entering
  // a new mode, then Stop Mode will be selected)
  modeStop.addAsDefaultMode();




  /* File transfer services: */
  
#ifdef WIN32
  // these server file things don't work under windows yet
  ArLog::log(ArLog::Normal, "Note, file upload/download services are not implemented for Windows; not enabling them.");
#else
  // This block will allow you to set up where you get and put files
  // to/from, just comment them out if you don't want this to happen
  // /*
  ArServerFileLister fileLister(&server, fileDir);
  ArServerFileToClient fileToClient(&server, fileDir);
  ArServerFileFromClient fileFromClient(&server, fileDir, "/tmp");
  ArServerDeleteFileOnServer deleteFileOnServer(&server, fileDir);
  // */
#endif

  // Create the service that allows the client to monitor the communication 
  // between the robot and the client.
  //
  ArServerHandlerCommMonitor handlerCommMonitor(&server);

  // Create service that allows client to change configuration parameters in ArConfig 
  ArServerHandlerConfig handlerConfig(&server, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());


  
  // Read in parameter files.
  Aria::getConfig()->useArgumentParser(&parser);
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal, "Trouble loading configuration file, exiting");
    Aria::exit(5);
  }

  // Warn about unknown params.
  if (!simpleOpener.checkAndLog() || !parser.checkHelpAndWarnUnparsed())
  {
    ArLog::log(ArLog::Normal, "\nUsage: %s -map mapfilename\n", argv[0]);
    simpleConnector.logOptions();
    simpleOpener.logOptions();
    Aria::exit(6);
  }

  // Warn if there is no map
  if (arMap.getFileName() == NULL || strlen(arMap.getFileName()) <= 0)
  {
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "### Warning, No map file is set up, you can make a map with sickLogger or arnlServer, and Mapper3; More info in docs/Mapping.txt and README.txt. Set the map with the -map command line option, or by changing the config with MobileEyes or by editing the config file.");
    ArLog::log(ArLog::Normal, "");    
  }

  // find out where we'll want to put files
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal, 
	     "Directory for maps and file serving: %s", fileDir);
  
  ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
  ArLog::log(ArLog::Normal, "");

  // If you want MobileSim to try and load up the same map as you are
  // using in guiServer then uncomment out the next line and this object
  // will send a command to MobileSim to do so, but make sure you start 
  // MobileSim from the Arnl/examples directory or use the --cwd option, 
  // so that the map names used by MobileSim match  the map names used 
  // by guiServer
  //ArSimMapSwitcher mapSwitcher(&robot, &arMap);
  
  if(arMap.findMapObject("Home") != nullptr)
    robot.moveTo(arMap.findMapObject("Home")->getPose());
  else
    printf("Couldn't reset to the home position.\n");

  /* Finally, get ready to run the robot: */

  robot.unlock();

  server.runAsync();

  // Add a key handler (mostly so that on windows you can exit by pressing
  // escape.) This key handler, however, prevents this program from
  // running in the background (e.g. as a system daemon or run from 
  // the shell with "&") -- it will lock up trying to read the keys; 
  // remove this if you wish to be able to run this program in the background.
  ArKeyHandler *keyHandler;
  if ((keyHandler = Aria::getKeyHandler()) == NULL)
  {
    keyHandler = new ArKeyHandler;
    Aria::setKeyHandler(keyHandler);
    robot.lock();
    robot.attachKeyHandler(keyHandler);
    robot.unlock();
    printf("To exit, press escape.\n");
  }

  robot.waitForRunExit();
  Aria::exit(0);
}

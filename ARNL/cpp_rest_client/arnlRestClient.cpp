#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientHandlerRobotUpdate.h"

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/fusion/adapted.hpp>

#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup.hpp>

#include <restc-cpp/restc-cpp.h>
#include <restc-cpp/RequestBuilder.h>

using namespace std;
using namespace restc_cpp;
namespace logging = boost::log;

// C++ structure that match the JSON entries received
// struct Post {
//     string name;
//     vector<double> position;
//     vector<double> theta;
// };

// Since C++ does not (yet) offer reflection, we need to tell the library how
// to map json members to a type. We are doing this by declaring the
// structs/classes with BOOST_FUSION_ADAPT_STRUCT from the boost libraries.
// This allows us to convert the C++ classes to and from JSON.
// BOOST_FUSION_ADAPT_STRUCT(
//     Post,
//     (string, name)
//     (vector<double>, position)
//     (vector<double>, theta)
// )

void handlePathPlannerStatus(ArNetPacket *packet) {
  char buf[64];
  packet->bufToStr(buf, 63);
  printf(".. Path planner status: \"%s\"\n", buf);
}

void handleGoalName(ArNetPacket* packet) {
  char buf[64];
  packet->bufToStr(buf, 63);
  printf(".. Current goal: \"%s\"\n", buf);
}

void handleRobotUpdate(ArNetPacket* packet) {
  char buf[64];
  packet->bufToStr(buf, 63);
  printf(".. Robot server status: \"%s\"\n", buf);
  packet->bufToStr(buf, 63);
  printf(".. Robot server mode: \"%s\"\n", buf);
}

void handleGoalList(ArNetPacket *packet) {
  printf(".. Server has these goals:\n");
  char goal[256];
  for(int i = 0; packet->getReadLength() < packet->getLength(); i++) {
    packet->bufToStr(goal, 255);
    if(strlen(goal) == 0)
        return;
    printf("      %s\n", goal);
  }
}
    
int main(int argc, char **argv) {
  Aria::init();
  ArClientBase client;
  ArArgumentParser parser(&argc, argv);
  ArClientSimpleConnector clientConnector(&parser);
  parser.loadDefaultArguments();

  if (!clientConnector.parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
    clientConnector.logOptions();
    exit(0);
  }
  
  printf("Connecting...\n");
  if (!clientConnector.connectClient(&client)) {
    if (client.wasRejected())
      printf("Server rejected connection, exiting\n");
    else
      printf("Could not connect to server, exiting\n");
    exit(1);
  } 

  ArClientHandlerRobotUpdate robotUpdates(&client);
  robotUpdates.requestUpdates();  // Won't work without this

  printf("Connected to server.\n");
  // client.addHandler("pathPlannerStatus", new ArGlobalFunctor1<ArNetPacket*>(&handlePathPlannerStatus));
  // client.addHandler("update", new ArGlobalFunctor1<ArNetPacket*>(&handleRobotUpdate));
  // client.addHandler("goalName", new ArGlobalFunctor1<ArNetPacket*>(&handleGoalName));
  // client.addHandler("getGoals", new  ArGlobalFunctor1<ArNetPacket*>(&handleGoalList));
  client.runAsync();
  // client.requestOnce("getGoals");
  // client.request("pathPlannerStatus", 5000);
  // client.request("goalName", 5000);
  // client.request("update", 5000);
  while(client.getRunningWithLock()) {
    char goal[128];
    printf("=> Enter a goal name, # for position or * to tour all goals, or ? to list goals.\n");
    if( fgets(goal, 127, stdin) == NULL ) { // EOF
        printf("goodbye.\n");
        Aria::shutdown();
        return 0;
    }
    goal[strlen(goal)-1] = '\0';  // remove newline
    if(strcmp(goal, "?") == 0) {
      client.requestOnce("getGoals");
      printf("=> Enter a goal name, # for position or * to tour all goals, or ? to list goals.\n");
    } else if(strcmp(goal, "*") == 0) {
      if(client.dataExists("tourGoals")) {
        printf("=> Touring all goals...\n");
        client.requestOnce("tourGoals");
      } else {
        printf("=> Can't tour goals, server does not have that request.\n");
      }
    } else if(goal[0] == '#') {
      ArPose currPose = robotUpdates.getPose();
      char c;
      int x_offset, y_offset;
      sscanf(goal, "%c%d%d\n", &c, &x_offset, &y_offset);
      ArNetPacket posePacket;
      posePacket.byte4ToBuf(int(currPose.getX()) + x_offset);
      posePacket.byte4ToBuf(int(currPose.getY()) + y_offset);
      printf("=> Current pose is %d %d...\n", int(currPose.getX()), int(currPose.getY()));
      printf("=> Going to pose: %d %d...\n", int(currPose.getX()) + x_offset, int(currPose.getY()) + y_offset);
      client.requestOnce("gotoPose", &posePacket);
    } else {
      printf("=> Sending goal \"%s\" to server...\n", goal);
      client.requestOnceWithString("gotoGoal", goal);
    }
  }
  printf("Server disconnected.\n");
  Aria::shutdown();
  return 0;
}
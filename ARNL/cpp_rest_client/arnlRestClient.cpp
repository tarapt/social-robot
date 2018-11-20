#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientHandlerRobotUpdate.h"

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

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

void gotoPose(int x, int y, ArClientBase client) {
  ArNetPacket posePacket;
  posePacket.byte4ToBuf(x);
  posePacket.byte4ToBuf(y);
  printf("=> Going to pose: %d %d...\n", x, y);
  client.requestOnce("gotoPose", &posePacket);
}

void followPerson(string name, ArClientBase client, ArClientHandlerRobotUpdate robotUpdates) {
  while(client.getRunningWithLock()) {
    // get location of the person
    int x, y;
    ArPose currPose = robotUpdates.getPose();
    x += currPose.getX();
    y += currPose.getY();
    gotoPose(x, y, client);
    this_thread::sleep_for (chrono::seconds(5));
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
  client.runAsync();

  followPerson("tara_prasad", client, robotUpdates);

  printf("Server disconnected.\n");
  Aria::shutdown();
  return 0;
}
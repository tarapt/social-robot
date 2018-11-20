#include <iostream>
#include "restclient-cpp/connection.h"
#include "restclient-cpp/restclient.h"
using namespace std;

int main() {
    // initialize RestClient
    RestClient::init();

    // get a connection object
    RestClient::Connection* conn = new RestClient::Connection("http://127.0.0.1:5000");

    // configure basic auth
    // conn->SetBasicAuth("WarMachine68", "WARMACHINEROX");;

    // set headers
    RestClient::HeaderFields headers;
    headers["Accept"] = "application/json";
    conn->SetHeaders(headers);

    RestClient::Response r = conn->get("/hello");

    cout << r.body << endl;
 
    // deinit RestClient. After calling this you have to call RestClient::init()
    // again before you can use it
    RestClient::disable();
    return 0;
}
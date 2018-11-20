#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/fusion/adapted.hpp>

#include "restc-cpp/restc-cpp.h"
#include "restc-cpp/SerializeJson.h"
#include "restc-cpp/RequestBuilder.h"

using namespace std;
using namespace restc_cpp;


// For entries received from http://jsonplaceholder.typicode.com/posts
struct Post {
    int userId = 0;
    int id = 0;
    string title;
    string body;
};

BOOST_FUSION_ADAPT_STRUCT(
    Post,
    (int, userId)
    (int, id)
    (string, title)
    (string, body)
)

void DoSomethingInteresting(Context& ctx) {

     try {
        // Asynchronously fetch the entire data-set, and convert it from json
        // to C++ objects was we go.
        // We expcet a list of Post objects
        list<Post> posts_list;
        SerializeFromJson(posts_list,
            ctx.Get("http://jsonplaceholder.typicode.com/posts"));

        // Just dump the data.
        for(const auto& post : posts_list) {
            clog << "Post id=" << post.id << ", title: " << post.title << endl;
        }

    } catch (const exception& ex) {
        clog << "Caught exception: " << ex.what() << endl;
    }
}

int main() {
    // Add the proxy information to the properties used by the client
    Request::Properties properties;
    properties.proxy.type = Request::Proxy::Type::HTTP;
    properties.proxy.address = "http://172.31.1.4:8080";

    auto rest_client = RestClient::Create(properties);

    // Call DoSomethingInteresting as a co-routine in a worker-thread.
    rest_client->Process(DoSomethingInteresting);

    // Wait for the coroutine to finish, then close the client.
    rest_client->CloseWhenReady(true);
}
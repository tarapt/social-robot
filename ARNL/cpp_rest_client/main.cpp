// We use the example from the main readme file

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
struct Post {
    string name;
    double position[3];
    double theta[3];
};

// Since C++ does not (yet) offer reflection, we need to tell the library how
// to map json members to a type. We are doing this by declaring the
// structs/classes with BOOST_FUSION_ADAPT_STRUCT from the boost libraries.
// This allows us to convert the C++ classes to and from JSON.

BOOST_FUSION_ADAPT_STRUCT(
    Post,
    (string, name)
    (double, position[3])
    (double, theta[3])
)

int main() {
    // Set the log-level to a reasonable value
    logging::core::get()->set_filter(
        logging::trivial::severity >= logging::trivial::info
    );

    // Create an instance of the rest client
    auto rest_client = RestClient::Create();

    // Create and instantiate a Post from data received from the server.
    list<Post> post_list = rest_client->ProcessWithPromiseT<list<Post> >([&](Context& ctx) {
        // This is a co-routine, running in a worker-thread

        // Instantiate a Post structure.
        list<Post> post_list;

        // Serialize it asynchronously. The asynchronously part does not really matter
        // here, but it may if you receive huge data structures.
        SerializeFromJson(post_list,

            // Construct a request to the server
            RequestBuilder(ctx)
                .Get("http://127.0.0.1:5000/detections")

                // Add some headers for good taste
                .Header("X-Client", "RESTC_CPP")
                .Header("X-Client-Purpose", "Testing")

                // Send the request
                .Execute());

        // Return the post instance trough a C++ future<>
        return post_list;
    })

    // Get the Post instance from the future<>, or any C++ exception thrown
    // within the lambda.
    .get();

    // Print the result for everyone to see.
    for(Post post: post_list)
        cout << post.name << endl;
}

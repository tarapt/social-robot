g++ -fPIC -g -Wall -D_REENTRANT -fno-exceptions \
    -I/usr/local/Arnl/include -I/usr/local/Arnl/include/Aria -I/usr/local/Arnl/include/ArNetworking \
    -DARNL -DSONARNL \
    -o justPathPlanning justPathPlanning.cpp \
    -L/usr/local/Arnl/lib -lBaseArnl -lArNetworkingForArnl -lAriaForArnl \
    -Bstatic -lstdc++ -Xlinker -Bdynamic -lpthread -ldl -lrt

# ./justPathPlanning -remoteHost 172.16.21.14 -robotPort 8101
./justPathPlanning
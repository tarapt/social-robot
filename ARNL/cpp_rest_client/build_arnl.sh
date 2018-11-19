g++ -fPIC -g -Wall \
    -I/usr/local/Arnl/include -I/usr/local/Arnl/include/Aria -I/usr/local/Arnl/include/ArNetworking \
    -DARNL -DSONARNL \
    -o arnlRestClient arnlRestClient.cpp \
    -L/usr/local/Arnl/lib -lBaseArnl -lArNetworkingForArnl -lAriaForArnl -lSonArnl \
    -Bstatic -lstdc++ -Xlinker -Bdynamic -lpthread -ldl -lrt
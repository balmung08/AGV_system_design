#ifndef MYUDP_H
#define MYUDP_H

#include <unistd.h>
#include <cstdarg>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <string>
#include <memory.h>
#include <iostream>
#include <fstream>
#include "public.h"

#define ERR_EXIT(m)         \
    do                      \
    {                       \
        perror(m);          \
        exit(EXIT_FAILURE); \
    } while (0)

struct TRemote_port
{
    string ip;
    int port;
};

class TUDP : public Thread
{
private:
    int sock;
    vector<string> s_buf;
    void SendFormat(char *fmt, ...);

public:
    char recvbuf[1024],rec_output[1024];
    int rec_flag;
    int rec_count=0;
    vector<TRemote_port> remote_client;
    vector<string> recv_strs;
    pthread_mutex_t mute;

    TUDP(int lcport = 8080);
    TUDP(char *filename);
    TUDP(int lcport, int rmport, const char *rmip);
    // int Receive(void);
    void Init(int port);
    void Close(void);
    void Send(char *str);
    void Send(string str);
    void Send(char *ip, int port, char *str);
    void SendBuf();
    void AddSendStr(char *str);
    void AddSendStr(string str);
    void CheckRec();
    void AddRemoteIP(string ip, int port);
    void run();
};
// void UdpSendData(void);

#endif // MYUDP_H
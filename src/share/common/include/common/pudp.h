#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string.h>
#include <string>
#include <vector>

using namespace std;

struct TRemote_port
{
    string ip;
    int port;
};

class PUDP
{
private:
    int serv_sock;
    // int localport;

public:
    char rec_buf[1024];
    int localport;
    vector<TRemote_port> remote_client;
    void UDPInit(int port,int time_out) 
    {
        struct timeval read_timeout;
        read_timeout.tv_sec=0;
        read_timeout.tv_usec=time_out;

        if ((serv_sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0)   printf("socket error\n");
        setsockopt(serv_sock,SOL_SOCKET,SO_RCVTIMEO,&read_timeout,sizeof(time_out));


        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        servaddr.sin_addr.s_addr =htonl(INADDR_ANY);  //inet_addr("127.0.0.1");

        //sin_family指代协议族，在socket编程中只能是AF_INET
        //sin_port存储端口号（使用网络字节顺序）
        //s_addr按照网络字节顺序存储IP地址
        printf("listen to %d port\n",port);
        if (bind(serv_sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
            printf("bind error");
    }

    PUDP(int lport = 1234,int time_out=10)
    {
        localport = lport;
        UDPInit(localport,time_out);
        // TRemote_port port = {"192.168.3.5", 1234};
        // remote_client.push_back(port);
    }

    void AddRemoteIP(string ip, int port)
    {
        TRemote_port p = {ip, port};
        remote_client.push_back(p);
    }

    void Close(void)
    {
        close(serv_sock);
    }

    void Send(char *str)
    {
        for (int i = 0; i < remote_client.size(); i++)
        {
            char ip[100];
            strcpy(ip, remote_client.at(i).ip.c_str());
            Send(ip, remote_client.at(i).port, str); 
            // printf("%s:%d %s\n", remote_client.at(i).ip.c_str(),remote_client.at(i).port, str);
        }
    }

    void Send(char *ip, int port, char *str)
    {
        sockaddr_in send_adr;

        send_adr.sin_family = AF_INET;
        send_adr.sin_port = htons(port);
        send_adr.sin_addr.s_addr = inet_addr(ip);
        sendto(serv_sock, str, strlen(str), 0, (sockaddr *)&send_adr, sizeof(sockaddr_in)); 
    }

    int Recv()
    {
        struct sockaddr_in peeraddr;
        socklen_t peerlen;
        peerlen = sizeof(peeraddr);
        memset(rec_buf, 0, sizeof(rec_buf));

        int len = recvfrom(serv_sock, rec_buf, sizeof(rec_buf), 0, (struct sockaddr *)&peeraddr, &peerlen);
        if (len <= 0)  printf("recv error\n");
        return len;
    }
};

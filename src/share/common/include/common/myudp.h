#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <string>
#include <memory.h>
#include <common/public.h>

using namespace std;

#define ERR_EXIT(m) \
    do { \
    perror(m); \
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
    int  sock;
    pthread_mutex_t mutex;    // 互斥锁
    int localport;

public:
    char rec_buf[1024];
    int rec_flag, rec_count, rec_len;
    vector<TRemote_port> remote_client;

    TUDP(int lport=8090)
    {
        localport=lport;
        Init(localport);
        // TRemote_port port={"192.168.3.103",8080};
        // remote_client.push_back(port);

        rec_flag=0;
        rec_count=0;
        start();
    }
    ~TUDP()
    {
	close(sock);
    }

    int Receive( )
    {
        int flag=rec_flag;
        rec_flag=0;
        return(flag);
    }
/*
//sin_family指代协议族，在socket编程中只能是AF_INET
//sin_port存储端口号（使用网络字节顺序）
//s_addr按照网络字节顺序存储IP地址
*/
    void Init(int port)
    {
        if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0)   ERR_EXIT("socket error");

        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        servaddr.sin_addr.s_addr =htonl(INADDR_ANY);  //inet_addr("127.0.0.1");
        //解决终端ctrl+c之后，端口无法释放问题
        int mw_optval = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&mw_optval,sizeof(mw_optval));

        printf("listen to %d port\n",port);
        if (bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
            ERR_EXIT("bind error");
    }

    void AddRemoteIP(string ip, int port)
    {
        TRemote_port p={ip,port};
        remote_client.push_back(p);
    }

    void Close(void)
    {
        close(sock);
    }

    void Send(char *str)
    {
        for(int i=0;i<remote_client.size();i++)
        {
            char ip[100];
            strcpy(ip,remote_client.at(i).ip.c_str());
            Send(ip, remote_client.at(i).port, str);
            //printf("%s:%d %s\n", remote_client.at(i).ip.c_str(),remote_client.at(i).port, str);
        }
    }

    void Send(char *ip, int port, char *str)
    {
        struct sockaddr_in sendaddr;

        //pthread_mutex_lock(&mutex);
        sendaddr.sin_family = AF_INET;
        sendaddr.sin_port = htons(port);
        sendaddr.sin_addr.s_addr = inet_addr(ip);
        sendto(sock, str, strlen(str), 0, (struct sockaddr *)&sendaddr, sizeof(struct sockaddr_in));
        //pthread_mutex_unlock(&mutex);
    }

    void Send(char *ip, int port, char *str, int count)
    {
        struct sockaddr_in sendaddr;

        //pthread_mutex_lock(&mutex);
        sendaddr.sin_family = AF_INET;
        sendaddr.sin_port = htons(port);
        sendaddr.sin_addr.s_addr = inet_addr(ip);
        sendto(sock, str, count, 0, (struct sockaddr *)&sendaddr, sizeof(struct sockaddr_in));
        //pthread_mutex_unlock(&mutex);
    }

    void Send(char *str, int count)
    {
        for(int i=0;i<remote_client.size();i++)
        {
            char ip[100];
            strcpy(ip,remote_client.at(i).ip.c_str());
            Send(ip, remote_client.at(i).port, str, count);
            // printf("%s:%d %s\n", remote_client.at(i).ip.c_str(),remote_client.at(i).port, str);
        }
    }

    void run()
    {
        int len;
        char recvbuf[1024] = {0};
        struct sockaddr_in peeraddr;
        socklen_t peerlen;

        printf("udp running\n");

        while (1)
        {
            peerlen = sizeof(peeraddr);
            memset(recvbuf, 0, sizeof(recvbuf));

            //memset某一块内存中的内容全部设置为指定的值
            //函数解释：将recvbuf中当前位置后面的sizeof(recvbuf)个字节，用0替换并返回recvbuf
            len = recvfrom(sock, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&peeraddr, &peerlen);
            //recvfrom函数(经socket接收数据)。
            string ip=inet_ntoa(peeraddr.sin_addr);
            //将一个十进制网络字节序转换为点分十进制IP格式的字符串。

            if(len>0)
            {
                rec_flag=1;
                rec_count++;
                rec_len=len;
                // strcpy(rec_buf,recvbuf);
                memcpy(rec_buf,recvbuf,len);
                
                // printf("recvbuf = %s\n",recvbuf);
            }

            if (len <= 0)
            {
                printf("recvfrom error\n");
                if (errno == EINTR)  continue;
                // ERR_EXIT("recvfrom error");
            }
        }
        close(sock);
    }
};



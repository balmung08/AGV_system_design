#include "myudp.h"
using namespace std;
// UDP信息接收发送
TUDP::TUDP(int lcport)
{
    recv_strs.clear();
    remote_client.clear();
    Init(lcport);
    printf("UDP init: local port: %d.\n", lcport);

    rec_flag = 0;
    rec_count = 0;
    start();

    pthread_mutex_init(&mute, NULL);
}

/*使用初始化文件进行构造,没写异常处理，文件格式不对必报错
初始化文件格式：
第一行：本地端口号
第二行：远端端口号1
第三行：远端IP 1
第四行：远端端口号2
第五行：远端IP 2
…………
例如：
1 | 8080
2 | 8090
3 | 192.168.31.192
4 | 8091
5 | 192.168.31.150
…………
没写异常处理，文件格式不对必报错*/
TUDP::TUDP(char *filename)
{
    ifstream fin;
    fin.open(filename, ios::in);
    char data[128];
    vector<string> list;
    while (fin.getline(data, sizeof(data)))
    {
        list.push_back(data);
    }
    fin.close();
    // print_vecstr(list);
    int lcport = stoi(list.at(0).c_str());
    Init(lcport);
    printf("UDP init: local port: %d.\n", lcport);
    remote_client.clear();
    if (list.size() > 1)
    {
        TRemote_port p;
        int rm_num = (int)((list.size() - 1) / 2);
        for (int i = 1; i <= rm_num; i++)
        {
            p.port = stoi(list.at(2 * i - 1).c_str());
            p.ip = list.at(2 * i);
            remote_client.push_back(p);
        }
    }
    for (int i = 0; i < remote_client.size(); i++)
        printf("remote%d %s:%d\n", i + 1, remote_client[i].ip.c_str(), remote_client[i].port);
    rec_flag = 0;
    rec_count = 0;
    start();
    pthread_mutex_init(&mute, NULL);
}

TUDP::TUDP(int lcport, int rmport, const char *rmip)
{
    recv_strs.clear();
    Init(lcport);
    printf("UDP init: local port: %d.\n", lcport);
    remote_client.clear(); // vector
    TRemote_port p;        // 远程节点结构体
    p.port = rmport;
    p.ip = rmip;
    remote_client.push_back(p);
    for (int i = 0; i < remote_client.size(); i++)
        printf("remote%d %s:%d\n", i + 1, remote_client[i].ip.c_str(), remote_client[i].port);
    rec_flag = 0;
    rec_count = 0;
    start();

    pthread_mutex_init(&mute, NULL);
}

void TUDP::Init(int port)
{
    if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket error");

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); // inet_addr("127.0.0.1");
    // sin_family指代协议族，在socket编程中只能是AF_INET
    // sin_port存储端口号（使用网络字节顺序）
    // s_addr按照网络字节顺序存储IP地址
    printf("listen to %d port\n", port);
    if (bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        ERR_EXIT("bind error");
}

void TUDP::Close(void)
{
    close(sock);
}

void TUDP::Send(char *str)
{
    for (int i = 0; i < remote_client.size(); i++)
    {
        char ip[100];
        strcpy(ip, remote_client[i].ip.c_str());
        Send(ip, remote_client[i].port, str);
    }
}

void TUDP::Send(string str)
{
    if (str.size() == 0)
        return;
    char s[1000];
    strcpy(s, str.c_str());
    for (int i = 0; i < remote_client.size(); i++)
    {
        char ip[100];
        strcpy(ip, remote_client[i].ip.c_str());
        Send(ip, remote_client[i].port, s);
    }
}

void TUDP::SendFormat(char *fmt, ...)
{
    char buf[1000] = {0};

    va_list ap;
    int d;
    double f;
    char c;
    char *s;
    char flag;
    va_start(ap, fmt);
    while (*fmt)
    {
        flag = *fmt++;
        if (flag != '%')
        {
            sprintf(buf, "%s%c", buf, flag);
            // putchar(flag);
            continue;
        }
        flag = *fmt++; // 记得后移一位
        switch (flag)
        {
        case 's':
            s = va_arg(ap, char *);
            sprintf(buf, "%s%s", buf, s);
            // printf("%s",s);
            break;
        case 'd': /* int */
            d = va_arg(ap, int);
            sprintf(buf, "%s%d", buf, d);
            // printf("%d", d);
            break;
        case 'f': /* double*/
            f = va_arg(ap, double);
            sprintf(buf, "%s%.5f", buf, f);
            // printf("%.5f", f);
            break;
        case 'c': /* char*/
            c = (char)va_arg(ap, int);
            sprintf(buf, "%s%c", buf, c);
            // printf("%c", c);
            break;
        default:
            // putchar(flag);
            sprintf(buf, "%s%c", buf, flag);
            break;
        }
    }
    va_end(ap);

    Send(buf);
    // printf("%s\n",buf);
}

void TUDP::Send(char *ip, int port, char *str)
{
    struct sockaddr_in sendaddr;
    sendaddr.sin_family = AF_INET;
    sendaddr.sin_port = htons(port);
    sendaddr.sin_addr.s_addr = inet_addr(ip);
    sendto(sock, str, strlen(str), 0, (struct sockaddr *)&sendaddr, sizeof(struct sockaddr_in));
}

void TUDP::SendBuf()
{
    string buf = "";
    pthread_mutex_lock(&mute);
    for (int i = 0; i < s_buf.size(); i++)
    {
        if (buf.size() > 500)
        {
            Send(buf);
            buf.clear();
            usleep(10000);
        }
        buf += s_buf.at(i);
    }
    Send(buf);
    s_buf.clear();
    pthread_mutex_unlock(&mute);
}

void TUDP::AddSendStr(char *str)
{
    pthread_mutex_lock(&mute);
    if (s_buf.size() < 1000)
        s_buf.push_back(str);
    pthread_mutex_unlock(&mute);
}

void TUDP::AddSendStr(string str)
{
    pthread_mutex_lock(&mute);
    if (s_buf.size() < 1000)
        s_buf.push_back(str);
    pthread_mutex_unlock(&mute);
}

void TUDP::CheckRec() // 将recvbuf中的内容转入recv_strs
{
    recv_strs = split(recvbuf, " ");
    rec_count = recv_strs.size();
    // cout << "Received "<<rec_count<<" data: " << recvbuf << endl;
}

void TUDP::AddRemoteIP(string ip, int port)
{
    TRemote_port p = {ip, port};
    remote_client.push_back(p);
}

void TUDP::run()
{
    int len;
    //    char recvbuf[1024] = {0};

    struct sockaddr_in peeraddr;
    socklen_t peerlen;
    while (1)
    {
        peerlen = sizeof(peeraddr);
        memset(recvbuf, 0, sizeof(recvbuf));
        // memset某一块内存中的内容全部设置为指定的值
        // 函数解释：将recvbuf中当前位置后面的sizeof(recvbuf)个字节，用0替换并返回recvbuf
        len = recvfrom(sock, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&peeraddr, &peerlen);
        // recvfrom函数(经socket接收数据)。
        // string ip = inet_ntoa(peeraddr.sin_addr);
        // 将一个十进制网络字节序转换为点分十进制IP格式的字符串。
        if (len > 0 && rec_flag == 0) // 接收到数据
        {
            strcpy(rec_output, recvbuf);
            rec_flag = 1;
            // CheckRec();
            // rec_flag = 0;
        }

        else if (len <= 0)
        {
            if (errno == EINTR)
                continue;
            ERR_EXIT("recvfrom error");
        }
    }
    close(sock);
}

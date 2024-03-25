### 部署方式
* 连接WIFI-EFA1;地址是192.168.3.xxx,xxxx可以任意;子网掩码255.255.255.0;网关192.168.1.1随便连一个其他wifi回来再连EFA1
* 复制编译文件到车里: sudo scp -r xxxxx root@192.168.3.103:/home/
* ssh登录车: ssh root@192.168.3.103
* 进入/home,执行./xxxxx即可

### 效果演示
<div align=center><img src="pic/c.gif"></div>

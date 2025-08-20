## 192.168.10.10的那个
sudo ip addr add 192.168.10.100/24 dev enp3s0
sudo ip link set enp3s0 up

如果机械臂控制器运行Linux，那么：
机械臂的enp3s0网卡配置了192.168.10.10
你的Linux主机的enp3s0网卡配置了192.168.10.100
两个设备通过网线直连通信

linux主机（nuc）有两个网线接口，应该就分别对应esp3s0和esp4s0.

然后现在在linux上面就能ping 通192.168.10.10了。


## 测试linux主机到底用的哪个网卡，enp3s0一般在上面，enp4s0在下面。
```
# 插上查看
ubuntu@ubuntu-NUC13RNGi5:~$ ip link show | grep -E "(enp3s0|enp4s0)"
2: enp3s0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc mq state DOWN mode DEFAULT group default qlen 1000
3: enp4s0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP mode DEFAULT group default qlen 1000

# 拔后查看
ubuntu@ubuntu-NUC13RNGi5:~$ ip link show | grep -E "(enp3s0|enp4s0)"
2: enp3s0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc mq state DOWN mode DEFAULT group default qlen 1000
3: enp4s0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc mq state DOWN mode DEFAULT group default qlen 1000

# 哪个网口从 UP,LOWER_UP 变成了 UP（没有LOWER_UP），那就是连接的网口
```
`Linux主机 enp4s0 ←→ 网线 ←→ 机械臂 enp4s0`

## 192.168.20.10的那个
## 以上是确保物理层面连接的问题，这下面是给网口配置ip，保证和机械臂的网口（也是enp4s0网卡，192.168.20.10）在同一个子网下，
ubuntu@ubuntu-NUC13RNGi5:~$ sudo ip addr add 192.168.20.100/24 dev enp4s0 # 注意这个enp4s0是linux主机的，两边的网卡都是enp4s0
ubuntu@ubuntu-NUC13RNGi5:~$ 
ubuntu@ubuntu-NUC13RNGi5:~$ sudo ip link set enp4s0 up

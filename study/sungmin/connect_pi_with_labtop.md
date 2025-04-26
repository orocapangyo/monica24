# 랜선으로 라즈베리파이와 노트북 연결하기

다음은 유선랜을 이용하여 라즈베리파이와 노트북을 연결하여 로컬 네트워크를 구축하고, SSH를 이용하여 노트북에서 라즈베리파이로 접속하는 방법이다.
이 방법을 통해 라즈베리파이에 모니터를 연결하지 않고 ssh로 연결하여 작업을 할 수 있다.

1. 라즈베리파이의 터미널을 열고 ssh 서버를 설치한다.
```bash
sudo apt update
sudo apt install openssh-server
```

2. 라즈베리파이 ssh를 enable해준다.
```bash
sudo systemctl enable ssh
```

3. 다시 노트북으로 돌아와서, settings > network 의 wired 의 설정 버튼을 눌러주고 다음과 같이 설정한다.
![image](https://github.com/user-attachments/assets/7b50c4a8-fea7-469f-8786-012c8b9c678c)

4. 노트북의 ip를 확인하기 위해 net-tools를 설치한다
```bash
sudo apt install net-tools
```

5. 설치 후 ifconfig 명령어를 이용하여 노트북 로컬 네트워크의 주소를 확인한다. (보통 첫번째 나오는 10.42.0.1이 로컬네트워크의 게이트 주소이며, 라즈베리파이도 10.42.0.* 중 임의의 ip가 할당된다.)
```bash
sm@sm-750XGK:~$ ifconfig
enp3s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.42.0.1  netmask 255.255.255.0  broadcast 10.42.0.255
        inet6 fe80::2280:d8b6:a619:8b3e  prefixlen 64  scopeid 0x20<link>
        ether c8:41:8a:13:e6:13  txqueuelen 1000  (Ethernet)
        RX packets 28  bytes 8232 (8.2 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 139  bytes 23695 (23.6 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

6. 랜선으로 라즈베리파이와 노트북을 연결한다.
7. 노트북에서 라즈베리파이의 ip를 확인하기 위해 노트북에 nmap을 설치하여준다.
```bash
sudo apt install nmap
```
8. 노트북에 nmap이 설치되면 다음 명령어로 라즈베리파이 주소를 확인한다.
```bash
nmap 10.42.0.*
```

9. 정상적으로 연결이 되었으면, 아래와 같이 결과가 나오는데 10.42.0.38이 라즈베리파이의 주소이다.
```bash
sm@sm-750XGK:~$ nmap 10.42.0.*
Starting Nmap 7.80 ( https://nmap.org ) at 2025-04-27 01:13 KST
Nmap scan report for sm-750XGK (10.42.0.1)
Host is up (0.00017s latency).
Not shown: 999 closed ports
PORT   STATE SERVICE
53/tcp open  domain

Nmap scan report for 10.42.0.38
Host is up (0.00051s latency).
Not shown: 999 closed ports
PORT   STATE SERVICE
22/tcp open  ssh
```

10. 노트북에서 ssh 명령어로 라즈베리파이에 접속한다. (@ 앞의 sm은 라즈베리파이에서 설정한 라즈베리파이 이름이다.)
```bash
ssh sm@10.42.0.38
```

11. 정상적으로 접속할 경우, 아래와 같이 나타난다. (처음 접속하면 뭐라고 묻는데, yes 라고 입력하고 그 다음에 라즈베리파이 비밀번호 물어보면 입력해준다.)
```bash
sm@sm-750XGK:~$ ssh sm@10.42.0.38
The authenticity of host '10.42.0.38 (10.42.0.38)' can't be established.
ED25519 key fingerprint is SHA256:SQYGHbHsU4wlnccTc1ZaZjFIvsRuR.
This key is not known by any other names
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes

Warning: Permanently added '10.42.0.38' (ED25519) to the list of known hosts.
sm@10.42.0.38's password:

Welcome to Ubuntu 22.04.5 LTS (GNU/Linux 5.15.0-1074-raspi aarch64)
```

참고링크
- https://github.com/orocapangyo/monica24/blob/main/study/dohun/2024_%EA%B2%A8%EC%9A%B8/2%EC%A3%BC%EC%B0%A8_%EC%98%88%EC%8A%B5_241220.pdf

   

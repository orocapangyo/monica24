## USB 고정하는 방법

USB의 번호는 라즈베리파이에 전원이 들어올때 결정이 된다. 대부분의 경우에 0번부터 시작이 되지만, 여러개의 USB가 있는 경우에는 장치마다 번호가 달라지는 경우가 있다. USB의 번호가 달라지면, 라즈베리파이에서 장치를 제어하는데 문제가 생길 수 있으므로, 라즈베리파이에서 고정하는 방법을 알아보자.

1. 터미널에서 다음 명령어를 입력한다.

```bash
opg@vm:/opt$ sudo dmesg | grep ttyUSB
[sudo] password for opg: 
[  541.181967] usb 2-2.2: cp210x converter now attached to ttyUSB0
```

2. 터미널에서 다음과 같이 입력해서 세부 정보를 알 수 있다.

```bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk 


 looking at parent device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.2':
    KERNELS=="1-1.2"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{authorized}=="1"
    ATTRS{avoid_reset_quirk}=="0"
    ATTRS{bConfigurationValue}=="1"
    ATTRS{bDeviceClass}=="00"
    ATTRS{bDeviceProtocol}=="00"
    ATTRS{bDeviceSubClass}=="00"
    ATTRS{bMaxPacketSize0}=="64"
    ATTRS{bMaxPower}=="100mA"
    ATTRS{bNumConfigurations}=="1"
    ATTRS{bNumInterfaces}==" 1"
    ATTRS{bcdDevice}=="0100"
    ATTRS{bmAttributes}=="80"
    ATTRS{busnum}=="1"
    ATTRS{configuration}==""
    ATTRS{devnum}=="3"
    ATTRS{devpath}=="1.2"
    ATTRS{devspec}=="(null)"
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"
    ATTRS{ltm_capable}=="no"
    ATTRS{manufacturer}=="Silicon Labs"
    ATTRS{maxchild}=="0"
    ATTRS{power/active_duration}=="676468"
    ATTRS{power/autosuspend}=="2"
    ATTRS{power/autosuspend_delay_ms}=="2000"
    ATTRS{power/connected_duration}=="676472"
    ATTRS{power/control}=="on"
    ATTRS{power/level}=="on"
    ATTRS{power/persist}=="1"
    ATTRS{power/runtime_active_kids}=="0"
    ATTRS{power/runtime_active_time}=="676287"
    ATTRS{power/runtime_enabled}=="forbidden"
    ATTRS{power/runtime_status}=="active"
    ATTRS{power/runtime_suspended_time}=="0"
    ATTRS{power/runtime_usage}=="1"
    ATTRS{product}=="CP2102 USB to UART Bridge Controller"
    ATTRS{quirks}=="0x0"
    ATTRS{removable}=="unknown"
    ATTRS{rx_lanes}=="1"
    ATTRS{serial}=="0001"
    ATTRS{speed}=="12"
    ATTRS{tx_lanes}=="1"
    ATTRS{urbnum}=="12"
    ATTRS{version}==" 1.10"

```

3. 위의 결과에서 ATTRS{port_number}의 값을 확인한다. 초기화 파일을 작성한다.
특히 devpath값은 사용하는 OS에 따라서 다르므로 주의가 필요하다.

    ATTRS{devpath}=="1.2"

    ATTRS{idProduct}=="ea60"

    ATTRS{idVendor}=="10c4"

```bash
sudo nano /etc/udev/rules.d/99-usb.rules

#for raspberry pi OS
#SUBSYSTEM=="tty", ATTRS{idProduct}=="ea60", SYMLINK+="esp32Nodemcu"
#for UBUNTU
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.2", ATTRS{idProduct}=="ea60", ATTRS{idVendor}=="10c4", MODE:="0777", SYMLINK+="esp32Nodemcu"
KERNEL=="ttyUSB*", ATTRS{devpath}=="%%", ATTRS{idProduct}=="ea60", ATTRS{idVendor}=="10c4", MODE:="0777", SYMLINK+="esp32Nodemcu"

```

4. 다음 명령어를 입력해서 초기화 파일을 적용한다.

```bash
opg@opg-pi:~$ sudo udevadm control --reload-rules
opg@opg-pi:~$ sudo udevadm trigger
opg@opg-pi:~$ ls -al /dev/esp*
lrwxrwxrwx 1 root root 7 Jan 25 13:20 /dev/esp32Nodemcu -> ttyUSB0
opg@opg-pi:~$ ls -al /dev/ttyUSB*
crwxrwxrwx 1 root dialout 188, 0 Jan 25 13:20 /dev/ttyUSB0
```


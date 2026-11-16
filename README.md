
## 1. To assign ip to bbb:

sudo ip addr add 192.168.7.1/24 dev eth1   
sudo ip link set eth1 up

## 2. To configure iptables:

sudo sysctl -w net.ipv4.ip_forward=1        
sudo iptables -t nat -A POSTROUTING -o amn0 -j MASQUERADE
sudo iptables -A FORWARD -i eth1 -o amn0 -j ACCEPT
sudo iptables -A FORWARD -i amn0 -o eth1 -m state --state RELATED,ESTABLISHED -j ACCEPT

### 2.1 On BBB:

sudo ip route add default via 192.168.7.1 dev usb0
sudo sh -c "echo 'nameserver 8.8.8.8' > /etc/resolv.conf"


## 3. To start BBB go server:

sudo ./beaglelogic-tcp &
sudo netstat -tulpan | grep 5555

# Alloc memory
sudo sh -c "echo 262144000 > /sys/devices/virtual/misc/beaglelogic/memalloc"

# Frequency
sudo sh -c 'echo 6000000 > /sys/devices/virtual/misc/beaglelogic/samplerate'

# sampleunit
sudo sh -c 'echo 1 > /sys/devices/virtual/misc/beaglelogic/sampleunit'

# Continuous (stream mode) 
sudo sh -c 'echo 1 > /sys/devices/virtual/misc/beaglelogic/triggerflags'

## снять захват
sigrok-cli -d beaglelogic -C P8_45,P8_46,P8_43,P8_44,P8_41,P8_42,P8_39,P8_40 --time 44s -c samplerate=6M -o /home/debian/n91_minute.sr


# emulator_mk4001mtc

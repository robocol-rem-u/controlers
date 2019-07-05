#!/usr/bin/env python3
import subprocess, socket, roslaunch

if __name__ == '__main__':
    arg1 = "ROS_MASTER_URI=http://192.168.0.105:11311"
    arg2 = "ROS_IP="+str(socket.gethostbyname(socket.gethostname()))
    subprocess.call('export'+arg1, shell=True)
    subprocess.call('export'+arg2, shell=True)




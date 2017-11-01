# KukaRosOpenCommunication
Allows controlling KUKA Robot without additional technology package.

## About KukaRosOpenCommunication
KukaOpenCommunication is an open-source interface that allows controlling Kuka robots by using Robot Operating System without requirement of an additional technology package. This interface was developed based on OpenShowVar[1] and JOpenShowVar[2] projects. It is compatible for KRC4 controllers. 

Maintainer: Aytaç Kahveci                                                                                                       
Mail: aytackahveci93@gmail.com

## References
[1]OpenShowVar project: https://github.com/cyberpro4/openshowvar                                                                   
[2]JOpenShowVar project: https://github.com/aauc-mechlab/JOpenShowVar

## Usage of the KukaRosOpenCommunication Interface
First you should open the port 7000 for communication in the KRC controller. 
    SmartHMI -> Network configuration -> NAT -> Add port -> Port number 7000 and Permitted protocols: tcp/udp
Make sure that the Windows interface of the controller and the PC with ROS is connected to the same subnet.
KUKAVARPROXY should be executed in the KRC controller. 
    Coppy paste the KUKAVARPROXY folder to somewhere in the Windows environment and run KUKAVARPROXY.exe


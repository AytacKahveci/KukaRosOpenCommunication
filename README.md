# KukaRosOpenCommunication
Allows controlling KUKA Robot without additional technology package.

## About KukaRosOpenCommunication
KukaOpenCommunication is an open-source interface that allows controlling Kuka robots by using Robot Operating System without requirement of an additional technology package. This interface was developed based on OpenShowVar[1] and JOpenShowVar[2] projects. It is compatible for KRC4 controllers. 

Maintainer: Aytaç Kahveci                                                                                                       
Mail: aytackahveci93@gmail.com

Demo Video :
https://www.youtube.com/watch?v=UnocwVtGvZc

## References
[1]OpenShowVar project: https://github.com/cyberpro4/openshowvar                                                                   
[2]JOpenShowVar project: https://github.com/aauc-mechlab/JOpenShowVar

## Usage of the KukaRosOpenCommunication Interface
- First you should open the port 7000 for communication in the KRC controller.
		SmartHMI -> Network configuration -> NAT -> Add port -> Port number 7000 and Permitted protocols: tcp/udp
- Make sure that the Windows interface of the controller and the PC with ROS is connected to the same subnet.	
- Coppy paste the KUKAVARPROXY folder to somewhere in the Windows environment and run KUKAVARPROXY.exe
- Change the "Robot_IP" parameter for your robot inside the KukaRosOpenCommunication/blob/master/kuka_moveit_configuration/config/controller.yaml file.
- Create a MYAXIS variable inside the KRC:\System\$config.dat. Make sure that initial joint positions of the MYAXIS variable is appropriate for your case.

		DECL AXIS MYAXIS={A1 0,A2 -90,A3 90,A4 0,A5 0,A6 0}
        
- Write a module     

		INI
		PTP HOME Vel = 100 % DEFAULT

		LOOP
			PTP MYAXIS C_PTP
		ENDLOOP
		PTP HOME Vel = 100 % DEFAULT

- Select and run this module. It is offered to run the robot in T1 mode for the first run!
- Launch the /KukaRosOpenCommunication/blob/master/kuka_moveit_configuration/launch/demo.launch
- You can plan the motion of the KUKA robot with MoveIt! package.


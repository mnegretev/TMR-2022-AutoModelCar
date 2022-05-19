
	>>>>>>>>>TMR-2022 LUNETTES_TEAM<<<<<<<<<<<<<<<<<
	INSTRUCCIONES PARA EJECUTAR LOS PROGRAMAS 

	1. SE INSTALAN LIBRERIAS NESCESARIAS PARA EJECUTAR LOS PROGRAMAS
		
		sudo python2.7 -m pip install tensorflow-gpu==1.14.0
		pip install protobuf==3.17.3
		sudo apt-get install python-scipy


	2. SE EJECUTA EL SIMULADOR SEGUN LA PRUEBA INDICADA 

	 cd TMR-2022-AutoModelCar
	 cd catkin_ws
	 catkin_make -j2 -l2
	 echo "source ~/TMR-2022-AutoModelCar/catkin_ws/devel/setup.bash" >> ~/.bashrc
	 source ~/.bashrc

	Navegación autónoma sin obstáculos: roslaunch bring_up navigation_no_obstacles.launch
	Navegación con obstáculos estáticos: roslaunch bring_up  navigation_static_obstacles.launch
	Navegación con obstáculos dinámicos: roslaunch bring_up navigation_dynamic_obstacles.launch
	Prueba de estacionamiento: roslaunch bring_up parking.launch

	3. Ejecutar en otra terminal python cnn_final.py en la ubicación


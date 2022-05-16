# TMR 2022 - Categoría AutoModelCar

Este repositorio es el que se utilizará para la categoría AutoModelCar del Torneo Mexicano de Robótica 2022 en la modalidad virtual. 

## Requerimientos:

* Ubuntu 18.04
* ROS Melodic

## Instalación:

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* Seguir las instrucciones para instalar Webots: https://cyberbotics.com/doc/guide/installing-webots
* Seguir el tutorial para el uso de Webots con ROS: https://cyberbotics.com/doc/guide/tutorial-9-using-ros
* $ cd
* $ git clone https://github.com/mnegretev/TMR-2022-AutoModelCar
* $ cd TMR-2022-AutoModelCar
* $ cd catkin_ws
* $ catkin_make -j2 -l2
* $ echo "source ~/TMR-2022-AutoModelCar/catkin_ws/devel/setup.bash" >> ~/.bashrc
* $ source ~/.bashrc

## Pruebas

Una vez instalado y compilado el repositorio, se pueden ejecutar los ambientes de simulación con los respectivos archivos 'launch':

* Navegación autónoma sin obstáculos: roslaunch bring_up navigation_no_obstacles.launch
* Navegación con obstáculos estáticos: roslaunch bring_up navigation_static_obstacles.launch
* Navegación con obstáculos dinámicos: roslaunch bring_up navigation_dynamic_obstacles.launch
* Prueba de estacionamiento: roslaunch bring_up parking.launch

En cada prueba, el robot inicia en una posición y orientación aleatoria (dentro de un intervalo razonable).

## Iniciar proyecto Autos-LAR

Para que se ejecute sin problemas

* Hacer git pull a la rama Autos-LAR
* Hacer $ catkin_make
* Hacer $ source devel/setup.bash
* Activar el ambiente simulado
* Ejecutar $rosrun paquete_lar recorre_pista.py

Nota: Utiliza python3 y openCV

## Contacto

Cualquier duda o comentario sobre este desarrollo, escribir al responsable académico:

Miriam Hernández<br>
lar@acatlan.unam.mx


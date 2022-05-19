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

## Tópicos relevantes

### Tópicos publicados:

* ``/camera/rgb/raw`` (sensor_msgs/Image): Imagen RGB de la cámara
* ``/point_cloud`` (sensor_msgs/PointCloud2): Nube de puntos generada por el Lidar
* ``/gps`` (sensor_msgs/NavSatFix): Lectura del GPS
* ``/gyro`` (sensor_msgs/Imu): Lectura del giroscopio

### Tópicos suscritos:

* ``/speed`` (std\_msgs/Float64): Velocidad lineal deseada en [m/s]
* ``/steering`` (std\_msgs/Float64): Ángulo de las llantas delanteras en [rad]

## Máquina Virtual

Se puede descargar una máquina virtual para [VirtualBox](https://www.virtualbox.org/wiki/Downloads) con todo lo necesario ya instalado de [esta dirección.](https://drive.google.com/drive/folders/1t87Lxv1sRdRwFoKFRD4IFDJBtvQ-UxEx?usp=sharing) <br>
En esa misma carpeta hay un video con instrucciones para usar la máquina virtual. <br>
Usuario: Neo <br>
Contraseña: fidelio


## Contacto

Cualquier duda o comentario sobre esta prueba, escribir al responsable técnico:

Marco Negrete<br>
marco.negrete@ingenieria.unam.edu

## Instrucciones CIDETEC para realizar la primer prueba de navegación sin obstaculos
*Clonar este repositorio
* Activar la rama de CIDETEC con: \
  $ git checkout CIDETEC
* Ir desde terminal a catkin_ws y ejecutar \
  $ catkin_make \
  $ source devel/setup.bash
* Cambiar los permisos para permitir ejecución: \
  $ cd src/bring_up/src \
  $ chmod +x *.py 
* Ejecutar la simulación con: \
  $ roslaunch bring_up launcher_task1.launch

# Sistema de Teleoperación Bilateral Háptico con xArm Lite6
**TE3001B:** Fundamentación de robótica
**Institución:** Tecnológico de Monterrey
**Tecnologías:** ROS 2 Humble, Python, MoveIt Servo, micro-ROS (ESP32)

---

# 1. Descripción del Proyecto

Este proyecto implementa un sistema de teleoperación maestro–esclavo con retroalimentación háptica utilizando dos manipuladores **xArm Lite6**. El objetivo principal es permitir que un operador controle remotamente un robot esclavo mientras recibe retroalimentación de fuerza cuando el robot remoto entra en contacto con un objeto.

El sistema incluye:

* Un robot maestro manipulado por el operador.
* Un robot esclavo que replica los movimientos del maestro.
* Un sensor de fuerza conectado a un ESP32, encargado de medir la interacción con el entorno.

---

# 2. Arquitectura del Sistema

## Hardware

El sistema está compuesto por los siguientes elementos:

* **Robot Maestro:** xArm Lite6 (modo de control de velocidad/estado).
* **Robot Esclavo:** xArm Lite6 (modo seguidor).
* **Sensor de Fuerza:** Resorte con sensor de distanncia conectsdo a un ESP32.
* **Laptop de control:** Ejecuta ROS2 y los nodos de control.
* **Red de comunicación:** Conexión Ethernet para robots y WiFi para ESP32.

### Topología de red

```id="arch_net"
                Switch Ethernet
         ┌───────────┬───────────┬
         │           │           │
      Laptop     Robot Maestro  Robot Esclavo
         │
         │ WiFi
         │
        ESP32
   (Sensor de fuerza)
```

La laptop funciona como centro de control, ejecutando los nodos ROS2 y el micro-ROS Agent que conecta el ESP32 con el sistema ROS.

---

# 3. Arquitectura de Software

El sistema utiliza **ROS2 Humble** para la comunicación entre dispositivos.

## Topics principales

| Topic                            | Tipo                   | Descripción                      |
| -------------------------------- | ---------------------- | -------------------------------- |
| `/force_esp32`                   | std_msgs/Float32       | Fuerza medida por el sensor      |
| `/master/joint_states`           | sensor_msgs/JointState | Estado del robot maestro         |
| `/slave/joint_states`            | sensor_msgs/JointState | Estado del robot esclavo         |
| `/servo_server/delta_joint_cmds` | control_msgs/JointJog  | Comandos de velocidad al esclavo |
| `/master/robot_state`  | xarm_msgs/RobotMsg | Reporta e estado real del robot desde el hardware |


---

# 4. Control de Seguimiento Maestro–Esclavo

El robot esclavo sigue al maestro mediante control de velocidad utilizando:

```id="ctrl_eq"
q̇ = Kv q̇_master + Kp (q_master − q_slave)
```

Donde:

* `q_master` = posición del maestro
* `q_slave` = posición del esclavo
* `q̇_master` = velocidad del maestro
* `Kv` = ganancia de velocidad
* `Kp` = ganancia proporcional

Este controlador permite:

* seguimiento continuo
* corrección de error de posición
* movimientos suaves y estables

El sistema corre a una frecuencia de aproximadamente **200–400 Hz**, permitiendo teleoperación en tiempo real.

---

# 5. Conversión Fuerza → Torque

Cuando el esclavo entra en contacto con un objeto, el sensor mide la fuerza y se calcula el torque equivalente en las articulaciones usando el **Jacobiano transpuesto**:

```id="jac_eq"
τ = JᵀF
```

Donde:

* `J` es el Jacobiano del robot
* `F` es el vector de fuerza cartesiana
* `τ` es el torque resultante

Esto permite transformar la fuerza medida en el efector final en torques equivalentes en cada articulación.

---

# 6. Funcionalidades Implementadas

## A. Control de Bloqueo por Colisión

Cuando la fuerza medida supera un umbral definido:

```id="thr_eq"
F > 10.5 N
```

El sistema envía el comando:

```id="brake_cmd"
set_state(3)
```

Este comando activa los **frenos internos del robot maestro**, bloqueando físicamente el movimiento.

---

## B. Desbloqueo Automático por Retroceso

Para evitar que el sistema quede bloqueado permanentemente, se monitorea el cambio de posición:

```id="delta_q"
Δq = q_actual − q_anterior
```

Si el operador intenta retroceder:

```id="release_cond"
Δq < −0.002
```

El sistema envía:

```id="release_cmd"
set_state(0)
```

Esto **libera los frenos del robot maestro**.

---

# 7. Sensor de Fuerza con ESP32

El ESP32 ejecuta un nodo **micro-ROS** que publica la fuerza medida.

### Topic publicado

```id="force_topic"
/force_esp32
```

Tipo de mensaje:

```id="force_msg"
std_msgs/Float32
```

Ejemplo:

```id="force_ex"
data: 12.3
```

El nodo ROS2 en la laptop utiliza esta información para calcular el torque equivalente.

---

# 8. Ejecución del Sistema

## 1. Establecimiento de la Red Local
Se conectaron ambos robots (Maestro y Esclavo) mediante cables Ethernet a un switch de red común a la computadora de control.
Se configuró una dirección IP estática en la estación de trabajo dentro del rango de los robots (ej. 192.168.1.115).

## 2. Acceso a la Interfaz uFactory Studio
Se debe abrir un un navegador web e ingresar la dirección IP del robot maestro (ej. 192.168.1.175) para acceder a la interfaz de control embebida. Dentro de la configuración se activo el modo manual.

## 3. Inicialización de la Interfaz ROS2 con los Robots
Este proceso habilita los drivers ROS2, MoveIt y MoveIt Servo, permitiendo que los tópicos y servicios del robot estén disponibles. Se debe inicializar el robot maestro y esclavo, ejecutando esto en dos terminales diferentes:
```
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.175 hw_ns:=master
ros2 launch xarm_xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.167 hw_ns:=slave
```

## 4. Cargar codigo al ESP32

Se debe cargar el codigo desarrollado en el microcontrolador. Este condigo publica el valor de fuerza medida.

## 5. Ejecutar el micro-ROS agent
Se debe ejecutar el micro-ROS agent para que el topico de fuerza sea visible por los demas nodos.

```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## 6. Verificar nodos

Verificamos que todos los topicos existe.
```
ros2 topic list
```
Se pueden observar los topicos necesarios:
```
/force_esp32
/master/joint_states
/master/robot_states
/servo_server/delta_joint_cmds
/slave/joint_states
```

## 7. Ejecutar los nodos del sistema
Una vez preparado el sistema, procedemos a ejecutar los nodos correspondientes del maestro y el esclavo, cada uno se corre en una terminal diferente.

```
ros2 run teleop master
ros2 run teleop slave
```

## 8. Graficar

Mientras el sistema se este ejecutando, ejecutamos el nodo que se encarga de graficar la fuerza recibida, asi como los torques articulares y el error de los joints entre el maestro y esclavo
```
ros2 run teleop plot
```
---

# 8. Registro de Datos (ROS Bag)

Para análisis posterior, el sistema permite grabar todos los tópicos utilizando:

```id="rosbag_cmd"
ros2 bag record -a -o grabacion_completa_haptica
```
Y para su posterior analisis solo basta con ejecutar lo siguiente:
```
ros2 bag info nombre_de_tu_carpeta
```
Una vez echo esto, podemos graficar los datos ejecutando el nodo que grafica los datos.

Esto permite analizar posteriormente:

* fuerzas
* posiciones
* tiempos de respuesta

---


# 14. Conclusión

Este proyecto demuestra la implementación de un sistema de teleoperación robótica con retroalimentación háptica, integrando:

* control maestro–esclavo
* sensores de fuerza
* control basado en Jacobiano
* comunicación distribuida mediante ROS2

El sistema logra proporcionar retroalimentación física al operador, mejorando la percepción del entorno remoto y aumentando la seguridad de la manipulación robótica.

# Autores
Jose Eduardo Sanchez Martinez		      IRS | A01738476;
Josue Ureña Valencia				IRS | A01738940;
César Arellano Arellano			      IRS | A00839373;
Rafael André Gamiz Salazar			IRS | A00838280;

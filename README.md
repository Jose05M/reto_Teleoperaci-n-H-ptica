# Teleoperación Maestro–Esclavo con Retroalimentación de Fuerza usando ROS2 y xArm Lite6

## Descripción del Proyecto

Este proyecto implementa un sistema de **teleoperación maestro–esclavo** utilizando dos robots **xArm Lite6**, donde:

* El **robot maestro** es manipulado por el usuario.
* El **robot esclavo** replica los movimientos del maestro en tiempo real.
* Un **sensor de fuerza conectado a un ESP32** detecta contacto del robot esclavo con el entorno.
* La fuerza detectada se transmite al maestro para generar **retroalimentación háptica**, creando un efecto de **muro virtual**.

El sistema utiliza **ROS2**, **MoveIt Servo**, y **micro-ROS** para comunicación entre dispositivos.

---

# Arquitectura del Sistema

El sistema está compuesto por tres elementos principales:

1. Robot Maestro (xArm Lite6)
2. Robot Esclavo (xArm Lite6)
3. Sensor de fuerza conectado a ESP32

Todos los dispositivos se comunican mediante ROS2.

```
                  Switch Ethernet
         ┌─────────────┬─────────────┬─────────────┐
         │             │             │
     Laptop        Robot Maestro   Robot Esclavo
         │
         │ WiFi
         │
       ESP32
   (Sensor de fuerza)
```

### Flujo de información

1. El **robot maestro publica sus estados articulares**.
2. El **robot esclavo sigue el movimiento del maestro** mediante control de velocidad.
3. Cuando el esclavo interactúa con un objeto, el **sensor de fuerza mide la interacción**.
4. La fuerza es publicada por el **ESP32 mediante micro-ROS**.
5. Un nodo ROS2 calcula el **torque equivalente usando el Jacobiano**.
6. El torque se aplica al robot maestro generando **retroalimentación háptica**.

---

# Tecnologías Utilizadas

* ROS2
* MoveIt Servo
* micro-ROS
* Python (rclpy)
* ESP32
* Jacobiano de manipulador
* Control de teleoperación maestro–esclavo

---

# Nodos del Sistema

## 1. Nodo de Seguimiento del Esclavo

Este nodo permite que el robot esclavo siga al maestro mediante control de velocidad.

Control implementado:

```
q̇ = Kv q̇_master + Kp (q_master − q_slave)
```

donde:

* `q_master` posición del maestro
* `q_slave` posición del esclavo
* `Kv` ganancia de velocidad
* `Kp` ganancia proporcional

Este controlador permite:

* Seguimiento rápido
* Corrección de error de posición
* Movimiento continuo y estable

### Topics

Suscripciones:

```
/master/joint_states
/slave/joint_states
```

Publicación:

```
/servo_server/delta_joint_cmds
```

---

## 2. Nodo de Retroalimentación de Fuerza

Este nodo recibe la fuerza del sensor y calcula el torque equivalente en las articulaciones del robot maestro.

### Conversión Fuerza → Torque

Se utiliza el **Jacobiano transpuesto**:

```
τ = Jᵀ F
```

donde:

* `J` Jacobiano del robot
* `F` fuerza cartesiana
* `τ` torque en articulaciones

### Topics

Suscripciones:

```
/force_esp32
/joint_states
```

Publicación:

```
/ufactory/joint_cmds
```

---

# Sensor de Fuerza con ESP32

El ESP32 ejecuta un nodo **micro-ROS** que publica la fuerza medida.

### Topic publicado

```
/force_esp32
```

Tipo de mensaje:

```
std_msgs/Float32
```

Ejemplo de mensaje:

```
data: 3.5
```

---

# Configuración de Red

Todos los dispositivos están conectados a la misma red local.

Ejemplo de configuración IP:

```
Laptop:        192.168.1.10
Robot Maestro: 192.168.1.208
Robot Esclavo: 192.168.1.209
ESP32:         192.168.1.50
```

La laptop ejecuta el **micro-ROS Agent**, que permite la comunicación entre el ESP32 y ROS2.

### Ejecutar el agente

```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

# Frecuencias del Sistema

| Componente      | Frecuencia |
| --------------- | ---------- |
| Control esclavo | 200–500 Hz |
| MoveIt Servo    | 400 Hz     |
| Sensor ESP32    | 50–100 Hz  |

---

# Seguridad del Sistema

Para evitar comportamientos inestables se aplican varias protecciones:

### Saturación de torque

```
τ = clip(τ, -τ_max, τ_max)
```

### Saturación de velocidad

```
q̇ = clip(q̇, -v_max, v_max)
```

### Filtro de fuerza

Se utiliza un promedio móvil para reducir ruido del sensor.

### Umbral de contacto

```
if |F| < threshold:
    F = 0
```

---

# Comportamiento Esperado

### Sin contacto

```
Maestro → movimiento libre
Esclavo → replica movimiento
```

### Con contacto

```
Esclavo toca objeto
        ↓
Sensor detecta fuerza
        ↓
τ = JᵀF
        ↓
Maestro siente resistencia
```

Esto genera un **muro virtual** que impide que el operador continúe moviendo el robot.

---

# Aplicaciones

Este tipo de sistema se utiliza en:

* Teleoperación robótica
* Manipulación remota
* Robótica médica
* Cirugía robótica
* Control háptico
* Robótica colaborativa

---

# Posibles Mejoras

1. Control bilateral completo (4-channel control)
2. Compensación de latencia
3. Modelado dinámico del robot
4. Control de impedancia
5. Sensores de fuerza de 6 ejes
6. Control adaptativo

---

# Referencias

* Siciliano, B. – *Robotics: Modelling, Planning and Control*
* Craig, J. – *Introduction to Robotics*
* Documentación de ROS2
* Documentación de MoveIt Servo
* Documentación de micro-ROS

---

# Autor

Proyecto de teleoperación maestro–esclavo desarrollado con ROS2 y xArm Lite6.

# Sistema de Teleoperación Bilateral Háptico con xArm Lite6

**Desarrollado por:** Eduardo Sánchez Martínez (Líder del Mañana)
**Institución:** Tecnológico de Monterrey
**Tecnologías:** ROS 2 Humble, Python, C++, MoveIt Servo, micro-ROS (ESP32)

---

# 1. Descripción del Proyecto

Este proyecto implementa un sistema de **teleoperación maestro–esclavo con retroalimentación háptica** utilizando dos manipuladores **xArm Lite6**. El objetivo principal es permitir que un operador controle remotamente un robot esclavo mientras recibe **retroalimentación de fuerza** cuando el robot remoto entra en contacto con un objeto.

El sistema incluye:

* Un **robot maestro** manipulado por el operador.
* Un **robot esclavo** que replica los movimientos del maestro.
* Un **sensor de fuerza conectado a un ESP32**, encargado de medir la interacción con el entorno.
* Un sistema de control que **bloquea físicamente el robot maestro** cuando se detecta una colisión.

Una característica importante del sistema es su **seguridad unidireccional**:

* El robot maestro **se bloquea cuando el esclavo detecta una colisión**.
* El operador puede **retroceder libremente para liberar el sistema**.

Esto previene daños tanto al robot como al objeto con el que interactúa.

---

# 2. Arquitectura del Sistema

## Hardware

El sistema está compuesto por los siguientes elementos:

* **Robot Maestro:** xArm Lite6 (modo de control de velocidad/estado).
* **Robot Esclavo:** xArm Lite6 (modo seguidor).
* **Sensor de Fuerza:** Celda de carga conectada a un ESP32.
* **Laptop de control:** Ejecuta ROS2 y los nodos de control.
* **Red de comunicación:** Conexión Ethernet para robots y WiFi para ESP32.

### Topología de red

```id="arch_net"
                Switch Ethernet
         ┌───────────┬───────────┬───────────┐
         │           │           │
      Laptop     Robot Maestro  Robot Esclavo
         │
         │ WiFi
         │
        ESP32
   (Sensor de fuerza)
```

La laptop funciona como **centro de control**, ejecutando los nodos ROS2 y el **micro-ROS Agent** que conecta el ESP32 con el sistema ROS.

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
| `/master/vc_set_joint_velocity`  | xarm_msgs/MoveVelocity | Comando de velocidad del maestro |

## Servicios principales

| Servicio            | Función                              |
| ------------------- | ------------------------------------ |
| `/master/set_state` | Controla el estado del robot maestro |
| `/master/set_mode`  | Configura el modo de operación       |

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

# 7. Lógica de Control Principal

El comportamiento del sistema puede describirse con la siguiente lógica:

```id="logic_block"
if current_force > threshold and not locked:
    call_service(state=3)   # Activar freno
    locked = True

if locked and delta_q < -0.002:
    call_service(state=0)   # Liberar freno
    locked = False
```

Esto garantiza que:

* el robot se detenga ante colisiones
* el operador pueda liberar el sistema fácilmente

---

# 8. Sensor de Fuerza con ESP32

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

# 9. Registro de Datos (ROS Bag)

Para análisis posterior, el sistema permite grabar todos los tópicos utilizando:

```id="rosbag_cmd"
ros2 bag record -a -o grabacion_haptica
```

Esto permite analizar posteriormente:

* fuerzas
* posiciones
* tiempos de respuesta

---

# 10. Resultados Esperados

## Gráfica de Posición

Se espera observar:

* movimiento continuo del maestro
* una **meseta (plateau)** cuando ocurre el bloqueo

## Gráfica de Fuerza

Se observa:

* un **pico de fuerza**
* coincidencia temporal con la activación del freno

---

# 11. Seguridad del Sistema

Se implementaron varias protecciones:

### Saturación de fuerza

```id="sat_force"
F = clip(F, -Fmax, Fmax)
```

### Saturación de torque

```id="sat_tau"
τ = clip(τ, -τmax, τmax)
```

### Filtro de ruido

Promedio móvil de la fuerza para reducir ruido del sensor.

---

# 12. Aplicaciones

Este sistema puede aplicarse en:

* teleoperación remota
* manipulación de objetos delicados
* robótica médica
* cirugía asistida por robot
* robótica colaborativa
* investigación en control háptico

---

# 13. Posibles Mejoras

El sistema puede mejorarse con:

* Control bilateral completo (4-channel control)
* Compensación de latencia
* Control de impedancia
* Sensores de fuerza de 6 ejes
* Modelado dinámico del robot
* Control adaptativo

---

# 14. Referencias

* Siciliano, B. – *Robotics: Modelling, Planning and Control*
* Craig, J. – *Introduction to Robotics: Mechanics and Control*
* Documentación oficial de ROS2
* Documentación de MoveIt Servo
* Documentación de micro-ROS

---

# 15. Conclusión

Este proyecto demuestra la implementación de un sistema de **teleoperación robótica con retroalimentación háptica**, integrando:

* control maestro–esclavo
* sensores de fuerza
* control basado en Jacobiano
* comunicación distribuida mediante ROS2

El sistema logra proporcionar **retroalimentación física al operador**, mejorando la percepción del entorno remoto y aumentando la seguridad de la manipulación robótica.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from xarm_msgs.srv import SetInt16
from xarm_msgs.msg import RobotMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class MasterHapticResetNode(Node):
    def __init__(self):
        super().__init__('master_haptic_node')
        
        # Configuración QoS para compatibilidad con ESP32
        qos_esp32 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- CLIENTE DE SERVICIO ---
        # Usamos este servicio para mandar el estado 3 (bloqueo) y 0 (libre)
        self.state_client = self.create_client(SetInt16, '/master/set_state')

        # --- SUSCRIPCIONES ---
        self.force_sub = self.create_subscription(
            Float32, '/force_esp32', self.force_callback, qos_esp32)
        
        self.master_sub = self.create_subscription(
            JointState, '/master/joint_states', self.master_callback, 10)
        
        self.status_sub = self.create_subscription(
            RobotMsg, '/master/robot_states', self.status_callback, 10)
        
        # --- VARIABLES DE ESTADO ---
        self.current_force = 0.0
        self.umbral_fuerza = 6.5   # Newtons para activar el freno
        self.frenado_activo = False
        self.real_state = 0
        self.last_q = None

        self.get_logger().info("Nodo Maestro con Auto-Reset Iniciado.")

    def status_callback(self, msg):
        # Monitoreamos el estado real reportado por el hardware
        self.real_state = msg.state

    def force_callback(self, msg):
        self.current_force = abs(msg.data)
        
        # 1. LÓGICA DE BLOQUEO (Si hay impacto y el robot no está ya frenado)
        if self.current_force > self.umbral_fuerza and not self.frenado_activo:
            self.get_logger().warn(f"¡COLISIÓN! Fuerza: {self.current_force:.1f}N. Bloqueando Maestro.")
            self.cambiar_estado_robot(3) # Estado 3: Pausa/Brake
            self.frenado_activo = True

    def master_callback(self, msg):
        if self.last_q is None:
            self.last_q = np.array(msg.position)
            return

        # Calculamos la dirección del movimiento del Maestro
        current_q = np.array(msg.position)
        delta_q = current_q - self.last_q
        self.last_q = current_q

        # 2. LÓGICA DE AUTO-RESET (Liberación por retroceso)
        # Si el robot está bloqueado, pero detectamos que el usuario jala hacia arriba
        if self.frenado_activo:
            # Joint 2 o 3 negativo suele ser subir el brazo (revisa tu configuración)
            if delta_q[1] < -0.002 or delta_q[2] < -0.002:
                self.get_logger().info("Retroceso detectado. Liberando frenos para maniobra.")
                self.cambiar_estado_robot(0) # Estado 0: Operativo
                self.frenado_activo = False

    def cambiar_estado_robot(self, estado):
        if not self.state_client.service_is_ready():
            return
        req = SetInt16.Request()
        req.data = estado
        self.state_client.call_async(req)

def main():
    rclpy.init()
    node = MasterHapticResetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al cerrar el nodo, intentamos dejar el robot libre por seguridad
        if node.frenado_activo:
            node.cambiar_estado_robot(0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
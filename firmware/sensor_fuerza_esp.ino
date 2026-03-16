#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>

// ========================= CONFIGURACIÓN WIFI =========================
const char* ssid     = "2_2!";
const char* password = "12345678";
const char* agent_ip = "10.205.111.211";
const int agent_port = 8888;
// =====================================================================

// ========================= CONFIGURACIÓN SENSOR =======================
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const float k = 830.0;                      // Constante del resorte en N/m
const float distancia_referencia = 0.0760; // Distancia sin compresión en m
const float umbral_deformacion = 0.0005;   // 0.5 mm
const float alpha = 0.1;                   // Filtro exponencial
// =====================================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RMCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RMW_RET_OK)){error_loop();}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Entidades de Micro-ROS
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Variables del filtro
float distancia_filtrada = 0.0;
bool primera_lectura = true;

// Periodo de muestreo/publicación
const float dt = 0.05;   // 50 ms

void error_loop() {
  while (1) {
    delay(100);
  }
}

float leerDistancia() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter / 1000.0;   // mm -> m
  } else {
    return -1.0;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;

  if (timer != NULL) {
    float distancia_actual = leerDistancia();

    if (distancia_actual > 0) {
      // Inicializar filtro con primera lectura válida
      if (primera_lectura) {
        distancia_filtrada = distancia_actual;
        primera_lectura = false;
      } else {
        distancia_filtrada = alpha * distancia_actual +
                             (1.0 - alpha) * distancia_filtrada;
      }

      float x = distancia_referencia - distancia_filtrada;

      if (x < 0) x = 0;
      if (x < umbral_deformacion) x = 0;

      float fuerza = k * x;

      msg.data = fuerza;
      rcl_publish(&publisher, &msg, NULL);

      // Monitoreo serial
      Serial.print("Distancia cruda: ");
      Serial.print(distancia_actual, 4);
      Serial.print(" m | Distancia filtrada: ");
      Serial.print(distancia_filtrada, 4);
      Serial.print(" m | Deformacion: ");
      Serial.print(x, 4);
      Serial.print(" m | Fuerza publicada: ");
      Serial.print(fuerza, 3);
      Serial.println(" N");
    } else {
      Serial.println("Error de lectura del VL53L0X");
    }
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_force_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "force_esp32"));

  const unsigned int timer_timeout = dt * 1000;

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inicializar I2C y sensor
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Iniciando sensor VL53L0X...");
  if (!lox.begin()) {
    Serial.println("Error: no se detecto el sensor VL53L0X");
    while (1) {
      delay(10);
    }
  }

  Serial.println("Sensor detectado correctamente");

  // Conexión WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");

  // Configurar transporte micro-ROS por WiFi
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);

  state = WAITING_AGENT;
  msg.data = 0.0;

  Serial.println("Sistema listo para publicar fuerza en ROS 2");
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );

      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}

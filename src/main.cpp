#include <WiFi.h>
#include <PubSubClient.h>

// Datos de tu red WiFi
const char* ssid = "TP-Link_4CC6";
const char* password = "V3A4WL2021";

// Datos del broker MQTT
const char* mqtt_server = "192.168.7.104";  // IP de tu broker Mosquitto (ajusta según tu red)

uint16_t led1 = 15;
uint16_t pwm_value = 0;  // Variable global compartida
uint16_t ledChannel = 0; //Canal
uint16_t frecuencia = 5000;  //Frecuencia de trabajo
uint16_t resolucion = 12; //Resolucion 

// Crear cliente WiFi y cliente MQTT
WiFiClient espClient;
PubSubClient client(espClient);

QueueHandle_t tuberia1;

void Task1_com(void *parameters);
void Task2_led(void *parameters);


void mqtt_motor(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);
  pwm_value = message.toInt();
  Serial.println(pwm_value);

  xQueueSend(tuberia1, &pwm_value, portMAX_DELAY);

}

void setup() {
  pinMode(led1, OUTPUT);
  Serial.begin(9600);  // Iniciar puerto serial

  // Conexión al WiFi
  Serial.println("Conectando al WiFi...");
  WiFi.begin(ssid, password);  // Conectar al WiFi

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado al WiFi");
  Serial.println(WiFi.localIP());  // Mostrar IP del ESP32


  // Establecer servidor MQTT
  client.setServer(mqtt_server, 1883);  // Establecer servidor MQTT
  client.setCallback(mqtt_motor);  // Usar la función mqqtt_motor como callback

  ledcSetup(ledChannel, frecuencia, resolucion);
  ledcAttachPin(led1, ledChannel);

  tuberia1 = xQueueCreate(10, sizeof(int));

  // Haciendo binding de las tasks al scheduler
  xTaskCreatePinnedToCore(
    Task1_com,      // Funcion de la tarea
    "Com",          // Nombre interno para esa tarea ID
    2048,           // Memoria para la tarea (particion)
    NULL,           // Parámetros
    3,              // Prioridad
    NULL,           // Handler
    1               // Core del MPU
  );

  xTaskCreatePinnedToCore(
    Task2_led,      // Funcion de la tarea
    "LED",          // Nombre interno para esa tarea ID
    2048,           // Memoria para la tarea (particion)
    NULL,           // Parámetros
    3,              // Prioridad
    NULL,           // Handler
    1               // Core del MPU
  );

}

void loop() {
  // El loop se deja vacío ya que la tarea se está manejando con FreeRTOS
}


void Task1_com(void *parameters) {
  while (1) {
    // Verifica la conexión y reconecta si es necesario
    if (!client.connected()) {
      Serial.println("Desconectado del broker MQTT. Intentando reconectar...");
      while (!client.connected()) {
        if (client.connect("ESP32Client")) {
          Serial.println("Reconectado al broker MQTT");
            client.subscribe("led/pwm");
        } else {
          Serial.print("Error de reconexión, rc=");
          Serial.print(client.state());
          delay(5000);  // Esperar 5 segundos antes de intentar nuevamente
        }
      }
    }
    
    client.loop();  // Mantener la conexión y procesar los mensajes MQTT
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Pequeña espera para no saturar
  }
}


void Task2_led(void *parameters) {
uint16_t PWM_led = 0;
while (1)
{
if(xQueueReceive(tuberia1, &PWM_led, portMAX_DELAY)) {
  ledcWrite(ledChannel, PWM_led);

}
 vTaskDelay(100 / portTICK_PERIOD_MS);  // Pequeña espera para no saturar
}
}



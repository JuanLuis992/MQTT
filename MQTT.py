import paho.mqtt.client as mqtt
from nicegui import ui

# Configuración de MQTT
broker = '192.168.7.104'  # Dirección del broker MQTT (puede ser IP si no estás en localhost)
port = 1883  # Puerto MQTT predeterminado

# Función para conectar al broker MQTT
def on_connect(client, userdata, flags, rc):
    print(f'Conectado con código {rc}')
    # Suscripción al tema
    client.subscribe("led/pwm")

# Función para recibir los mensajes de MQTT
def on_message(client, userdata, msg):
    print(f"Mensaje recibido en el tema {msg.topic}: {msg.payload.decode()}")

# Crear el cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Conectar al broker MQTT
client.connect(broker, port, 60)

# Iniciar el loop MQTT en segundo plano
client.loop_start()

# Función para publicar el valor del slider en el tema MQTT
def publish_slider_value(value):
    print(f"Valor del slider: {value}")
    client.publish("led/pwm", str(value))  # Publica el valor del slider
    # Actualiza la etiqueta con el nuevo valor del slider
    label_value.text = f'Valor del PWM: {value}'

# Crear la interfaz NiceGUI
label_value = ui.label('Control de LED PWM: 0')  # Encabezado con el valor inicial del slider

# Crear el slider
slider = ui.slider(min=0, max=255, value=0, on_change=lambda e: publish_slider_value(e.value))

# Iniciar la aplicación NiceGUI
ui.run()

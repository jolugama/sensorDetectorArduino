
// quitar barra baja del archivo
namespace CONFIG {

const char* ssid = "ssid de tu wifi";
const char* password = "contraseña";

IPAddress staticIP(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 1);   //routers
IPAddress subnet(255, 255, 255, 0);  //netmask
IPAddress dns(192, 168, 2, 1);

const char* mqttServer = "192.168.2.51";
const uint16_t mqttPort = 1883;
const char* mqttUser = "nombreUsuarioMqtt";
const char* mqttPassword = "passwordUsuarioMqtt";

const char* topicPubSensors = "hall/sensores";
const char* topicPubAmbientalSensor= "hall/ambientalSensor";

const char* topicSubAlarmado = "hall/alarmado";
}  // namespace CONFIG
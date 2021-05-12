#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);

namespace mqtt {

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void callback(char* topic, byte* payload, int length) {
    Serial.print("Msg mqtt: [");
    Serial.print(topic);
    Serial.print("] ");

    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }

    if (strcmp(topic, "porter/ServerOn") == 0) {
        if ((char)payload[0] == '1') {
            Serial.println("siiii ok");
            // camServer::startCameraServer();
        } else {
            Serial.println("nooooo");
            // camServer::stopCameraServer();
        }
    } else if (strcmp(topic, "porter/flash") == 0) {
        // if ((char)payload[0] == '1') {
        //     digitalWrite(FLASH, HIGH);
        // } else {
        //     digitalWrite(FLASH, LOW);
        // }
    }
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("MQTT, Conectando...");
        // Create a random client ID
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str(), CONFIG::mqttUser, CONFIG::mqttPassword)) {
            Serial.println("Ok");
            // SUBSCRIPCIONES MOSQUITTO. AÑADIR AQUÍ TODAS.
            client.subscribe("porter");
            client.subscribe("porter/flash");
            // client.subscribe("porter/ServerOn");
            // client.subscribe("porter/door");
            // client.subscribe("porter/pir");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

}  // namespace mqtt
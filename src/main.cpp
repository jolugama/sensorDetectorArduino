// https://descubrearduino.com/interrupciones-esp32-gpio/

// GENERALES
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>  // I2C
#include <driver/dac.h>

#include "Arduino.h"

// SENSORES
#include <Adafruit_Sensor.h>

// PRIVADAS
#include "CONFIG.h"
#include "bme680_.h"
#include "mqtt.h"  // mosquito

// const byte pir1 = 25;
// const byte pir2 = 26;
// const byte mc38 = 35;
// const byte max9814 = adc
// apds9960 i2c

// free memory
unsigned int __bss_end;
unsigned int __heap_start;
void *__brkval;
//end free memory

unsigned long currentMillis;
unsigned long lastTimeSensorDoorMillis = 0;
const int intervalsensorDoor = 500;

unsigned long lastTimeMqttMillis = 0;
const int intervalMqtt = 7000;
unsigned long lastTimeBme680Millis = 0;
const int intervalBme680 = 1200000;  // TODO CAMBIAR A 60*20*1000  (20 minutos)
unsigned long lastTimeMax9814Millis = 0;
const int intervalMax9814 = 300;
String bmeData;

const int nTimesSensorDoor = 0;  // ajustar.
const int nTimesPir = 0;
const int numRangeNoise = 3300;  // nivel mínimo de ruido para avisar de ruido por el altavoz

unsigned long lastTimeRefreshMillis = 0;
const int intervalRefresh = 20000;  // cada 20 segundos refresco

char data[230];  //para enviar mttq a node-red

struct Sensor {
    const byte PIN;
    unsigned int numberDetected;
    unsigned long lastTrigger;  //millis
    bool detected;
};

// SENSORES
Sensor pir1 = {25, 0, 0, false};
Sensor pir2 = {26, 0, 0, false};
Sensor mc38 = {35, 0, 0, false};
Sensor max9814 = {32, 0, 0, false};  //micrófono

// Con el atributo IRAM_ATTR estamos declarando que el código compilado se colocará en la RAM interna (IRAM)
//  ISR (Interruption Service Rutine).
void IRAM_ATTR isrPir1() {
    pir1.numberDetected += 1;
    pir1.lastTrigger = millis();
    Serial.printf("isr pir1\n");
    // Serial.printf("isr pir1 %u\n", pir1.numberDetected);
    // pir1.detected = true;
}
void IRAM_ATTR isrPir2() {
    pir2.numberDetected += 1;
    pir2.lastTrigger = millis();
    Serial.printf("isr pir2\n");
    // Serial.printf("isr pir2 %u\n", pir2.numberDetected);
    // pir2.detected = true;
}
void IRAM_ATTR isrMc38() {
    mc38.numberDetected += 1;
    mc38.lastTrigger = millis();
    Serial.printf("isr mc38\n");
    // mc38.detected = true;
}

void IRAM_ATTR isrMax9814() {
    max9814.numberDetected += 1;
    // max9814.detected = true;
}

void setup() {
    Serial.begin(115200);

    pinMode(pir1.PIN, INPUT_PULLUP);
    pinMode(pir2.PIN, INPUT_PULLUP);
    pinMode(mc38.PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(pir1.PIN), isrPir1, RISING);
    attachInterrupt(digitalPinToInterrupt(pir2.PIN), isrPir2, RISING);
    attachInterrupt(digitalPinToInterrupt(mc38.PIN), isrMc38, RISING);

    //WIFI
    if (WiFi.config(CONFIG::staticIP, CONFIG::gateway, CONFIG::subnet, CONFIG::dns, CONFIG::dns) == false) {
        Serial.println("Wifi fallido. Revisar configuración.");
    }
    WiFi.begin(CONFIG::ssid, CONFIG::password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("WiFi contectado en http://");
    Serial.println(WiFi.localIP());

    // MOSQUITTO
    client.setServer(CONFIG::mqttServer, CONFIG::mqttPort);
    client.setCallback(mqtt::callback);

    bme680::init();

    // valores para max9814 que es analógico.
    analogReadResolution(12);
    analogSetPinAttenuation(max9814.PIN, ADC_6db);
}

int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
}

void loop() {
    currentMillis = millis();

    // MOSQUITO
    while (!client.connected()) {
        mqtt::reconnect();
    }
    client.loop();
    delay(150);  // doy un respiro al microcontrolador.

    if (pir1.numberDetected > nTimesPir) {
        Serial.println("pir1 detected" );
        // pir1.numberDetected = 0;
        pir1.detected = true;
        detachInterrupt(pir1.PIN);
    }
    if (pir2.numberDetected > nTimesPir) {
        Serial.println("pir2 detected" );
        // pir2.numberDetected = 0;
        pir2.detected = true;
        detachInterrupt(pir2.PIN);
    }
    // con el número que comparo(nTimesSensorDoor), elimino ruido y posible falsos positivos. graduar.
    if (mc38.numberDetected > nTimesSensorDoor) {
        delay(100);
        // Serial.println("mc38");
        Serial.println("mc38 detected");
        // mc38.numberDetected = 0;
        mc38.detected = true;
        detachInterrupt(mc38.PIN);
    }

    if (currentMillis - lastTimeMax9814Millis > intervalMax9814) {
        lastTimeMax9814Millis = currentMillis;
        // int microValue = analogRead(max9814.PIN);
        // elimino ruido. Max 4095. Solo sonidos altos.
        if (analogRead(max9814.PIN) > numRangeNoise) {
            max9814.detected = true;
        }
    }

    if (currentMillis - lastTimeMqttMillis > intervalMqtt) {
        // reseteo si o si cada x segundos las veces detectadas.
        pir1.numberDetected = 0;
        pir2.numberDetected = 0;
        mc38.numberDetected = 0;
        max9814.numberDetected = 0;

        if (mc38.detected || pir1.detected || pir2.detected || max9814.detected) {
            lastTimeMqttMillis = currentMillis;
            String json = " {\"door\": " + String(mc38.detected) + ",\"pir1\": " + String(pir1.detected) + ", \"pir2\":" + String(pir2.detected) + ",\"audio\":" + String(max9814.detected) + "}";
            json.toCharArray(data, json.length() + 1);
            client.publish(CONFIG::topicPubSensors, data);
            if (pir1.detected) {
                attachInterrupt(pir1.PIN, isrPir1, RISING);
                Serial.println("vuelvo a escuchar pir1");
            }
            if (pir2.detected) {
                attachInterrupt(pir2.PIN, isrPir2, RISING);
                Serial.println("vuelvo a escuchar pir2");
            }
            if (mc38.detected) {
                attachInterrupt(mc38.PIN, isrMc38, RISING);
                Serial.println("vuelvo a escuchar sensor puerta");
            }
            if (max9814.detected) {
            }
            pir1.detected = false;
            pir2.detected = false;
            mc38.detected = false;
            max9814.detected = false;
        }
    }

    // // cada x segundos, borro las veces detectadas.
    // if (currentMillis - lastTimeRefreshMillis > intervalRefresh) {
    //     lastTimeRefreshMillis = currentMillis;
    //     mc38.numberDetected = 0;
    //     pir1.numberDetected = 0;
    //     pir2.numberDetected = 0;
    //      max9814.numberDetected= 0;
    // }

    if (currentMillis - lastTimeBme680Millis > intervalBme680) {
        lastTimeBme680Millis = currentMillis;
        bmeData = bme680::getData();

        // transformar string a char
        char charBuffer[bmeData.length() + 1];
        bmeData.toCharArray(charBuffer, bmeData.length() + 1);
        client.publish(CONFIG::topicPubAmbientalSensor, charBuffer);

        Serial.println(bmeData);
    }
}
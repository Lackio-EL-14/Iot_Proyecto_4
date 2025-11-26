#include <pgmspace.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define THINGNAME "Object_PP"

const char WIFI_SSID[] = "Flia LAMAS";
const char WIFI_PASSWORD[] = "kf142004";
const char AWS_IOT_ENDPOINT[] = "a1fmgwdlekclx6-ats.iot.us-east-2.amazonaws.com";

// Topics
#define AWS_IOT_SHADOW_UPDATE "$aws/things/Object_PP/shadow/update"
#define AWS_IOT_SHADOW_UPDATE_DELTA "$aws/things/Object_PP/shadow/update/delta"
#define AWS_IOT_SHADOW_UPDATE_ACCEPTED "$aws/things/Object_PP/shadow/update/accepted"
#define AWS_IOT_SHADOW_UPDATE_REJECTED "$aws/things/Object_PP/shadow/update/rejected"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
#define AWS_IOT_PUBLISH_TOPIC "esp32/sensor"

// Pines
const int SERVO1_PIN = 32;    // Servo 1
const int SERVO2_PIN = 25;    // Servo 2 
const int PIR_SENSOR_PIN = 27; // Sensor movimiento HC-SR501
const int IR_SENSOR_PIN = 26;  // Sensor infrarrojo


static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUZjhTqVDUvDuvy0GkUX9lGXzaZNwwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MTExNjE1Mzg0
MloXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMkZ0QPu8RQbLg7la8+L
zeKnJl2AtQnL8NNFTFx3yij4YOyFWmjw9Anp1SseZbWEOJjGoUbdU5voq3StlrKb
MWY0SIbKD8rW1ZsYF8dMSVB2OJmNcwf6HOsdbq4uLnohdFChlJosh7fKXEcOjNa1
XB4OiKpjpWhPWHM6xYjb2Cuq+YN3XHCx0474KXyt5Bk/ddZ5jz0mru5jN+UKTNa4
H+QXj9HhzTp/MTR5no9YVnJPYihhMX6dGcTC72OoaiuYCklhrnknKVaCpqx+IJhz
D2X3W4Gfn9ok1XxMfJd7bROr3rCDQHlprH4xXgdP2PxOdGPeO/JONqg8zBNWX0bT
OqMCAwEAAaNgMF4wHwYDVR0jBBgwFoAUz9dsYqWCiYU8K2kXMHjTt+m3ZoEwHQYD
VR0OBBYEFFKy+lTrXd+8M4UOO/UK7TVF7YORMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQB0XQ+TJMbpPUE281GS/h0BCXle
XjYxg5jP/MMgDyltRI3vRn1/mSkPVbHA57sPRkqlbVBmWQTAxx7zOly1w45VX4C/
D5k48CTWeKBhoOcuK1nTYOyy56pYSgyz9KDpkZ6APBXY54J12icLE1f9PacuUs+F
ewE4we+jsJo/fbA5YYGWdFhfBEZM6dRLAZWkFnGruUDSuFVoUew2u+qtVpElO81g
1iEshj+SLXavbVbIs+DzN/rV4JygHIMxWwv2HEd12c9XtTOnP36PXrdAB2Hw5AkZ
btfdZmtw9ntY4lGt9s2/25HoSrzEsiDFpeiBAkLUZlgsaMccud8nVtZ7NuSR
-----END CERTIFICATE-----
)KEY";

static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAyRnRA+7xFBsuDuVrz4vN4qcmXYC1Ccvw00VMXHfKKPhg7IVa
aPD0CenVKx5ltYQ4mMahRt1Tm+irdK2WspsxZjRIhsoPytbVmxgXx0xJUHY4mY1z
B/oc6x1uri4ueiF0UKGUmiyHt8pcRw6M1rVcHg6IqmOlaE9YczrFiNvYK6r5g3dc
cLHTjvgpfK3kGT911nmPPSau7mM35QpM1rgf5BeP0eHNOn8xNHmej1hWck9iKGEx
fp0ZxMLvY6hqK5gKSWGueScpVoKmrH4gmHMPZfdbgZ+f2iTVfEx8l3ttE6vesINA
eWmsfjFeB0/Y/E50Y9478k42qDzME1ZfRtM6owIDAQABAoIBAEXqfNmyPAyyemvh
ZX8t6vGvDU6azdZPGjZJtIe9egKOgBwbLNipVR8RNfScx924z3iGLlIV5tigpuOU
6nzEKgOPj9uU6rKpyG/4i2PV6PNZdsoj/Gc8lMtFW1KhabU6ZlcWJjUH4FOMvlJS
A7MuMI2enfMnW72NQIInX7/6NwjO9cEmZVKhYKQ5pFPm2yAyqS3ELCYo5D7iXGdq
Ein5T6fW7AWRjgjHEDye0mlMj5VJVY8j6q/jPOa2M4RAuW4Y9zBwuZ5j22c0++O0
rmQ29PHPZEaCxzfXTwmKQJny5p01xm4T8hF8VSilT9XB2ueRZVMj/NC2EZoa5MZt
LALTu8kCgYEA6jRzck9vlj8t8ozjGiw6+7oDvYENxvXc+2hXgvQAnB3rY7dphEF6
FvR3rzJjaOusQi8tQ0iImsQ2TL4H/eKfV0XKv7akS8Gj8akbMC2ZJ87BCmDZRyZQ
GDtUm7nRf9uA/39amZXH2zwYONvIOvoosUJA/qePjo3ezmDPNH5yQ50CgYEA29C3
PQwd8iahl5m5+A7ccNktp6SafcxW9yfU5JpgqzV5zLV04ppK62bWLwtamU5+maMO
IuhjP3GWk38Y+MlX6qXAgNHMHKJcAP2sJfd+J3HFly6mZVi9PG3/KynpyF2n5JNR
nWEPb0BimOT/O9bjOh68A4uKfKRsRcqvUVfEwz8CgYBxwI3+DFT/Xapcb09yi98O
Gpfsd/0QEKSHO3OduBN/wAxvP0sxoHfDJHPpFQkxkZ31+2H5lwY6XWieN4I8LtED
m3NRUE1WOjP9kQwSh2Mm8YXTG50MsLAgRNjgVg1KE3wBHIMNq7EJa5O8Mgt7kbsn
9S0KQqo4K/H2vRCGx4+KBQKBgC3Nl+EInVTP5+wHBFFEB3scRf8gtXBxG830ZtAb
uvjx86dMSEbPj7zteMPkwcQgLxEkgNWVRtj5ej7RxQpSKhei4jH1hWqSCknKkgx3
PbpKYrnV5qZHj2cAJKOB4Ez39vwSFneMugKcj+78CJMXe9GohdTXRswiFs2UJmP0
vfNRAoGBAL9nfbqmgxhvhY6x4GOSMRQu8AGLhahMSocwj5rdO1jGTEef3aqer5Pf
OPeFSjD1/AWJ1IxzrvrwwATTRxXEbrivHjywCde4o6ZFNapgSi/5qBDmgtrOYWTg
0w7Tf4aGTcuAWWK7tHdmTNKX3NaVJZDbCNM7NXenF5MrtmAPbABS
-----END RSA PRIVATE KEY-----
)KEY";


class BaseSensor {
protected:
    int pin;
    bool currentState;
    bool lastState;
    unsigned long lastDetectionTime;
    unsigned long lastStateChangeTime;
    unsigned long lastReadTime;
    const long readInterval;
    const long debounceTime;

public:
    BaseSensor(int sensorPin, long readInt = 1000, long debounce = 4000) 
        : pin(sensorPin), currentState(false), lastState(false),
          lastDetectionTime(0), lastStateChangeTime(0), lastReadTime(0),
          readInterval(readInt), debounceTime(debounce) {}
    
    virtual void begin() = 0;
    virtual void update() = 0;
    
    bool isDetected() const { return currentState; }
    unsigned long getLastDetectionTime() const { return lastDetectionTime; }
};


class PIRSensor : public BaseSensor {
public:
    PIRSensor(int sensorPin) : BaseSensor(sensorPin, 1000, 4000) {}
    
    void begin() override {
        pinMode(pin, INPUT);
        Serial.println("✅ Sensor PIR HC-SR501 inicializado (GPIO " + String(pin) + ")");
    }
    
    void update() override {
        unsigned long now = millis();
        if (now - lastReadTime < readInterval) return; // No leer antes del intervalo
        lastReadTime = now;

        bool reading = digitalRead(pin) == HIGH;
        bool stateChanged = false;

        if (reading && !currentState) {
            // Movimiento detectado
            if (now - lastStateChangeTime >= debounceTime) {
                currentState = true;
                lastDetectionTime = now;
                lastStateChangeTime = now;
                stateChanged = true;
                Serial.println("🚨 ¡MOVIMIENTO PIR DETECTADO!");
            }
        } 
        else if (!reading && currentState) {
            // Movimiento desapareció
            if (now - lastStateChangeTime >= debounceTime) {
                currentState = false;
                lastStateChangeTime = now;
                stateChanged = true;
                Serial.println("✅ PIR: Sin movimiento");
            }
        }

        if (stateChanged) {
            lastState = currentState; 
        }
    }

};


class IRSensor : public BaseSensor {
public:
    IRSensor(int sensorPin) : BaseSensor(sensorPin, 1000, 3000) {}
    
    void begin() override {
        pinMode(pin, INPUT);
        Serial.println("✅ Sensor Infrarrojo inicializado (GPIO " + String(pin) + ")");
    }
    
    void update() override {
        unsigned long now = millis();
        if (now - lastReadTime < readInterval) return; // No leer antes del intervalo
        lastReadTime = now;

        bool reading = digitalRead(pin) == LOW; 
        bool stateChanged = false;

        if (reading && !currentState) {
            if (now - lastStateChangeTime >= debounceTime) {
                currentState = true;
                lastDetectionTime = now;
                lastStateChangeTime = now;
                stateChanged = true;
                Serial.println("📡 ¡OBJETO DETECTADO POR IR!");
            }
        }
        else if (!reading && currentState) {
            if (now - lastStateChangeTime >= debounceTime) {
                currentState = false;
                lastStateChangeTime = now;
                stateChanged = true;
                Serial.println("✅ IR: Sin objeto");
            }
        }

        if (stateChanged) {
            lastState = currentState;
        }
    }

};


class DualServoController {
private:
    Servo servo1;
    Servo servo2;
    int pin1;
    int pin2;
    int currentAngle1;
    int currentAngle2;

public:
    DualServoController(int servoPin1, int servoPin2) 
        : pin1(servoPin1), pin2(servoPin2), currentAngle1(0), currentAngle2(0) {}
    
    void begin() {
        servo1.attach(pin1);
        servo2.attach(pin2);
        servo1.write(0);
        servo2.write(0);
        currentAngle1 = 0;
        currentAngle2 = 0;
        Serial.println("✅ Servo 1 inicializado (GPIO " + String(pin1) + ")");
        Serial.println("✅ Servo 2 inicializado (GPIO " + String(pin2) + ")");
    }
    
    // Mover ambos servos sincronizadamente al mismo ángulo
    void setAngleBoth(int angle) {
        angle = constrain(angle, 0, 180);
        currentAngle1 = angle;
        currentAngle2 = angle;
        servo1.write(currentAngle1);
        servo2.write(currentAngle2);
        Serial.print("✅ Ambos servos a: ");
        Serial.print(angle);
        Serial.println("°");
    }
    
    void open() { 
        setAngleBoth(180); 
        Serial.println("🔓 TAPA ABIERTA");
    }
    
    void close() { 
        setAngleBoth(0); 
        Serial.println("🔒 TAPA CERRADA");
    }
    
    void mid() { 
        setAngleBoth(90); 
        Serial.println("⏸️ TAPA MEDIA");
    }
    
    int getAngle1() const { return currentAngle1; }
    int getAngle2() const { return currentAngle2; }
};


class Command {
private:
    bool pending;
    String type;
    int angle;

public:
    Command() : pending(false), type(""), angle(0) {}
    
    void set(String cmdType, int cmdAngle = 0) {
        pending = true;
        type = cmdType;
        angle = cmdAngle;
    }
    
    bool isPending() const { return pending; }
    String getType() const { return type; }
    int getAngle() const { return angle; }
    void clear() { pending = false; type = "idle"; }
};


class AWSIoTManager {
private:
    WiFiClientSecure* net;
    PubSubClient* client;
    String thingName;
    Command* pendingCommand;
    static AWSIoTManager* instance;

public:
    AWSIoTManager(const char* thing) : thingName(thing) {
        net = new WiFiClientSecure();
        client = new PubSubClient(*net);
        instance = this;
    }
    
    ~AWSIoTManager() {
        delete client;
        delete net;
    }
    
    void begin() {
        net->setCACert(AWS_CERT_CA);
        net->setCertificate(AWS_CERT_CRT);
        net->setPrivateKey(AWS_CERT_PRIVATE);
        client->setServer(AWS_IOT_ENDPOINT, 8883);
        client->setBufferSize(1024);
        client->setCallback(staticMessageHandler);
    }
    
    bool connect() {
    Serial.println("🔌 Conectando a AWS IoT Core...");
    
    if (client->connect(thingName.c_str())) {
        Serial.println("✅ AWS IoT conectado!");
        
        // ⭐ ESPERAR antes de suscribirse
        delay(1000);
        

        bool deltaSubscribed = client->subscribe(AWS_IOT_SHADOW_UPDATE_DELTA, 1);
        Serial.print("📡 Suscripción a Delta: ");
        Serial.println(deltaSubscribed ? "✅ EXITOSA" : "❌ FALLÓ");
        
        delay(500);
        
        bool acceptedSubscribed = client->subscribe(AWS_IOT_SHADOW_UPDATE_ACCEPTED, 1);
        Serial.print("📡 Suscripción a Accepted: ");
        Serial.println(acceptedSubscribed ? "✅ EXITOSA" : "❌ FALLÓ");
        
        delay(500);
        
        bool rejectedSubscribed = client->subscribe(AWS_IOT_SHADOW_UPDATE_REJECTED, 1);
        Serial.print("📡 Suscripción a Rejected: ");
        Serial.println(rejectedSubscribed ? "✅ EXITOSA" : "❌ FALLÓ");
        
        delay(500);
        
        bool subTopicSubscribed = client->subscribe(AWS_IOT_SUBSCRIBE_TOPIC, 1);
        Serial.print("📡 Suscripción a Subscribe Topic: ");
        Serial.println(subTopicSubscribed ? "✅ EXITOSA" : "❌ FALLÓ");
        

        if (!deltaSubscribed) {
            Serial.println("⚠️ Reintentando suscripción al Delta...");
            delay(1000);
            deltaSubscribed = client->subscribe(AWS_IOT_SHADOW_UPDATE_DELTA, 1);
            Serial.println(deltaSubscribed ? "✅ EXITOSA en 2do intento" : "❌ FALLÓ en 2do intento");
        }
        
        return true;
    }
    
    Serial.print("❌ Fallo, rc=");
    Serial.println(client->state());
    return false;
    }
    bool isConnected() { return client->connected(); }
    void loop() { client->loop(); }
    
    void reportState(int servo1Angle, int servo2Angle, bool motionDetected, 
                     bool irDetected, unsigned long lastMotionTime, 
                     unsigned long lastIRTime, unsigned long uptime, String command) {
        StaticJsonDocument<768> doc;
        JsonObject state = doc.createNestedObject("state");
        JsonObject reported = state.createNestedObject("reported");
        
        reported["servo1_angle"] = servo1Angle;
        reported["servo2_angle"] = servo2Angle;
        reported["motion_detected"] = motionDetected;
        reported["infrared_detected"] = irDetected;
        reported["last_motion_time"] = lastMotionTime;
        reported["last_infrared_time"] = lastIRTime;
        reported["uptime"] = uptime;
        reported["command"] = command;
        
        String status = "idle";
        if (servo1Angle == 180 && servo2Angle == 180) status = "open";
        else if (servo1Angle == 0 && servo2Angle == 0) status = "closed";
        else if (servo1Angle == 90 && servo2Angle == 90) status = "mid";
        reported["status"] = status;
        
        char buffer[768];
        serializeJson(doc, buffer);
        
        if (client->publish(AWS_IOT_SHADOW_UPDATE, buffer)) {
            Serial.println("✅ Shadow reportado");
        }
    }
    
    void publishData(int servo1Angle, int servo2Angle, bool motionDetected, 
                     bool irDetected, unsigned long lastMotionTime, 
                     unsigned long lastIRTime, unsigned long uptime) {
        StaticJsonDocument<768> doc;
        
        doc["device"] = thingName;
        doc["servo1_angle"] = servo1Angle;
        doc["servo2_angle"] = servo2Angle;
        doc["motion_detected"] = motionDetected;
        doc["infrared_detected"] = irDetected;
        doc["last_motion_time"] = lastMotionTime;
        doc["last_infrared_time"] = lastIRTime;
        doc["uptime"] = uptime;
        
        char buffer[768];
        serializeJson(doc, buffer);
        
        client->publish(AWS_IOT_PUBLISH_TOPIC, buffer);
        Serial.println("📊 Datos publicados");
    }
    
    void setPendingCommand(Command* cmd) { pendingCommand = cmd; }
    
    static void staticMessageHandler(char* topic, byte* payload, unsigned int length) {
        if (instance) instance->messageHandler(topic, payload, length);
    }
    
    void messageHandler(char* topic, byte* payload, unsigned int length) {
    Serial.println();
    Serial.println("====================================");
    Serial.print("📨 Topic: ");
    Serial.println(topic);
    
    char message[length + 1];
    for (unsigned int i = 0; i < length; i++) {
        message[i] = (char)payload[i];
    }
    message[length] = '\0';
    
    Serial.print("📄 Payload: ");
    Serial.println(message);
    
    // Ignorar mensajes de la Rule
    if (strcmp(topic, AWS_IOT_SUBSCRIBE_TOPIC) == 0) {
        StaticJsonDocument<512> testDoc;
        if (deserializeJson(testDoc, message) == DeserializationError::Ok) {
            if (testDoc.containsKey("timestamp")) {
                Serial.println("⚠️ Mensaje de Rule ignorado");
                Serial.println("====================================");
                return;
            }
        }
    }
    
    if (strcmp(topic, AWS_IOT_SHADOW_UPDATE_DELTA) == 0) {
        Serial.println("🔄 ¡DELTA RECIBIDO! Procesando...");
        
        StaticJsonDocument<1024> doc;
        DeserializationError error = deserializeJson(doc, message);
        
        if (error) {
            Serial.print("❌ Error parseando JSON: ");
            Serial.println(error.c_str());
            return;
        }
        
        JsonObject state = doc["state"];
        if (state.isNull()) return;
        
        if (state.containsKey("command")) {
            String command = state["command"].as<String>();
            Serial.print("🎤 Comando detectado: ");
            Serial.println(command);
            
            if (command == "idle") {
                Serial.println("⏸️ Comando 'idle' ignorado - ya sincronizado");
                Serial.println("====================================");
                return;
            }
            
            if (command == "open") {
                pendingCommand->set("open", 180);
            } else if (command == "close") {
                pendingCommand->set("close", 0);
            } else if (command == "mid") {
                pendingCommand->set("mid", 90);
            } else if (command == "set_angle") {
                int angle = 90;
                if (state.containsKey("servo1_angle")) {
                    angle = state["servo1_angle"];
                }
                pendingCommand->set("set_angle", angle);
            }
        }
        else if (state.containsKey("servo1_angle") && !state.containsKey("command")) {
            int angle = state["servo1_angle"];
            pendingCommand->set("set_angle", angle);
        }
    }
    
    Serial.println("====================================");
}
};

AWSIoTManager* AWSIoTManager::instance = nullptr;

class WiFiManager {
public:
    static void connect(const char* ssid, const char* password) {
        Serial.print("🌐 Conectando a WiFi: ");
        Serial.println(ssid);
        
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 30) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println();
            Serial.println("✅ WiFi conectado!");
            Serial.print("📍 IP: ");
            Serial.println(WiFi.localIP());
        }
    }
    
    static bool isConnected() {
        return WiFi.status() == WL_CONNECTED;
    }
};


class SmartTrashCan {
private:
    PIRSensor* pirSensor;
    IRSensor* irSensor;
    DualServoController* servos;
    AWSIoTManager* awsManager;
    Command* command;
    String lastCommand;

    unsigned long lastPublish;
    const long publishInterval = 5000;

    bool lastPIRState;
    bool lastIRState;

    // 🔹 Variables para control de reportes por si hay loop ya sabemos donde es
    bool lastReportedPIR = false;
    bool lastReportedIR = false;
    int lastReportedServo1 = -1;
    int lastReportedServo2 = -1;
    String lastReportedCommand = "idle";

public:
    SmartTrashCan() {
        pirSensor = new PIRSensor(PIR_SENSOR_PIN);
        irSensor = new IRSensor(IR_SENSOR_PIN);
        servos = new DualServoController(SERVO1_PIN, SERVO2_PIN);
        awsManager = new AWSIoTManager(THINGNAME);
        command = new Command();
        lastCommand = "idle";
        lastPublish = 0;
        lastPIRState = false;
        lastIRState = false;

        lastReportedPIR = false;
        lastReportedIR = false;
        lastReportedServo1 = 0;
        lastReportedServo2 = 0;
        lastReportedCommand = "idle";
    }

    ~SmartTrashCan() {
        delete pirSensor;
        delete irSensor;
        delete servos;
        delete awsManager;
        delete command;
    }

    void begin() {
        Serial.println("====================================");
        Serial.println("🗑️ BASURERO INTELIGENTE - ESP32");
        Serial.println("====================================");

        pirSensor->begin();
        irSensor->begin();
        servos->begin();

        WiFiManager::connect(WIFI_SSID, WIFI_PASSWORD);
        awsManager->begin();
        awsManager->setPendingCommand(command);

        if (WiFiManager::isConnected()) {
            if (awsManager->connect()) {
                delay(2000);

                awsManager->reportState(
                    servos->getAngle1(),
                    servos->getAngle2(),
                    false,
                    false,
                    0,
                    0,
                    0,
                    "idle"
                );

                Serial.println("✅ Sistema inicializado en estado 'idle'");
            }
        }

        Serial.println("====================================");
        Serial.println("🚀 Sistema listo!");
        Serial.println("====================================");
    }

    void update() {
        // Reconexiones
        if (!WiFiManager::isConnected()) WiFiManager::connect(WIFI_SSID, WIFI_PASSWORD);
        if (!awsManager->isConnected()) awsManager->connect();

        // Mantener conexión MQTT
        awsManager->loop();

        // Procesar comando pendiente
        if (command->isPending()) {
            processCommand();
        }

        // Guardar estados anteriores
        bool prevPIR = lastPIRState;
        bool prevIR = lastIRState;

        // Actualizar sensores
        pirSensor->update();
        irSensor->update();

        // Obtener estados actuales
        lastPIRState = pirSensor->isDetected();
        lastIRState = irSensor->isDetected();

        //  Solo reportar Shadow si hubo cambio real de estado
        if ((prevPIR != lastPIRState) || (prevIR != lastIRState) || (lastCommand != "idle")) {
            reportState();
            lastCommand = "idle"; // Reiniciar comando
        }

        //  Publicar telemetría cada publishInterval ms
        unsigned long now = millis();
        if (now - lastPublish >= publishInterval) {
            lastPublish = now;
            publishData();
        }
    }

private:
    void processCommand() {
        Serial.println();
        Serial.println("⚙️ EJECUTANDO COMANDO");

        String cmdType = command->getType();
        int angle = command->getAngle();

        if (cmdType == "open") {
            servos->open();
            lastCommand = "open";
        } else if (cmdType == "close") {
            servos->close();
            lastCommand = "close";
        } else if (cmdType == "mid") {
            servos->mid();
            lastCommand = "mid";
        } else if (cmdType == "set_angle") {
            servos->setAngleBoth(angle);
            lastCommand = "set_angle";
        }

        command->clear();

        // Dar tiempo a que el servo se mueva
        delay(500);

        awsManager->reportState(
            servos->getAngle1(),
            servos->getAngle2(),
            pirSensor->isDetected(),
            irSensor->isDetected(),
            pirSensor->getLastDetectionTime(),
            irSensor->getLastDetectionTime(),
            millis() / 1000,
            "idle"
        );

        Serial.println("✅ Comando completado y reportado como 'idle'");
    }

    void reportState() {

        bool currentPIR = pirSensor->isDetected();
        bool currentIR  = irSensor->isDetected();
        int currentServo1 = servos->getAngle1();
        int currentServo2 = servos->getAngle2();


        if (currentPIR != lastReportedPIR || currentIR != lastReportedIR ||
            currentServo1 != lastReportedServo1 || currentServo2 != lastReportedServo2 ||
            lastCommand != lastReportedCommand) {

            awsManager->reportState(
                currentServo1,
                currentServo2,
                currentPIR,
                currentIR,
                pirSensor->getLastDetectionTime(),
                irSensor->getLastDetectionTime(),
                millis() / 1000,
                lastCommand
            );

            // Actualizar últimos reportes
            lastReportedPIR = currentPIR;
            lastReportedIR = currentIR;
            lastReportedServo1 = currentServo1;
            lastReportedServo2 = currentServo2;
            lastReportedCommand = lastCommand;

            Serial.println("📡 Estado reportado a AWS (cambio detectado)");
        }
        else {
            Serial.println("⏸️ Sin cambios, no se reporta");
        }
    }

    void publishData() {
        awsManager->publishData(
            servos->getAngle1(),
            servos->getAngle2(),
            pirSensor->isDetected(),
            irSensor->isDetected(),
            pirSensor->getLastDetectionTime(),
            irSensor->getLastDetectionTime(),
            millis() / 1000
        );
    }
};


SmartTrashCan* trashCan;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    trashCan = new SmartTrashCan();
    trashCan->begin();
}

void loop() {
    trashCan->update();
    yield();
}
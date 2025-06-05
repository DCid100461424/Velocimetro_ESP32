#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Arduino.h>
#include <OneWire.h> //Para el sensor de temperatura
#include <DallasTemperature.h> //Para el sensor de temperatura
#include "BluetoothSerial.h"
#include "BLE2902.h"


// BLE UUIDs (must match Flutter app)
// TODO: Volver a generarlos de nuevo de forma aleatoria? Mirar si hay algún estándar para los de conexión y notify
//          eg: dc9a393d-bebf-4d11-8235-01878bbec7fe
// TODO: Hacer que el SERVICE_UUID vaya en modo servidor GATT? Habría que ver que implicaría eso.
#define SERVICE_UUID        "4fafc201-0000-459e-8fcc-c5c9c331914b" //TODO: Cambiarlo también en la app ("-0000-")
#define WRITE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define NOTIFY_CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80d751a2"
#define AVG_SPEED_UUID "4fafc201-0001-459e-8fcc-c5c9c331914b"
#define TOTAL_DISTANCE_UUID "4fafc201-0002-459e-8fcc-c5c9c331914b"
#define TEMPERATURE_UUID "4fafc201-0003-459e-8fcc-c5c9c331914b"
#define TOTAL_TIME_UUID "4fafc201-0004-459e-8fcc-c5c9c331914b"  //Mandamos esto siquiera?
#define STOP_SIGNAL "STOP"
#define MAX_DIAMETER 1023 // Para evitar números excesivamente grandes o inf


// GPIO and variables
const int SENSOR_PIN = 32;
const int TEMP_SENSOR_PIN = 25;
volatile unsigned long pulseCount = 0;
float W_DIAMETER = 0.0;
float wCircunferenceKm = 0.0;
float totalDistance = 0.0;
bool deviceConnected = false;
//unsigned long lastConnectTime = 0;
unsigned long lastDisconnectTime = 0;
unsigned long trainStartTime = 0;

const unsigned long TIMEOUT = 300000; // 5 minutes
bool inTraining = false;

// Objetos temperatura
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);

// BLE Objects
BLEServer* pServer;
BLECharacteristic* pWriteCharacteristic;
BLECharacteristic* pNotifyCharacteristic;
BLECharacteristic* pTemperatureCharacteristic;
BLECharacteristic* pAvgSpeedCharacteristic;
BLECharacteristic* pDistanceCharacteristic;
BLECharacteristic pTimeCharacteristic(TOTAL_TIME_UUID);

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        
        //lastConnectTime = millis();
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        lastDisconnectTime = millis();
        inTraining = false;
        pulseCount = 0;
    }
};

class WriteCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        double convValue = atof(value.c_str());
        Serial.printf("\nDiameter: %.2f", convValue);
        Serial.println("");

        // Si el valor no es un número, comprueba si es la señal de terminar. Si lo es, para el entrenamiento y resetea los valores. Después termina la función.
        if(convValue == 0){
            if(!value.compare(STOP_SIGNAL)){
                inTraining = false;
                pulseCount = 0;
                W_DIAMETER = 0.0;
                wCircunferenceKm = 0.0;
                totalDistance = 0.0;
            }
            return;
        }

        if (convValue > 0 && convValue < MAX_DIAMETER) {
            // Guardamos el diámetro que nos llega, convirtiendolo de cm a metros
            W_DIAMETER = convValue /100.0;
            // Calculamos la circunferencia en km ahora para no volver a hacerlo cada segundo
            wCircunferenceKm = W_DIAMETER*PI*0.001; 
            pulseCount = 0;
            if (!inTraining){
                trainStartTime = millis();
                inTraining = true;
            }
        }
    }
};

void IRAM_ATTR countPulse() {
    pulseCount++;
}

void setup() {
    Serial.begin(9600);
    DS18B20.begin();    // initialize the DS18B20 sensor
    
    // Configure sensor input
    pinMode(SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, RISING);

    // BLE Setup
    BLEDevice::init("ESP32-Speedometer");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    // Write characteristic for diameter
    pWriteCharacteristic = pService->createCharacteristic(
        WRITE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pWriteCharacteristic->setCallbacks(new WriteCallback());

    // Notify characteristic for speed
    pNotifyCharacteristic = pService->createCharacteristic(
        NOTIFY_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pNotifyCharacteristic->addDescriptor(new BLE2902());

    
    pTemperatureCharacteristic = pService -> createCharacteristic(
        TEMPERATURE_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pAvgSpeedCharacteristic = pService -> createCharacteristic(
        AVG_SPEED_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pDistanceCharacteristic = pService -> createCharacteristic(
        TOTAL_DISTANCE_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    

    // TODO: Poner las characteristics de los otros. Probablemente pueda hacer de primeras que sean NOTIFY y ya.
    // Podemos crearlos en la declaración y después añadirlos, como en este thread: https://forum.arduino.cc/t/ble-very-weak-signal/631751/12

    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();
}

void loop() {
    static unsigned long lastSend = 0;
    static unsigned long lastPulse = 0;
    float speed = 0.0;
    float lastSpeed = 0.0;
    float avgSpeed = 0.0;
    char speedStr[8];
    char avgSpeedStr[8];
    char distanceStr[8];
    char temperatureStr[8];
    
    
    if (deviceConnected) {
        // IMPORTANTE: elcódigo tiene en cuenta el cálculo una vez por segundo, si se modifica la frecuencia de actualización
        // que está abajo, hay que modificar el código.
        if (millis() - lastSend >= 1000) { // Update every second
            DS18B20.requestTemperatures();
            dtostrf(DS18B20.getTempCByIndex(0), 1, 1, temperatureStr);
            pTemperatureCharacteristic->setValue(temperatureStr);
            pTemperatureCharacteristic->notify();

            if(inTraining){
                // Calculate speed
                unsigned long currentPulses = pulseCount;
                //float rpm = (currentPulses - lastPulse) * (60.0); // Pulses per minute.
                //rpm (rev/min)= (currentPulses - lastPulse) (rev) /1 (s) * 60.0 (s/min);   // Pulses per minute. Se calculan los pulsos (revoluciones) que ha habido en un segundo. Como ambas variables de pulso están en ms, hay que pasarlos a s (/1000), y después a min (*60)
                float rpm = ((currentPulses - lastPulse) * 60.0); // Versión algo optimizada
                //float speed = (rpm * wCircunferenceKm * 3.6) / 60.0; // km/h
                //speed (km/h) = (rpm (rev/min) * wCircunferenceKm (m/rev) ) * 60.0 (min/h) / 1000 (m/km);   // km/h
                float speed = (rpm * wCircunferenceKm)*60.0; // Versión optimizada (sería más optimizado usar una variable de circunferencia ya multiplicada, o juntarla con el cálculo de rpm)
                
                totalDistance = wCircunferenceKm * currentPulses;
                //TODO: ¿Que la velocidad que envíe sea la media entre esta velocidad y la anterior? Así hay menos picos, y si tiene 0 en dos segundos seguidos estaría parada ( (0+0)/2 )
                //avgSpeed (km/h) = totalDistance (m) / ((millis-trainStartTime) (ms) / (1000 (ms/s) * 3600 (s/h) ))
                avgSpeed = totalDistance/ ((millis()-trainStartTime)/3600000.0);


                // Send via BLE
                dtostrf( (speed+lastSpeed)/2 , 1, 1, speedStr);
                //dtostrf(speed, 1, 1, speedStr);
                pNotifyCharacteristic->setValue(speedStr);
                pNotifyCharacteristic->notify();

                dtostrf(avgSpeed , 1, 1, avgSpeedStr);
                pAvgSpeedCharacteristic->setValue(avgSpeedStr);
                pAvgSpeedCharacteristic->notify();

                dtostrf(totalDistance , 1, 1, distanceStr);
                pDistanceCharacteristic->setValue(distanceStr);
                //pDistanceCharacteristic->setValue("120.0");
                pDistanceCharacteristic->notify();
                
                lastPulse = currentPulses;
                lastSend = millis();
                lastSpeed = speed;

                /**
                //debug print
                Serial.printf("\nDistance: %.2f, %s", totalDistance, distanceStr);
                Serial.printf("\nAvgSpeed: %.2f, %s\n", avgSpeed, avgSpeedStr);
                 */
            }

            delay(180);
        }

    }
    
    // Handle timeout
    if (!deviceConnected && (millis() - lastDisconnectTime >= TIMEOUT)) {
        inTraining = false;
        W_DIAMETER = 0.0;
        wCircunferenceKm = 0.0;
        totalDistance = 0.0;
        pulseCount = 0;
        lastPulse = 0;
    }

}

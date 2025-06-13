#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Arduino.h>
#include <OneWire.h> 
#include <DallasTemperature.h> 
#include "BluetoothSerial.h"
#include "BLE2902.h"



// UUIDs BLE (deben coincidir con los de la app Flutter)
#define SERVICE_UUID        "dc9a0000-bebf-4d11-8235-01878bbec7fe"
#define WRITE_CHARACTERISTIC_UUID "dc9a0100-bebf-4d11-8235-01878bbec7fe"
#define SPEED_CHARACTERISTIC_UUID "dc9a0200-bebf-4d11-8235-01878bbec7fe"
#define AVG_SPEED_UUID "dc9a0201-bebf-4d11-8235-01878bbec7fe"
#define TOTAL_DISTANCE_UUID "dc9a0202-bebf-4d11-8235-01878bbec7fe"
#define TEMPERATURE_UUID "dc9a0203-bebf-4d11-8235-01878bbec7fe"
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
unsigned long trainStartTime = 0;

bool inTraining = false;

// Objetos para temperatura
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);

// Objetos BLE
BLEServer* pServer;
BLECharacteristic* pWriteCharacteristic;
BLECharacteristic* pSpeedCharacteristic;
BLECharacteristic* pTemperatureCharacteristic;
BLECharacteristic* pAvgSpeedCharacteristic;
BLECharacteristic* pDistanceCharacteristic;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
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
    DS18B20.begin();    // inicializar el sensor DS18B20
    
    // Configurar entrada del sensor
    pinMode(SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, RISING);

    // Setup BLE
    BLEDevice::init("ESP32-Velocímetro");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    // Característica de escritura para el diámetro
    pWriteCharacteristic = pService->createCharacteristic(
        WRITE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pWriteCharacteristic->setCallbacks(new WriteCallback());

    // Características de notificación para enviar las mediciones
    pSpeedCharacteristic = pService->createCharacteristic(
        SPEED_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pSpeedCharacteristic->addDescriptor(new BLE2902());

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
    static unsigned long currentPulses = 0;
    float speed = 0.0;
    float lastSpeed = 0.0;
    float avgSpeed = 0.0;
    char speedStr[8];
    char avgSpeedStr[8];
    char distanceStr[8];
    char temperatureStr[8];
    
    
    if (deviceConnected) {
        // IMPORTANTE: el código tiene en cuenta el cálculo una vez por segundo. Si se modifica la frecuencia de actualización que está
        // abajo hay que modificar el código.
        if (millis() - lastSend >= 1000) { //Se calcula cada segundo
            DS18B20.requestTemperatures();
            dtostrf(DS18B20.getTempCByIndex(0), 1, 1, temperatureStr);
            pTemperatureCharacteristic->setValue(temperatureStr);
            pTemperatureCharacteristic->notify();

            if(inTraining){
                currentPulses = pulseCount;
                
                // wCircunferenceKm ya está en km, así que no hace falta transformarlo
                // speed (km/h) = (rps (rev/s) * wCircunferenceKm (km/rev) ) * 3600.0 (s/h)
                float speed = (currentPulses - lastPulse) * wCircunferenceKm *3600.0;

                totalDistance = wCircunferenceKm * currentPulses;
                
                //avgSpeed (km/h) = totalDistance (m) / ((millis-trainStartTime) (ms) / (1000 (ms/s) * 3600 (s/h) ))
                avgSpeed = totalDistance/ ((millis()-trainStartTime)/3600000.0);

                // Mandar las medidas a través de BLE

                if(speed>=0.0 && speed<300.0){ //Control de errores
                    dtostrf( (speed+lastSpeed)/2 , 1, 1, speedStr);
                    pSpeedCharacteristic->setValue(speedStr);
                    pSpeedCharacteristic->notify();
                }
                
                dtostrf(avgSpeed , 1, 1, avgSpeedStr);
                pAvgSpeedCharacteristic->setValue(avgSpeedStr);
                pAvgSpeedCharacteristic->notify();

                dtostrf(totalDistance , 1, 1, distanceStr);
                pDistanceCharacteristic->setValue(distanceStr);
                pDistanceCharacteristic->notify();
                
                // Prepara medidas para el próximo cálculo
                lastPulse = currentPulses;
                lastSend = millis();
                lastSpeed = speed;
            }

            delay(180);
        }

    }
}

#include <ArduinoBLE.h>
#include <MotorController.h>

#define MOTOR_PIN 10
#define PWM_FREQUENCY 31000

MotorController skateboard(MOTOR_PIN);

BLEService motorService("19B10000-E8F2-537E-4F6C-D104768A1214");  // create service

// create switch characteristic and allow remote device to read and write
BLEUnsignedCharCharacteristic percentCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN;  // pin to use for the LED

// Function Prototypes
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void percentCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic);

void setup() {
    Serial.begin(9600);
    while (!Serial)
        ;

    pinMode(ledPin, OUTPUT);  // use the LED pin as an output

    // Initialize skateboard
    skateboard.setFrequency(PWM_FREQUENCY);
    skateboard.setSpeed(0);

    // begin initialization
    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");

        while (1)
            ;
    }

    // set the local name peripheral advertises
    BLE.setLocalName("SkateBoard");
    // set the UUID for the service this peripheral advertises
    BLE.setAdvertisedService(motorService);

    // add the characteristic to the service
    motorService.addCharacteristic(percentCharacteristic);

    // add service
    BLE.addService(motorService);

    // assign event handlers for connected, disconnected to peripheral
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    // assign event handlers for characteristic
    percentCharacteristic.setEventHandler(BLEWritten, percentCharacteristicWritten);
    // set an initial value for the characteristic
    percentCharacteristic.setValue(0);

    // start advertising
    BLE.advertise();

    Serial.println(("Bluetooth device active, waiting for connections..."));
}

void loop() {
    // poll for BLE events
    BLE.poll();
}

void blePeripheralConnectHandler(BLEDevice central) {
    // central connected event handler
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
    // central disconnected event handler
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
}

void percentCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
    // central wrote new value to characteristic, update skateboard speed
    Serial.print("Characteristic event, written: ");

    short speed = percentCharacteristic.value();
    if (speed > 100) {
        percentCharacteristic.setValue(100);
        speed = 100;
    } else if (speed < 0) {
        percentCharacteristic.setValue(0);
        speed = 0;
    }

    if (speed > 50) {
        Serial.print("Speed Fast: ");
        Serial.println(speed);
        digitalWrite(ledPin, HIGH);
    } else {
        Serial.print("Speed Slow: ");
        Serial.println(speed);
        digitalWrite(ledPin, LOW);
    }
    skateboard.setSpeed(speed);
}
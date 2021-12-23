#include <ArduinoBLE.h>
#include <MotorController.h>

// Define constants
#define BLE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"

const short FORWARD_MOTOR_PIN = 10;
const short REVERSE_MOTOR_PIN = 8;
const unsigned short PWM_FREQUENCY = 31000;
const short LED_PIN = LED_BUILTIN;

// Initialize skateboard and blutooth objects
MotorController skateboard(FORWARD_MOTOR_PIN, REVERSE_MOTOR_PIN);
BLEService motorService(BLE_UUID);
BLEShortCharacteristic speedCharacteristic(BLE_UUID, BLERead | BLEWrite);

// Define Function Prototypes
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void speedCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic);

void setup() {
    // Initialize led and Serial for debugging and status updates.
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);

    // Initialize skateboard
    skateboard.setFrequency(PWM_FREQUENCY);
    skateboard.setSpeed(0);

    // begin initialization
    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");

        while (1) {
            // Bluetooth failed to initialize. Enter endless loop. Blink Error Code.
            digitalWrite(LED_PIN, HIGH);
            delay(1000);
            digitalWrite(LED_PIN, LOW);
            delay(1000);
        };
    }

    // set the local name peripheral advertises
    BLE.setLocalName("SkateBoard");
    // set the UUID for the service this peripheral advertises
    BLE.setAdvertisedService(motorService);
    // add the characteristic to the service
    motorService.addCharacteristic(speedCharacteristic);
    // add service
    BLE.addService(motorService);
    // assign event handlers for connected, disconnected to peripheral
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
    // assign event handlers for characteristic
    speedCharacteristic.setEventHandler(BLEWritten, speedCharacteristicWritten);
    // set an initial value for the characteristic
    speedCharacteristic.setValue(0);
    // start advertising
    BLE.advertise();
    // Print status message
    Serial.println(("Bluetooth device active, waiting for connections..."));
}

void loop() {
    // poll for BLE events
    BLE.poll();
}

/**
 * @brief Function to be called on ble conntection initialization
 * 
 * @param central - The connected device object instance
 */
void blePeripheralConnectHandler(BLEDevice central) {
    // central connected event handler
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
}

/**
 * @brief Function to be called on ble connection disconnected
 * 
 * @param central - The connected device object instance
 */
void blePeripheralDisconnectHandler(BLEDevice central) {
    // central disconnected event handler
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
}

/**
 * @brief Function to be called when a new speed value is written
 * 
 * @param central - The connected device object instance
 * @param characteristic - The characteristic object
 */
void speedCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
    // central wrote new value to characteristic, update skateboard speed
    Serial.println("Characteristic event, written: ");

    // Ensure the skateboard speed percentage is between 0 and 100
    short speed = speedCharacteristic.value();
    Serial.print("Char Value: ");
    Serial.println(speed);
    if (speed > 100) {
        Serial.println("Set Positive");
        speedCharacteristic.setValue(100);
        speed = 100;
    } else if (speed < -100) {
        Serial.println("Set Negative");
        speedCharacteristic.setValue(-100);
        speed = -100;
    }
    Serial.print("New Char Value: ");
    Serial.println(speedCharacteristic.value());

    if (speed != 0) {
        Serial.println("Skateboard on");
        digitalWrite(LED_PIN, HIGH);
    } else {
        Serial.println("Skateboard off");
        digitalWrite(LED_PIN, LOW);
    }
    int maxLoop = 10000;
    int lastTime = millis();
    int currentSpeed;
    do {
        skateboard.setSpeed(speed);
        currentSpeed = skateboard.getSpeed();
        lastTime = millis();
    } while (currentSpeed != speed and (millis() - lastTime) <= maxLoop);
}
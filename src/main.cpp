#include <ArduinoBLE.h>
#include <MotorController.h>

// Define constants
#define BLE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SPEED_UUID "19B10000-E8F2-537E-4F6C-D104776A1219"
#define DIR_UUID "19B10000-E8F2-537E-4F6C-D604168C1217"

const short FORWARD_MOTOR_PIN = 10;
const short REVERSE_MOTOR_PIN = 8;
const unsigned short PWM_FREQUENCY = 31000;
const short LED_PIN = LED_BUILTIN;

// Initialize skateboard and blutooth objects
MotorController skateboard(FORWARD_MOTOR_PIN, REVERSE_MOTOR_PIN);
BLEService motorService(BLE_UUID);
BLEIntCharacteristic speedCharacteristic(SPEED_UUID, BLERead | BLEWrite);
BLEBoolCharacteristic forwardDirectionCharacteristic(DIR_UUID, BLERead | BLEWrite);

// Define Function Prototypes
void manageSkateboard(BLEDevice central);
void debugBleSkateboardRemote(BLEDevice central);

void setup() {
    // Initialize led and Serial for debugging and status updates.
    Serial.begin(9600);
    // while (!Serial);
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
    motorService.addCharacteristic(forwardDirectionCharacteristic);
    // add service
    BLE.addService(motorService);

    // set an initial value for the characteristic
    speedCharacteristic.setValue(0);
    forwardDirectionCharacteristic.setValue(true);
    // start advertising
    BLE.advertise();
    // Print status message
    Serial.println(("Bluetooth device active, waiting for connections..."));
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    

    // while the central is still connected to peripheral:
    manageSkateboard(central);
    // debugBleSkateboardRemote(central);
    
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

void manageSkateboard(BLEDevice central) {
    short desiredSpeed = 0;
    short currentSpeed = 0;
    bool forward = true;
    while (central.connected()) {
        if (speedCharacteristic.written() || forwardDirectionCharacteristic.written()) {
            desiredSpeed = speedCharacteristic.value();
            if (desiredSpeed > 100) {
                speedCharacteristic.setValue(100);
                desiredSpeed = 100;
            }
            forward = (bool)forwardDirectionCharacteristic.value();
            if (!forward)
                desiredSpeed *= -1;
        }
        currentSpeed = skateboard.getSpeed();
        if (currentSpeed != desiredSpeed) {
            skateboard.setSpeed(desiredSpeed);
        }
    }
}

void debugBleSkateboardRemote(BLEDevice central) {
    int oldSpeed = 0;
    int desiredSpeed = 0;
    bool forward = true;
    while (central.connected()) {
        if (speedCharacteristic.written()) {
            desiredSpeed = speedCharacteristic.value();
            if (desiredSpeed > (oldSpeed + 2) || desiredSpeed < (oldSpeed - 2)) {
                oldSpeed = desiredSpeed;
                Serial.print("Returned Speed:");
                Serial.println(desiredSpeed);
                forward = (bool)forwardDirectionCharacteristic.value();
                Serial.print("Current Forward Direction:");
                Serial.println(forward);
            }
        }
    }
}

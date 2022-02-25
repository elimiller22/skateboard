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
const short MIN_PWM_MOTOR_SPEED = 25;

// Initialize skateboard and blutooth objects
MotorController skateboard(FORWARD_MOTOR_PIN, REVERSE_MOTOR_PIN);
BLEService motorService(BLE_UUID);
BLEIntCharacteristic speedCharacteristic(SPEED_UUID, BLERead | BLEWrite);
BLEBoolCharacteristic forwardDirectionCharacteristic(DIR_UUID,
                                                     BLERead | BLEWrite);

// Define Function Prototypes
void manageSkateboard(BLEDevice central);
void stopSkateboard();

/**
 * @brief Function to run setup on arduino.
 *
 */
void setup() {
  // Initialize led and Serial for debugging and status updates.
  Serial.begin(9600);
  // while (!Serial);
  pinMode(LED_PIN, OUTPUT);

  // Initialize skateboard
  skateboard.setFrequency(PWM_FREQUENCY);
  skateboard.setMinPwmSpeed(MIN_PWM_MOTOR_SPEED);
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

/**
 * @brief Loop function on Arduino
 *
 */
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

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
  // If the skateboard isn't connected to a remote, ensure it is stopped.
  // Needed to prevent a runaway condition.
  stopSkateboard();
}

/**
 * @brief Function to manage the skateboard. Retrieves new values from
 * the characteristic, and writes the values to the skateboard speed
 * controller.
 *
 * @param central The central (remote) ble controller.
 */
void manageSkateboard(BLEDevice central) {
  // Define variables for skateboard management
  short desiredSpeed = 0;
  short currentSpeed = 0;
  bool forward = true;
  // While the ble controller is connected, control the skateboard.
  while (central.connected()) {
    // If the speed or direction changes, get the new values.
    if (speedCharacteristic.written() ||
        forwardDirectionCharacteristic.written()) {
      desiredSpeed = speedCharacteristic.value();
      if (desiredSpeed > 100) {
        speedCharacteristic.setValue(100);
        desiredSpeed = 100;
      }
      forward = (bool)forwardDirectionCharacteristic.value();
      if (!forward)
        desiredSpeed *= -1;
    }
    // Get the current speed of the skateboard
    currentSpeed = skateboard.getSpeed();
    // If the current skateboard speed doesn't match the desired speed, set the
    // speed.
    if (currentSpeed != desiredSpeed) {
      skateboard.setSpeed(desiredSpeed);
    }
  }
}

/**
 * @brief Function to check if the skateboard is running, and if it is to set
 * the speed to 0.
 *
 */
void stopSkateboard() {
  // Get the current speed of the skateboard
  short currentSpeed = skateboard.getSpeed();
  // If the current skateboard speed doesn't match the desired speed, set the
  // speed.
  if (currentSpeed != 0) {
    skateboard.setSpeed(0);
  }
}

# Sensors_Interface

## Arduino IDE

It's a software platform used for programming and uploading code to Arduino microcontrollers. It provides a user-friendly interface and a set of tools for writing, compiling, and uploading code to Arduino boards. The Arduino IDE simplifies the process of creating programs for a wide range of projects, from simple blinking LEDs to complex robotics and IoT applications.

### Steps to install Arduino IDE

- Visit the official Arduino IDE 2.0 download page  [Arduino IDE 2.0 Download](https://www.arduino.cc/en/software)
- Launch Arduino IDE 2.0 from your computer's application menu or by running the executable.
- Upon the first run, you may be prompted to select a workspace location. Choose or create a directory where your Arduino projects will be stored.

## ULTRASONIC SENSOR HC-SR04

### source code
```
const int trigPin = 9;  // Trigger pin of the ultrasonic sensor
const int echoPin = 10; // Echo pin of the ultrasonic sensor

void setup() {
  Serial.begin(9600);   // Initialize serial communication
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Send a pulse to the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the time it takes for the pulse to return
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in centimeters (speed of sound is approximately 343 meters per second)
  int distance = duration * 0.034 / 2;

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(1000);  // Wait for a second before taking another measurement
}
```

### circuit diagram


![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/6733b224-4521-4b32-9e22-fc2c493f70f8)


### wiring

+ Connect the VCC pin of the ultrasonic sensor to the 5V pin on the Arduino
+ Connect the GND pin of the ultrasonic sensor to the GND pin on the Arduino.
+ Connect the Trig pin of the ultrasonic sensor to Digital Pin 9 on the Arduino.
+ Connect the Echo pin of the ultrasonic sensor to Digital Pin 10 on the Arduino.


### output

![us_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/013d6da6-5eb2-4e48-bccc-c73cf5b73354)


## PASSIVE INFRARED SENSOR HC-SR505

### source code

```
const int pirPin = 2;  // PIR sensor output pin

void setup() {
  Serial.begin(9600);   // Initialize serial communication
  pinMode(pirPin, INPUT);
}

void loop() {
  int motionState = digitalRead(pirPin);

  if (motionState == HIGH) {
    Serial.println("Motion detected!");
  } else {
    Serial.println("No motion detected");
  }

  delay(1000);  // Delay to avoid rapid serial print
}

```

### circuit diagram


![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/fbab9bd3-5158-49ea-90eb-96b0b4a954f1)


### wiring
+ Connect the VCC pin of the PIR sensor to the 5V pin on the Arduino.
+ Connect the GND pin of the PIR sensor to the GND pin on the Arduino.
+ Connect the OUT (or DOUT) pin of the PIR sensor to Digital Pin 2 on the Arduino.


### output


![PIR_SS](https://github.com/benedict04/Sensors_Interface/assets/109859485/fe1c3f0b-db68-4d74-942b-f32de6cc465d)


## IR PROXIMITY

### source code

```
const int irSensorPin = 2;  // Digital pin connected to the output of the IR proximity sensor

void setup() {
  Serial.begin(9600);       // Initialize serial communication
  pinMode(irSensorPin, INPUT);
}

void loop() {
  // Read the digital output from the IR proximity sensor
  int proximityValue = digitalRead(irSensorPin);

  // Print the result to the Serial Monitor
  if (proximityValue == HIGH) {
    Serial.println("Object detected!");
  } else {
    Serial.println("No object detected");
  }

  delay(500);  // Delay for stability
}
```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/b2827089-1a27-4a80-b60d-7a961a2f328b)


### wiring

+ Connect the sensor's VCC to the Arduino's 5V.
+ Connect the sensor's GND to the Arduino's GND.
+ Connect the sensor's OUT to the Arduino's Digital Pin 2.


### output

![irp_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/9492c888-4eb5-4aba-bc75-660354ac7aaf)


## DHT 11 TEMPERATURE SENSOR

### source code

```
#include <DHT.h>

#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT sensor type (DHT11 or DHT22)

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  delay(2000); // Wait for 2 seconds between readings

  float temperature = dht.readTemperature(); // Read temperature in Celsius
  float humidity = dht.readHumidity();       // Read humidity

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
}

```

### circuit diagram


![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/89193b74-4591-48a0-9530-246eb4204bd7)


### wiring

+ Connect the sensor's VCC to the Arduino's 5V.
+ Connect the sensor's GND to the Arduino's GND.
+ Connect the sensor's DATA to the Arduino's Digital Pin 2.


### output

![dht_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/f9a6b594-07d6-48af-8099-b1f2e5fdc34b)


## BMP180 SENSOR

### source code

```
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  Serial.print("Temperature: ");
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  delay(1000);
}

```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/53284f6f-04ce-47a2-9a54-8d080b45d303)

### wiring

+ Connect the sensor's VCC to the Arduino's 5V.
+ Connect the sensor's GND to the Arduino's GND.
+ Connect the sensor's SDA to the Arduino's A4.
+ Connect the sensor's SCL to the Arduino's A5.

### output

![bmp_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/c9277213-337b-4328-88a4-d0323ba74e6f)


## JOYSTICK


### source code

```
const int xAxisPin = A0;  // Connect the X-axis of the joystick to analog pin A0
const int yAxisPin = A1;  // Connect the Y-axis of the joystick to analog pin A1

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read analog values from the joystick
  int xValue = analogRead(xAxisPin);
  int yValue = analogRead(yAxisPin);

  // Print the X and Y values to the Serial Monitor
  Serial.print("X-axis: ");
  Serial.print(xValue);
  Serial.print(", Y-axis: ");
  Serial.println(yValue);

  delay(500);  // Delay for stability
}
```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/ed8551cd-d40a-463e-b83b-c4001aeb682c)


### wiring

+ Connect the VRx (X-axis) of the joystick to Analog Pin A0 on the Arduino.
+ Connect the VRy (Y-axis) of the joystick to Analog Pin A1 on the Arduino.
+ Connect the GND of the joystick to the GND on the Arduino.
+ Connect the VCC of the joystick to the 5V on the Arduino.


### output

![js_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/fd931834-7d00-4797-a274-cdbd8f730ea5)


## SOUND SENSOR

### source code

```
const int soundSensorPin = A0; // Connect the analog output of the sound sensor to analog pin A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read analog value from the sound sensor
  int soundValue = analogRead(soundSensorPin);

  // Print the sound value to the Serial Monitor
  Serial.print("Sound Level: ");
  Serial.println(soundValue);

  delay(500);  // Delay for stability
}
```

### wiring

+ Connect the Analog Output of the sound sensor to Analog Pin A0 on the Arduino.
+ Connect the GND of the sound sensor to the GND on the Arduino.
+ Connect the VCC of the sound sensor to the 5V on the Arduino.


### output

![ss_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/84583ce6-5338-4224-88fc-5ac71e87872d)


## ROTARY ENCODER

### source code

```
// Rotary Encoder Pins
const int encoderPinA = 2;  // Connect to digital pin 2
const int encoderPinB = 3;  // Connect to digital pin 3

volatile int encoderPos = 0;  // Current position of the encoder
int lastEncoderPos = 0;       // Last position of the encoder

void setup() {
  Serial.begin(9600);

  // Configure encoder pins as INPUT_PULLUP to enable internal pull-up resistors
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void loop() {
  // Check if the encoder position has changed
  if (encoderPos != lastEncoderPos) {
    Serial.print("Encoder Position: ");
    Serial.println(encoderPos);

    lastEncoderPos = encoderPos;
  }
}

// Interrupt service routine to update encoder position
void updateEncoder() {
  // Read the current state of the encoder pins
  int encA = digitalRead(encoderPinA);
  int encB = digitalRead(encoderPinB);

  // Determine the direction of rotation based on the state changes
  if (encA == encB) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}
```

### diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/9d6135dc-3c51-4224-aa0c-3486929d6d67)


### wiring

+ Connect one side of the rotary encoder to the Arduino's 5V.
+ Connect the other side of the rotary encoder to the Arduino's GND.
+ Connect the A output of the rotary encoder to digital pin 2 on the Arduino.
+ Connect the B output of the rotary encoder to digital pin 3 on the Arduino.


### output

![rm_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/9f3aca68-679a-4dbe-b20b-3b47c2a23268)


## LIGHT DEPENDENT RESISTOR 


###source code

```
const int ldrPin = A0;  // Connect the LDR to analog pin A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read the analog value from the LDR
  int ldrValue = analogRead(ldrPin);

  // Print the LDR value to the Serial Monitor
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);

  delay(500);  // Delay for stability
}

```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/4440c107-c6c4-4b28-bb63-b4c201b5bddc)


### wiring

+ Connect one leg of the LDR to the Arduino's 5V.
+ Connect the other leg of the LDR to the analog pin A0 on the Arduino.
+ Connect a resistor from the same leg of the LDR to the GND on the Arduino.


### output

![ldr_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/3e14b360-c94d-4968-99ec-2055cf3ac36a)


## LM35 TEMPERATURE SENSOR

### source code 

```
const int lm35Pin = A0;  // Connect the LM35 to analog pin A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read the analog value from the LM35
  int lm35Value = analogRead(lm35Pin);

  // Convert the analog value to temperature in Celsius
  float temperatureCelsius = (lm35Value / 1024.0) * 500.0;

  // Print the temperature to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureCelsius);
  Serial.println(" °C");

  delay(1000);  // Delay for stability
}

```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/a34ef6d5-08ee-4598-88ee-24791aaad1c2)


### wiring

+ Connect the LM35's VCC to the Arduino's 5V.
+ Connect the LM35's GND to the Arduino's GND.
+ Connect the LM35's OUT to analog pin A0 on the Arduino.


### output

![lm_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/4868213f-49fa-4c7f-9719-23c7adb02bde)


## 7 SEGMENT DISPLAY

### source code

```
// Pin mapping for common cathode seven-segment display
const int segmentPins[] = {2, 3, 4, 5, 6, 7, 8};  // Connect these pins to segments a, b, c, d, e, f, and g
const int digitPin = 9;                              // Connect this pin to the common cathode pin

void setup() {
  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }
  pinMode(digitPin, OUTPUT);
}

void loop() {
  // Display numbers 0 to 9 on the seven-segment display
  for (int i = 0; i < 10; i++) {
    displayNumber(i);
    delay(1000);
  }
}

void displayNumber(int number) {
  // Define the segments for each digit
  const byte digitSegments[] = {
    B0111111,  // 0
    B0000110,  // 1
    B1011011,  // 2
    B1001111,  // 3
    B1100110,  // 4
    B1101101,  // 5
    B1111101,  // 6
    B0000111,  // 7
    B1111111,  // 8
    B1101111   // 9
  };

  // Activate the segments for the given number
  for (int i = 0; i < 7; i++) {
    bool isSegmentActive = bitRead(digitSegments[number], i);
    digitalWrite(segmentPins[i], isSegmentActive ? HIGH : LOW);
  }

  // Activate the display
  digitalWrite(digitPin, LOW);
  delay(5);  // A small delay to improve visibility
  digitalWrite(digitPin, HIGH);
}

```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/f0987df1-482c-4e7b-aa8e-cf7addd2d445)


### wiring

+ Connect the common cathode pin of the seven-segment display to the digitPin (pin 9).
+ Connect each segment pin (a, b, c, d, e, f, g) to the corresponding digital pins (2, 3, 4, 5, 6, 7, 8).


## HALL SENSOR

### source code

```
const int hallSensorPin = 2;  // Connect the Hall sensor signal pin to digital pin 2

void setup() {
  Serial.begin(9600);
  pinMode(hallSensorPin, INPUT);
}

void loop() {
  // Read the state of the Hall sensor
  int hallState = digitalRead(hallSensorPin);

  // Print the state to the Serial Monitor
  Serial.println(hallState);

  delay(500);  // Delay for stability
}

```

### circuit diagram


![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/852a6ea0-48c4-44e7-aadc-0c4c4f9e005d)


### wiring

+ Connect the signal pin of the Hall sensor module to digital pin 2 on the Arduino.
+ Connect the VCC pin of the Hall sensor module to the Arduino's 5V.
+ Connect the GND pin of the Hall sensor module to the Arduino's GND.


### output

![hs_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/b24d62d2-6fc9-42d1-b7d5-b4d7a84a2080)


## POTENTIOMETER

### source code

```
const int potentiometerPin = A0;  // Connect the potentiometer to analog pin A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read the analog value from the potentiometer
  int potValue = analogRead(potentiometerPin);

  // Print the potentiometer value to the Serial Monitor
  Serial.println(potValue);

  delay(100);  // Delay for stability
}

```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/c938c3e3-0f5e-49d3-b563-f81deb6a0ce3)


### wiring

+ Connect one leg of the potentiometer to the Arduino's 5V.
+ Connect the other leg of the potentiometer to the Arduino's GND.
+ Connect the middle pin (wiper) of the potentiometer to analog pin A0 on the Arduino.


### output


![pm_ss](https://github.com/benedict04/Sensors_Interface/assets/109859485/2264807a-f38d-47f4-b63c-9850f1f9b671)


## LIQUID CRYSTAL DISPLAY

### source code

```
#include <LiquidCrystal.h>

// Pin configuration for LCD
const int rs = 12;   // Register select pin
const int en = 11;   // Enable pin
const int d4 = 5;    // Data pin 4
const int d5 = 4;    // Data pin 5
const int d6 = 3;    // Data pin 6
const int d7 = 2;    // Data pin 7

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  lcd.begin(16, 2);  // Initialize the LCD with 16 columns and 2 rows
  lcd.print("Hello, Arduino!");
}

void loop() {
  // Your code here
}

```

### circuit diagram


![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/0b98ddfc-3266-4b40-980b-600965bedf60)


### wiring

+ Connect the LCD RS pin to digital pin 12.
+ Connect the LCD EN pin to digital pin 11.
+ Connect the LCD D4 pin to digital pin 5.
+ Connect the LCD D5 pin to digital pin 4.
+ Connect the LCD D6 pin to digital pin 3.
+ Connect the LCD D7 pin to digital pin 2.
+ Connect the LCD VSS (ground) pin to the Arduino's GND.
+ Connect the LCD VDD (power) pin to the Arduino's 5V.
+ Connect the LCD V0 (contrast) pin to a potentiometer (adjustable resistor) and connect the other end of the potentiometer to GND.


## STEPPER MOTOR


### source code

```
#include <AccelStepper.h>

// Define the motor connections
const int stepPin = 2;
const int dirPin = 3;

// Create an AccelStepper object
AccelStepper stepper(1, stepPin, dirPin);

void setup() {
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Set the initial position
  stepper.setCurrentPosition(0);
}

void loop() {
  // Move the stepper motor 200 steps in one direction
  stepper.moveTo(200);
  stepper.run();

  delay(1000);  // Pause for 1 second

  // Move the stepper motor 200 steps in the opposite direction
  stepper.moveTo(0);
  stepper.run();

  delay(1000);  // Pause for 1 second
}

```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/08021cb9-1594-460f-9dcd-ed595e7983ac)



### wiring

+ Connect the driver's step pin to digital pin 2 on the Arduino.
+ Connect the driver's dir pin to digital pin 3 on the Arduino.
+ Connect the stepper motor to the output terminals of the driver.
+ Connect a power supply to the stepper motor driver.


## DC MOTOR

### source code

```
// Motor A
const int motorAPin1 = 9;   // Input 1 for Motor A
const int motorAPin2 = 10;  // Input 2 for Motor A
const int enableAPin = 11;  // PWM Speed Control for Motor A

void setup() {
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(enableAPin, OUTPUT);

  // Set motor speed to zero initially
  analogWrite(enableAPin, 0);
}

void loop() {
  // Move the motor forward
  motorControl(motorAPin1, motorAPin2, enableAPin, 255);
  delay(2000);  // Run for 2 seconds

  // Stop the motor
  motorControl(motorAPin1, motorAPin2, enableAPin, 0);
  delay(1000);  // Pause for 1 second

  // Move the motor backward
  motorControl(motorAPin2, motorAPin1, enableAPin, 255);
  delay(2000);  // Run for 2 seconds

  // Stop the motor
  motorControl(motorAPin1, motorAPin2, enableAPin, 0);
  delay(1000);  // Pause for 1 second
}

// Function to control the motor speed and direction
void motorControl(int input1Pin, int input2Pin, int enablePin, int speed) {
  // Set motor direction based on the sign of the speed
  if (speed > 0) {
    digitalWrite(input1Pin, HIGH);
    digitalWrite(input2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, HIGH);
  } else {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, LOW);
  }

  // Set motor speed (PWM)
  analogWrite(enablePin, abs(speed));
}
```

### circuit diagram

![image](https://github.com/benedict04/Sensors_Interface/assets/109859485/dc03efa6-6299-40e0-96c9-32e227d39c9b)


### wiring

+ Connect the L298N Input 1 pin to digital pin 9 on the Arduino.
+ Connect the L298N Input 2 pin to digital pin 10 on the Arduino.
+ Connect the L298N Enable pin to digital pin 11 on the Arduino.
+ Connect the DC motor to the output terminals of Motor A on the L298N.
+ Connect a power supply to the motor driver (be sure to share the ground with the Arduino).











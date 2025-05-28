// ESP32-WROOM-32D Ultrasonic Sensor Code
// Converted from Arduino Uno

// Define Trig and Echo pin:
const int trigPin = 5;  // Changed to GPIO5
const int echoPin = 18; // Changed to GPIO18

// Define variables:
long duration;
int distance;

void setup() {
  // Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Begin Serial communication at a baudrate of 115200 (ESP32 preferred):
  Serial.begin(115200);
}

void loop() {
  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);
  
  delayMicroseconds(5);

  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance:
  distance = (duration/2) / 29.1;
  
  // Print the distance on the Serial Monitor:
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
  
  delay(500); // Short delay between readings
}
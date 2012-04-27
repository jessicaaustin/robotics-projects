
// motors
int M1A = 3;
int M1B = 4;
int M2A = 5;                         
int M2B = 6;    

// Parallax PING))) sensor
// 300cm max distance
int ultraSoundSignal = 7;
int val = 0;
int ultrasoundValue = 0;
int timecount = 0; // Echo counter
// multiply the ultrasound time by this value to get centimeters
float distanceMultiplier = 0.0877;

// misc
int loopDelay = 30;

// 255 is max speed, and 300cm is max distance
// multiply this by the obstacle distance to get speed
float speedMultiplier = 0.85;

void setup() { 
    pinMode(M1A, OUTPUT); 
    pinMode(M1B, OUTPUT);   
    pinMode(M2A, OUTPUT); 
    pinMode(M2B, OUTPUT); 
} 

void loop() {
  float distToObs = check_obstacles();
  if (distToObs > 10) {
    forward(distToObs*speedMultiplier);
  } else {
    clearObstacle();
  }
  delay(loopDelay);
}

void clearObstacle() {
  pause();
  for (int i=0; i<55; i++) {
    backward(200);
    delay(loopDelay);
  }
  pause();
  for (int i=0; i<55; i++) {
    left(200);
    delay(loopDelay);
  }
  pause();
}

void pause() {
  for (int i=0; i<30; i++) {
    stop();
    delay(loopDelay);
  }
}

void forward(int speed) {
    analogWrite(M1A,LOW);
    analogWrite(M1B,speed);    
    analogWrite(M2A,LOW);    
    analogWrite(M2B,speed);     
}

void backward(int speed) {
    analogWrite(M1A,speed);
    analogWrite(M1B,LOW);    
    analogWrite(M2A,speed);    
    analogWrite(M2B,LOW);     
}

void left(int speed) {
    analogWrite(M1A,speed);
    analogWrite(M1B,LOW);    
    analogWrite(M2A,LOW);    
    analogWrite(M2B,speed);     
}

void right(int speed) {
    analogWrite(M1A,LOW);
    analogWrite(M1B,speed);    
    analogWrite(M2A,speed);    
    analogWrite(M2B,LOW);     
}

void stop() {
    analogWrite(M1A,LOW);
    analogWrite(M1B,LOW);    
    analogWrite(M2A,LOW);    
    analogWrite(M2B,LOW);     
}

/**
 * Queries the ultrasound sensor and returns distance in cm of closest obstacle.
 */
float check_obstacles() {
   timecount = 0;
   val = 0;
   pinMode(ultraSoundSignal, OUTPUT); // Switch signalpin to output
  
  /* Send low-high-low pulse to activate the trigger pulse of the sensor
   * -------------------------------------------------------------------
   */

  digitalWrite(ultraSoundSignal, LOW); // Send low pulse
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(ultraSoundSignal, HIGH); // Send high pulse
  delayMicroseconds(5); // Wait for 5 microseconds
  digitalWrite(ultraSoundSignal, LOW); // Holdoff

  /* Listening for echo pulse
   * -------------------------------------------------------------------
   */

  pinMode(ultraSoundSignal, INPUT); // Switch signalpin to input
  val = digitalRead(ultraSoundSignal); // Append signal value to val
  while(val == LOW) { // Loop until pin reads a high value
    val = digitalRead(ultraSoundSignal);
  }
  
  while(val == HIGH) { // Loop until pin reads a high value
    val = digitalRead(ultraSoundSignal);
    timecount = timecount +1;            // Count echo pulse time
  }

  /* Return result
   * -------------------------------------------------------------------
   */

  ultrasoundValue = timecount; // Append echo pulse time to ultrasoundValue
  return ultrasoundValue*distanceMultiplier;

} 

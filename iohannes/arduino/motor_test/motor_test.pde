
/**
State  A  B
FWD    0  1
REV    1  0
STOP   0  0
short  1  1
circuit!
*/

// left track
int M1A = 3;
int M1B = 4;
// right track
int M2A = 5;                         
int M2B = 6;                           

void setup() 
{ 
    pinMode(M1A, OUTPUT); 
    pinMode(M1B, OUTPUT);   
    pinMode(M2A, OUTPUT); 
    pinMode(M2B, OUTPUT); 
} 

void loop() 
{ 
  int value;
  for(value = 0 ; value <= 255; value+=5) 
  { 
    forward(value);
    delay(30); 
  }  
  delay(2000); 
  
  for(value = 0 ; value <= 255; value+=5) 
  { 
    backward(value);
    delay(30); 
  }  
  delay(2000); 
  
  for(value = 0 ; value <= 255; value+=5) 
  { 
    left(value);
    delay(30); 
  } 
  delay(2000); 
  
  for(value = 0 ; value <= 255; value+=5) 
  { 
    right(value);
    delay(30); 
  } 
  delay(2000); 
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


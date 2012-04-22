
 
int M1A = 4;
int M1B = 5;
int M2A = 6;                         
int M2B = 7;                           
 
void setup() 
{ 
    pinMode(M1A, OUTPUT); 
    pinMode(M1B, OUTPUT);   
    pinMode(M2A, OUTPUT); 
    pinMode(M2B, OUTPUT); 
} 
 
void loop() {
  int value;
  for(value = 0 ; value <= 255; value+=5) 
  { 
    forward(value);
    delay(30); 
  }  
  delay(2000); 
}
 
void xloop() 
{ 
  int value;
  for(value = 0 ; value <= 255; value+=5) 
  { 
    forward(value);
    delay(1000); 
  }  
  delay(2000); 
  
  for(value = 0 ; value <= 255; value+=5) 
  { 
    backward(value);
    delay(1000); 
  }  
  delay(2000); 
  
  for(value = 0 ; value <= 255; value+=5) 
  { 
    left(value);
    delay(1000); 
  } 
  delay(2000); 
  
  for(value = 0 ; value <= 255; value+=5) 
  { 
    right(value);
    delay(1000); 
  } 
  delay(2000); 
}

void forward(int speed) {
    analogWrite(M1A,LOW);
    analogWrite(M1B,speed);    
    analogWrite(M2A,speed);    
    analogWrite(M2B,LOW);     
}

void backward(int speed) {
    analogWrite(M1A,speed);
    analogWrite(M1B,LOW);    
    analogWrite(M2A,LOW);    
    analogWrite(M2B,speed);     
}

void left(int speed) {
    analogWrite(M1A,speed);
    analogWrite(M1B,LOW);    
    analogWrite(M2A,speed);    
    analogWrite(M2B,LOW);     
}

void right(int speed) {
    analogWrite(M1A,LOW);
    analogWrite(M1B,speed);    
    analogWrite(M2A,LOW);    
    analogWrite(M2B,speed);     
}

void fullForward() {
    digitalWrite(M1A,LOW);
    digitalWrite(M1B,HIGH);    
    digitalWrite(M2A,HIGH);    
    digitalWrite(M2B,LOW);     
}

void fullBack() {
    digitalWrite(M1A,HIGH);
    digitalWrite(M1B,LOW);    
    digitalWrite(M2A,LOW);    
    digitalWrite(M2B,HIGH);     
}

void fullLeft() {
    digitalWrite(M1A,HIGH);
    digitalWrite(M1B,LOW);    
    digitalWrite(M2A,HIGH);    
    digitalWrite(M2B,LOW);     
}

void fullRight() {
    digitalWrite(M1A,LOW);
    digitalWrite(M1B,HIGH);    
    digitalWrite(M2A,LOW);    
    digitalWrite(M2B,HIGH);     
}


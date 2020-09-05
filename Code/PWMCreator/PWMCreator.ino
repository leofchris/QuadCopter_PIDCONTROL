

int ESC1 = 9;
int ESC2 = 10;
int maxPulse = 2285;
int minPulse = 681;
int newMaxPulse = 0;
int dutyCycle = 0;
String input;
char token[10];
int motor1 = 0;
int motor2 = 0;

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ESC1, OUTPUT);
  pinMode(ESC2, OUTPUT);
  

  
  Serial.println("Running ESC Calibration");

  while (Serial.available() <= 0)
  {
      analogWrite(ESC1, 150);
      analogWrite(ESC2, 150);
  }
  
  
  Serial.println("High Throttle Calibrated");
  serialFlush();
  
   while (Serial.available() <= 0)
  {
     
     analogWrite(ESC1, 90);
     analogWrite(ESC2, 90);
  }
  
  Serial.println("Low Throttle Calibrated");
  Serial.println("Done!");

  Serial.println("Type The Duty Cycle");
 
  
   
}
  
  
void loop() {
    serialFlush();
    Serial.println("weeeeeeeeeeeeeeeeeee i'm a helicopter!!");
    delay(1000);
    while(input == NULL){
      input = Serial.readString();
    }
     Serial.println(input);
    input.toCharArray(token,10);
   
    motor1 = atoi(strtok(token," "));
    motor2 = atoi(strtok(NULL,"\n"));
    Serial.println(motor1);
    Serial.println(motor2);
    
    analogWrite(ESC1, motor1);
    analogWrite(ESC2, motor2);
    input = "";

    
}
  
  


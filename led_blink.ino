String uav;
int LED=2;


void setup() {
  Serial.begin(9600);
  pinMode(LED,OUTPUT);

}

void loop() {
  if( Serial.available()>0){
    uav=Serial.readStringUntil('\n');
    if (uav=="on"){
      digitalWrite(LED,HIGH);
      delay(100);
      
      digitalWrite(LED,LOW);
      delay(100);
      Serial.write("led on off ..");
  }
    else {
      
      Serial.write("invalid input");
  }
}

}

#define LED 31

String x;
void setup() {
 pinMode(LED, OUTPUT);
  
 Serial.begin(115200);
 Serial.setTimeout(1);
}
void loop() {
 while (!Serial.available());
 x = Serial.readString();
 Serial.print(x);
 if(x == "Hi")
  {
    digitalWrite(LED,HIGH);
  }
  else
  {
    digitalWrite(LED,LOW);
  }
}

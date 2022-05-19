

String x;
void setup() {

 Serial.begin(115200);
 Serial.setTimeout(1);

}
void loop() {

 while (!Serial.available());
 x = Serial.readString();
 Serial.print(x);
 
}







//#define LED 31

//pinMode(LED, OUTPUT);

/*
 if(x == "Hi")
  {
    digitalWrite(LED,HIGH);
  }
  else
  {
    digitalWrite(LED,LOW);
  }
  */
#define LED_PIN 13

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  Serial.setTimeout(1);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    int x = Serial.readString().toInt();
    if (x == 0){
        digitalWrite(LED_PIN, LOW);
    }
    else{
        digitalWrite(LED_PIN, HIGH);

    }
  }
  delay(0.1);

}

void setup() {
  // This is an even divisor of F_CPU (16000000), so we get no error
  Serial.begin(500000);
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    int byte = Serial.read();
    if (byte == 'y') {
      digitalWrite(13, HIGH);
    } else if (byte == 'n') {
      digitalWrite(13, LOW);
    }
  }
}

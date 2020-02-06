#define MR_PIN A2

  int MR_min = 100;
  int MR_max = 100;
  int MR_read = 100;
  
void setup() {
  Serial.begin(230400);
}

void loop() {
  MR_read = analogRead(MR_PIN);

  MR_max = max(MR_read, MR_max);
  MR_min = min(MR_read, MR_min);

  Serial.print(MR_min);
  Serial.print(", ");
  Serial.println(MR_max);

}

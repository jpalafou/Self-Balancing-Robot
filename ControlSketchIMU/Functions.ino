void printData() {
  Serial.print(millis()/float(1000));
  Serial.print(", ");
  Serial.print(deltaT, 4);
  Serial.print(", ");
  Serial.print(theta_0);
  Serial.print(", ");
  Serial.print(theta);
  Serial.print(", ");
  Serial.print(theta_dot_0);
  Serial.print(", ");
  Serial.print(theta_dot);
  Serial.print(", ");
  Serial.println(phi);
}

float AverageArray(float Array[50]) {
  float avg = 0.0;
  for (int i = 0; i < hist; i = i + 1) {
    avg += Array[i];
  } 
  avg = avg/float(hist);
  return avg;
}

float CalculateDeltaTime(){
  float currentTime = millis();
  float deltaTime = currentTime - oldTime;
  oldTime = currentTime;
  return deltaTime;
}

float float_map(float x, float x1, float x2, float y1, float y2){
  float y;
  y = y1 + ( ( x - x1 ) * ( y2 - y1 ) / ( x2 - x1 ) );
  return y;
}

void printData() {
  Serial.print(theta);
  Serial.print(", ");
  Serial.println(theta_dot);
}

float AverageArray(float Array[50]) {
  float avg = 0.0;
  for (int i = 0; i < hist; i = i + 1) {
    avg += Array[i];
  } 
  avg = avg/float(hist);
  return avg;
}

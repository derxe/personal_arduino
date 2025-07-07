void setup() {
  Serial.begin(115200);
}

void printBinary3(int value) {
  for (int i = 2; i >= 0; i--) {
    Serial.print((value >> i) & 1);
  }
}

void loop() {
  int bottom = analogRead(A2);
  int middle = analogRead(A4);
  int top = analogRead(A3);

  int bottom_state = bottom > 100;
  int middle_state = 0; // miyddle > 100;
  int top_state = 1 - (top > 100);

  int direction_code = (bottom_state << 2) | (top_state << 1) | middle_state;

  Serial.print("direction:");
  printBinary3(direction_code);
  Serial.print(" ");
  Serial.print(direction_code, BIN);
  Serial.print(" direction_name:");
  switch (direction_code) {
  case 0b000: Serial.print("North"); break;
  case 0b001: Serial.print("North-East"); break;
  case 0b010: Serial.print("East"); break;
  case 0b011: Serial.print("South-East"); break;
  case 0b100: Serial.print("South"); break;
  case 0b101: Serial.print("South-West"); break;
  case 0b110: Serial.print("West"); break;
  case 0b111: Serial.print("North-West"); break;
  }

// 0 5 7 5 1 3 1 4 6 4 0 2 0 

/*
0  → 0b000  
5  → 0b101  
7  → 0b111  
5  → 0b101  
1  → 0b001  
3  → 0b011  
1  → 0b001  
4  → 0b100  
6  → 0b110  
4  → 0b100  
0  → 0b000  
2  → 0b010  
0  → 0b000


*/


  Serial.print(" bottom:"); Serial.print(bottom); Serial.print(" ");
  Serial.print("   top:"); Serial.print(top); Serial.print(" ");
  Serial.print("middle:"); Serial.print(middle); Serial.print("");
  Serial.println();
  delay(200);
}

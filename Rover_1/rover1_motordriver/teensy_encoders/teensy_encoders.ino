/* Encoder reader - based on:
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 */

#include <Encoder.h>

//Both pins should have interrupt capability. All teensy 3.1 pins can have interrupts
Encoder enc1(20, 21);
Encoder enc2(18, 19);
Encoder enc3(16, 17);
Encoder enc4(14, 15);

void setup() {
  Serial.begin(115200);
}

long positions[]  = {0,0,0,0};
long last_positions[] = {0,0,0,0};

const char delimiter = ':';

void loop() {
  positions[0] = enc1.read();
  positions[1] = enc2.read();
  positions[2] = enc3.read();
  positions[3] = enc4.read();
  
  long deltas[4];
  for (int i = 0; i < 4; i++){
    
  deltas[i] = positions[i]-last_positions[i];
  last_positions[i] = positions[i];   
 
   
  }
  // 0 -> venstre bak
  // 1 -> venstre foran
  // 2 -> hoyre foran
  // 3 -> hoyre bak
  
  Serial.print(deltas[0]);
  Serial.print(delimiter);
  Serial.print(deltas[1]);
  Serial.print(delimiter);
  Serial.print(deltas[2]);
  Serial.print(delimiter);
  Serial.print(deltas[3]);
  Serial.print(delimiter);
  
  
  Serial.print(positions[0]);
  Serial.print(delimiter);
  Serial.print(positions[1]);
  Serial.print(delimiter);
  Serial.print(positions[2]);
  Serial.print(delimiter);
  Serial.print(positions[3]);
  
  Serial.println("");
  
  //Delay 100ms, loop runs at 10hz
  delay(100);
}

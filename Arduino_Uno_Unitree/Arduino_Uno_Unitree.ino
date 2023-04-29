#include <ArduinoBLE.h>

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial.begin(19200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(1000);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);


  digitalWrite(13, HIGH);
  delay(5000);
  digitalWrite(13, LOW);
  
  BLE.begin();
  Serial.println("Central device robot is on.");

  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

}

void loop() {
  BLEDevice peripheral = BLE.available();
  // print the string when a newline arrives:
    if (peripheral) {

    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    // stop scanning
    BLE.stopScan();
    serialEvent(peripheral);
    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

  
  if (stringComplete) {
    Serial.println(inputString);
    // clear the string:
    inputString = "";
    digitalWrite(12, HIGH);
    delay(1000);
    digitalWrite(12, LOW);
    stringComplete = false;
  }
  
  delay(500);
}
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent(BLEDevice peripheral) {    
  while (Serial.available()) {
    // get the new byte:
    
  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }
    BLECharacteristic SensorsCharacteristic1 = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");
 while(peripheral.connected()) {
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
       SensorsCharacteristic1.writeValue(inChar);
    }
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(12, LOW);
  }
}
}
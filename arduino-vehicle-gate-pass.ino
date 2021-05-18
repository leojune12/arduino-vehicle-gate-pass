#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

Servo servoIn;
Servo servoOut;

#define RST_PIN         9          // Configurable, see typical pin layout above
#define SS_1_PIN        10         // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 2
#define SS_2_PIN        8          // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 1

#define NR_OF_READERS   3          // Only last 2 are used

int IR1 = 3;                       // Entrance IR pin
int IR2 = 4;                       // Exit IR pin

byte ssPins[] = {14, SS_1_PIN, SS_2_PIN};   // Ignore first item

MFRC522 mfrc522[NR_OF_READERS];   // Create MFRC522 instance.

bool gate_entrance = false;
bool gate_exit = false;

int servoSpeed = 5;

/**
 * Initialize.
 */
void setup() {

  servoIn.attach(5);
  servoOut.attach(6);

  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW); // Reset
  delay(500);

  Serial.begin(115200); // Initialize serial communications with the PC
  while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

  SPI.begin();        // Init SPI bus

  for (uint8_t reader = 1; reader < NR_OF_READERS; reader++) {  // Ignore first item
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN); // Init each MFRC522 card
  }

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
}

void loop() {
  readRFID();

  readSerial();

  readIR();
}

void readRFID() {
  for (uint8_t reader = 1; reader < NR_OF_READERS; reader++) {  // Ignore first item

    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) {
      Serial.print("{ 'reader': '");
      Serial.print(reader);
      // Show some details of the PICC (that is: the tag/card)
      Serial.print("', 'uid': '");
      dump_byte_array(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.print("' }");
      Serial.println();

      // Halt PICC
      mfrc522[reader].PICC_HaltA();
      // Stop encryption on PCD
      mfrc522[reader].PCD_StopCrypto1();

      
    }
  }
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void readSerial() {
  if (Serial.available()) {
    int inByte = Serial.parseInt();
    
    if (inByte == 1) {
      gate_entrance = true;

      for (int i=90; i>0; i--) {
        servoIn.write(i);
        delay(servoSpeed);
      }
    } else if (inByte == 2){
      gate_exit = true;

      for (int i=90; i>0; i--) {
        servoOut.write(i);
        delay(servoSpeed);
      }
    }
  }  
}

void readIR() {
  if (gate_entrance) {
    int reading = digitalRead(IR1);
    if (reading == 1) {
      gate_entrance = false;

      delay(2000);

      for (int i=0; i<90; i++) {
        servoIn.write(i);
        delay(servoSpeed);
      }
    }
  }

  if (gate_exit) {
    int reading = digitalRead(IR2);
    if (reading == 1) {
      gate_exit = false;

      delay(2000);

      for (int i=0; i<90; i++) {
        servoOut.write(i);
        delay(servoSpeed);   
      }
    }
  }
}

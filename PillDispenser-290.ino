//include libraries for hardware
#include <MFRC522.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_Fingerprint.h> 

//initializing pins
#define SDAPIN 10
#define RESETPIN 8
#define LED_RED 4
#define LED_GREEN 3

#define LOCK_RELAY 7  //the Arduino pin that connects to the IN pin of relay

byte TagSerialNumber[4];

int prescription_array[3] = {10, 6, 8};

//initialize array of registered RFID tags. Index 1 holds user 1's RFID tag and son on
byte RFID_array[3][4] = {{0x00, 0x00, 0x00, 0x00}, {0x6D, 0x1B, 0x8A, 0x3F}, {0x53, 0x98, 0x2E, 0xDA}};

byte GoodTagSerialNumber1[4] = {0x6D, 0x1B, 0x8A, 0x3F}; // First accepted tag serial number
byte GoodTagSerialNumber2[4] = {0x53, 0x98, 0x2E, 0xDA}; // Second accepted tag serial number

//create servo object
Servo myservo;

//initialize varibales used to track user id
uint8_t id;

//initialize true and false variables to their starting conditions
bool fingerDetect = false;
bool isGoodTag = false;
bool locked = true;

int pos = 0;    //variable to store the servo position
int direction = 0; //variable to change servo direction

#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
//For UNO and others without hardware serial, we must use software serial
//pin #5 is IN from sensor (GREEN wire)
//pin #6 is OUT from arduino  (WHITE wire)

//Set up the serial port to use softwareserial
SoftwareSerial mySerial(5, 6);

#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is green wire, #1 is white
#define mySerial Serial1

#endif

//not sure if needed
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

MFRC522 mfrc522(SDAPIN, RESETPIN);

void setup() {

//setup for arduino and LED's
  SPI.begin();
  Serial.begin(9600);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

//use pin 9 for servo
  myservo.attach(9);

  pinMode(LOCK_RELAY, OUTPUT); //initialize digital pin 7 for lock

  digitalWrite(LED_GREEN, LOW); //Ensure LED's are off initially
  digitalWrite(LED_RED, LOW);

  digitalWrite(LOCK_RELAY, HIGH); //initialize lock as unlocked
  delay(100);

//ensuring RFID detector is ready for use
  Serial.println("Looking for RFID Reader");
  mfrc522.PCD_Init();

  byte version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
  if (!version) {
    Serial.println("Didn't find RC522 board.");
    while (1); // Wait until an RFID Module is found
  }

  Serial.print("Found chip RC522 ");
  Serial.print("Firmware version: 0x");
  Serial.println(version, HEX);
  Serial.println();


//output fingerprint sensor information and ensure it is ready for use
  while (!Serial);
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

//check to see if any user data is stored in the fingerprint sensor
  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
      Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
}

////////////////////// MAIN CODE /////////////////////////////

void loop() {

//search for a fingerprint until a valid fingerprint is found
  while (fingerDetect == false){
    id = -1;
    id = getFingerprintID();
    if (id >= 0){
      Serial.print("Id ");
      Serial.print(id);
      Serial.println(" found");
      fingerDetect = true;
    }
    delay(500); //small delay between fingerprint scans
  }

  while(!isGoodTag){
    //Look for RFID tag
    if (!mfrc522.PICC_IsNewCardPresent()) {
      return;
    }

    //Select tag being scanned
    if (!mfrc522.PICC_ReadCardSerial()) {
      return;
    }

    //Copy the UID to TagSerialNumber
    memcpy(TagSerialNumber, mfrc522.uid.uidByte, mfrc522.uid.size);

    //Print the detected serial number for verification
    Serial.print("Detected Tag Serial Number: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(TagSerialNumber[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Check if the detected tag matches the tag registered to the user detecetd through fingerprint id
    if (memcmp(RFID_array[id], TagSerialNumber, mfrc522.uid.size) == 0) {
      isGoodTag = true;
    }
  }

  Serial.print("User identity verified!");

//if user has been verified the dor can be unlocked
if (locked == true){
  digitalWrite(LED_GREEN, HIGH); //turn on green lED
  digitalWrite(LOCK_RELAY, UNLOCK); //unlock door
  delay(5000);
  digitalWrite(LOCK_RELAY, LOCK); //lock door
  digitalWrite(LED_GREEN, LOW); //turn off green LED
  locked = false;
}

//call dispense function with using detected users prescription
  dispense(prescription_array[id]);

  digitalWrite(LED_GREEN, HIGH); //turn on green LED
  digitalWrite(LOCK_RELAY, LOW); //unlock door
  delay(5000);
  digitalWrite(LOCK_RELAY, HIGH); //lock door
  digitalWrite(LED_GREEN, LOW); //turn off green LED

//reset all boolean variables to return to initial state
  fingerDetect = false;
  isGoodTag = false;
  locked = true;
  
  delay(500);
  mfrc522.PICC_HaltA(); // Halt the current card
  
}

//dispesne function used to dispense specified number of pills
void dispense(int pill){
  for (int i=0; i < pill; i++) {
    if (direction == 0) {
      for (int pos = 0; pos < 180; pos++) { //Rotate servo 180˚
        myservo.write(pos);
        delay(20);
      }
      direction = 1; //Switch direction when done
    } else {
      for (int pos = 180; pos > 0; pos--) { //Rotate servo 180˚ in the other direction
        myservo.write(pos);
        delay(20);
      }
      direction = 0; //Switch direction when done
    }
  }
}

//////////////////// Fingerprint ///////////////////////////

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage(); //prompt sensor to scan for finger
  switch (p) {
    case FINGERPRINT_OK: //finger succesfully scanned
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER: //no finger detected
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR: //wiring issue
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL: //cannot identify fingerprint features
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz(); //convert image to scan template
  switch (p) {
    case FINGERPRINT_OK: //succesfully converted image
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS: //image too messy to convert
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR: //wiring error
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL: //cannot idnetify finerprint features to convert
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE: //cannot identify fingerprint features to convert
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch(); //compare newly created template to stored templates
  if (p == FINGERPRINT_OK) { //if match is found
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) { //wiring error
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) { //no match found
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  //found a match print id and confidence
  Serial.print("Found ID #"); 
  Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); 
  Serial.println(finger.confidence);

  return finger.fingerID;
}
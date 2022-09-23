//Anemoi ICD Fehler/Fragen
//Protokoll: besser Stop Byte nach Checksum (ist ja auch newline)
//hin und wieder ein Byte zu wenig im AHRS sentence (wahrscheinlich CircleDiameter LSB, evtl. wenn LSB == 0)
//Warum wird Heading 3-fach übertragen (Wind, AHRS, Data)?
//Sensor Health sentence besser umsortieren: Sensor Health, Orientation, V_Main (je ein Byte)
//Sensor Health sentence nur mit 'S' (table 2-1)
//Menu sentence nur mit 'M' (table 2-1)
//V_Main in Sensor Health sentence 3bit und 3bit (2.1.3)
//Table 2-7: Data sentence (heißt aktuell table 2-6: AHRS sentence)
//  Schreibfehler B9 pitot calibration
//ab Table 2-7 weitere Nummerierung der Tables falsch
//was fehlt: Lastvielfaches (evtl. statt Kreisdurchmesser), vCAS (evtl. statt pitot factor)
//kein RX am Anemoi?

#define DISABLE_ALL_LIBRARY_WARNINGS

#include "Arduino.h"
//#include <String.h>
#include <StreamString.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <EasyButton.h>
#include "TFT_eSPI.h"
#include "T4_V13.h"
#include "MPU9250.h"

#include "imagedata.h"

void drawHorizon(int roll, int pitch);
void drawCrosshair(int roll);
void numberBox(int x, int y, float num);
void print_calibration();

MPU9250 mpu;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite AhrsSprite = TFT_eSprite(&tft);
TFT_eSprite WbkSprite = TFT_eSprite(&tft);
TFT_eSprite InfoSprite = TFT_eSprite(&tft);

#define AHRS_Height 160 //172
#define AHRS_Width 160
#define AHRS_SkyColor TFT_SKYBLUE
#define AHRS_GroundColor TFT_DARKGREEN
#define CrosshairSize 4

bool tempAvail = false;
bool sdAvail = false;
bool wbkChanged = true;
const char fileName[] = "/putty.log";
Adafruit_BMP280 bme;
EasyButton Button(38); //war 39
TinyGPSPlus gps;
TinyGPSCustom lxwp0Speed(gps, "LXWP0", 2);
TinyGPSCustom lxwp0Alt(gps, "LXWP0", 3);
TinyGPSCustom lxwp0Heading(gps, "LXWP0", 10);
TinyGPSCustom lxwp0WindDirection(gps, "LXWP0", 11);
TinyGPSCustom lxwp0WindSpeed(gps, "LXWP0", 12);
unsigned long screen_update_ms;
unsigned long logging_ms;
unsigned long wbk_sens_ms;
unsigned long temp_sens_ms;
StreamString tempString;
// StreamString humidString;
StreamString pressString;
StreamString valueString;
char timeString[7]="";
char dateString[7]="";
float SensorValue;
float baro;
double gpsAlt;
double airSpeed;
int wbkValue = 0; //S1:1   S:2   -2:3   -1:4   0:5   +1:6   +2:7   L:8
unsigned int mode = 0; // 0: standard mode (only WBK display), 1: info mode, 2: setup mode

// Anemoi output
uint8_t Anemoi_Orientation = 0;
uint8_t Anemoi_SensorHealth = 0;
uint8_t Anemoi_SensorHealthIndication = 0;
float Anemoi_Version = 0;
uint16_t Anemoi_WindDirection = 0;
uint8_t Anemoi_WindMagnitude = 0;
uint16_t Anemoi_WindDirectionAvg = 0;
uint8_t Anemoi_WindMagnitudeAvg = 0;
uint16_t Anemoi_Heading = 0;
uint16_t Anemoi_AhrsRollAngle = 0;
int8_t Anemoi_AhrsPitchAngle = 0;
uint16_t Anemoi_AhrsHeading = 0;
uint16_t Anemoi_AhrsCircleDiameter = 0;
uint16_t Anemoi_VGnd = 0;
uint16_t Anemoi_VTas = 0;
uint16_t Anemoi_Track = 0;
uint16_t Anemoi_DataHeading = 0;
int8_t Anemoi_Temperature = 0;
uint8_t Anemoi_PitotFactor = 0;
uint16_t Anemoi_FlightLevel = 0;
uint8_t Anemoi_MenuPage = 0;
uint8_t Anemoi_MenuMarker = 0;
uint8_t Anemoi_MenuClock = 0;
uint8_t Anemoi_MenuNumber = 0;
// Anemoi statistics
uint32_t Anemoi_encodedCharCount = 0;
uint32_t Anemoi_failedChecksumCount = 0;
uint32_t Anemoi_passedChecksumCount = 0;
// Anemoi parsing state variables
//uint8_t Anemoi_parity;
//bool Anemoi_isChecksumTerm;
char Anemoi_term[16];
//uint8_t Anemoi_curSentenceType;
//uint8_t Anemoi_curTermNumber;
uint8_t Anemoi_curTermOffset;

float GetSmoothSensorValue() {
  int const numReadings = 101;
  int values[numReadings];
  unsigned long Sum = 0;
  for (int i = 0; i < numReadings; i++) {
    int SensorVal = analogRead(27);
    values[i] = SensorVal;
    Sum += values[i];
//    delay(1);
//    Serial.print(SensorVal);
//    Serial.print("\t");
  }
  
//Bubble sort
  // for(int out=0 ; out < numReadings; out++) {  // outer loop
  // bool swapped = false;
  //   for(int in=0; in < (numReadings-1); in++) {  // inner loop
  //     if( values[in] > values[in+1] ) {   // out of order?
  //       int swapper = values[in];
  //       values [in] = values[in+1];
  //       values[in+1] = swapper;
  //       swapped = true;
  //     }
  //   }
  //   if (swapped == false) 
  //     break; 
  // }
//  unsigned long sSum = 0;
//  for (int i = 10; i < 40; i++) {
//    sSum += values[i];
//  }
//  Serial.printf("Max-Min %i, Mean %.1f, Med %i, Quant %.1f\n", values[50]-values[0], Sum/50.0, values[numReadings / 2], sSum/30.0);
  
  return Sum/numReadings;//values[numReadings / 2];   //sSum/30.0;  //
}

void buttonISR()
{
  Button.read();
  //Serial.println("Button action.");
}
void buttonPressed()
{
  Serial.println("Button pressed");
  switch (mode){
    case 0:
      mode = 1;
      //mode1_ms = millis();
      break;
    case 1:
    case 3:
      mode = 0;
      //epaper_int = 0;
      //wbkChanged = true;
      break;
    case 2:
      // if (mode2count>7) mode2count = 1;
      // else mode2count += 1;
      //TODO: save momentary Cts value to intermed storage
      break;
    default:
      break;
  }
}
void buttonPressedThreeSeconds()
{
  Serial.println("Button pressed for three seconds");
  switch (mode){
    case 0:
      mode = 2;
      //mode2_ms = millis();
      break;
    case 1:
      mode = 2;
      //epaper_int = 0;
      break;
    case 2:
      mode = 0;
      //epaper_int = 0;
      //TODO: save wbk values to final storage
      break;
    default:
      break;
  }
}
void buttonThreeTimes()
{
  Serial.println("Button pressed three times");
  mode = 3;
}

static const uint8_t CRC_TABLE[256] = {
 0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
 0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
 0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
 0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
 0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
 0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
 0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
 0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};
uint8_t crc8ccitt(const void * data, size_t size) {
 uint8_t val = CRC_TABLE[val ^ '$'];//0;
 uint8_t * pos = (uint8_t *) data;
 uint8_t * end = pos + size;
 while (pos < end) {
 val = CRC_TABLE[val ^ *pos];
 pos++;
 }
 return val;
}
bool Anemoi_endOfSentenceHandler()
{
  // If it's the checksum term, and the checksum checks out, commit
    uint8_t Anemoi_parity = Anemoi_term[Anemoi_curTermOffset-1];
    byte checksum = crc8ccitt(Anemoi_term, Anemoi_curTermOffset-1);

    if (checksum == Anemoi_parity)
    {
      Anemoi_passedChecksumCount++;
      //Serial.println("passed: " + String(Anemoi_term));

      switch(Anemoi_term[0])
      {
        case 'S': // Sensor Health sentence
        //case 's': //wohl nicht genutzt (Fehler im ICD)
          Serial.println("Sensor Anemoi_curTermOffset: " + String(Anemoi_curTermOffset));
          if (Anemoi_curTermOffset == 5)
          {
            Anemoi_Orientation = (Anemoi_term[1]>>6) & 0x03;
            Anemoi_SensorHealth = Anemoi_term[1] & 0xFC;
            Anemoi_SensorHealthIndication = (Anemoi_term[2]>>7) & 0x01;
            Anemoi_Version = ((Anemoi_term[2]>>4) & 0x03) + ((float)(Anemoi_term[2] & 0x0F))/10; //wohl ein Fehler im ICD
            Serial.println("Orentation: " + String(Anemoi_Orientation));
          }
          break;
        case 'W': // Wind sentence
        case 'w':
          if (Anemoi_curTermOffset == 11)
          {
            Anemoi_WindDirection = (uint16_t)Anemoi_term[1]*256 + Anemoi_term[2];
            Anemoi_WindMagnitude = (uint8_t)Anemoi_term[3];
            Anemoi_WindDirectionAvg = (uint16_t)Anemoi_term[4]*256 + Anemoi_term[5];
            Anemoi_WindMagnitudeAvg = (uint8_t)Anemoi_term[6];
            Anemoi_Heading = (uint16_t)Anemoi_term[7]*256 + Anemoi_term[8];
            //Serial.println("Wind: " + String(Anemoi_WindDirection));
          }
          break;
        case 'A': // AHRS sentence
        case 'a':
          //Serial.println("AHRS Anemoi_curTermOffset: " + String(Anemoi_curTermOffset));
          if (Anemoi_curTermOffset == 10)
          {
            //if (Anemoi_term[1] != 0xFF)
            {
              Anemoi_AhrsRollAngle = (uint16_t)Anemoi_term[1]*256 + Anemoi_term[2];
              Anemoi_AhrsPitchAngle = (int8_t)Anemoi_term[3];
              Anemoi_AhrsHeading = (uint16_t)Anemoi_term[4]*256 + Anemoi_term[5];
              Anemoi_AhrsCircleDiameter = (uint16_t)Anemoi_term[6]*256 + Anemoi_term[7];
              //Serial.println("AhrsTerm: " + String(Anemoi_term) + ", a "  + String((uint16_t)Anemoi_term[1],16) + " "  + String((uint16_t)Anemoi_term[2],16) + "    "  + String((uint16_t)Anemoi_term[8],16) + " "  + String((uint16_t)Anemoi_term[9],16) + " "  + String((uint16_t)Anemoi_term[10],16));
              Serial.print("Anemoi_AhrsRollAngle: " + String(Anemoi_AhrsRollAngle));
              Serial.println(", Anemoi_AhrsPitchAngle: " + String(Anemoi_AhrsPitchAngle));
              // Lastvielfaches = sqrt(1 + V_Cas^4 / (9.81^2 * R^2))
            }
          }
          break;
        case 'D': // Data sentence
        case 'd':
          //Serial.println("Data Anemoi_curTermOffset: " + String(Anemoi_curTermOffset));
          if (Anemoi_curTermOffset == 15)
          {
            Anemoi_VGnd = (uint16_t)Anemoi_term[1]*256 + Anemoi_term[2];
            Anemoi_VTas = (uint16_t)Anemoi_term[3]*256 + Anemoi_term[4];
            Anemoi_Track = (uint16_t)Anemoi_term[5]*256 + Anemoi_term[6];
            Anemoi_DataHeading = (uint16_t)Anemoi_term[7]*256 + Anemoi_term[8];
            Anemoi_Temperature = (int8_t)Anemoi_term[9];
            Anemoi_PitotFactor = (uint8_t)Anemoi_term[10];
            Anemoi_FlightLevel = (uint16_t)Anemoi_term[11]*256 + Anemoi_term[12];
            //Serial.println("Vgnd: " + String(Anemoi_VGnd));
          }
          break;
        case 'M': // Menu sentence
        //case 'm':  //wohl nicht genutzt (Fehler im ICD)
          Serial.println("Menu Anemoi_curTermOffset: " + String(Anemoi_curTermOffset));
          if (Anemoi_curTermOffset == 7)
          {
            Anemoi_MenuPage = (uint8_t)Anemoi_term[1];
            Anemoi_MenuMarker = (uint8_t)Anemoi_term[2];
            Anemoi_MenuClock = (uint8_t)Anemoi_term[3];
            Anemoi_MenuNumber = (uint8_t)Anemoi_term[4];
            Serial.println("MenuPage: " + String(Anemoi_MenuPage));
          }
          break;
        default:
          break;
      }

    }
    else
    {
      ++Anemoi_failedChecksumCount;
      Serial.println("failed: " + String(Anemoi_term) + ", Anemoi_curTermOffset: " + String(Anemoi_curTermOffset));
    }

  // the first term determines the sentence type

    
  return false;
}
bool anemoi_encode(char c)
{
  ++Anemoi_encodedCharCount;
  bool isValidSentence = false;;

  switch(c)
  {
  //case '\r':
  //case '\n':
  //case '*':
    // bool isValidSentence = false;
    // if (Anemoi_curTermOffset < sizeof(Anemoi_term))
    // {
    //   Anemoi_term[Anemoi_curTermOffset] = 0;
    //   isValidSentence = Anemoi_endOfSentenceHandler();
    // }
    // //++Anemoi_curTermNumber;
    // Anemoi_curTermOffset = 0;
    // //Anemoi_isChecksumTerm = c == '\n';
    // return isValidSentence;
  //case ' ':
    //break;
  case '$': // sentence begin
    if (Anemoi_curTermOffset < sizeof(Anemoi_term))
    {
      Anemoi_term[Anemoi_curTermOffset] = 0;
      isValidSentence = Anemoi_endOfSentenceHandler();
    }
    Anemoi_curTermOffset = 0;
    return isValidSentence;
    //Anemoi_curTermNumber = Anemoi_curTermOffset = 0;
    //Anemoi_parity = 0;
    //Anemoi_curSentenceType = GPS_SENTENCE_OTHER;
    //Anemoi_isChecksumTerm = false;
    //return false;

  default: // ordinary characters
    if (Anemoi_curTermOffset < sizeof(Anemoi_term) - 1)
      Anemoi_term[Anemoi_curTermOffset++] = c;
    //if (!Anemoi_isChecksumTerm)
      //Anemoi_parity ^= c;
    return false;
  }

  return false;
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8N1, 33, 32); //connection to Butterfly Vario //war Serial2 auch im Loop
  //Serial2.begin(19200, SERIAL_8N1, 29, 32); //connection to Anemoi

  Serial.print("Initializing BME280 sensor...");
  //Wire.begin(-1,-1,(uint32_t)50000); //default 100000
  Wire.begin();
  if (!bme.begin()) Serial.println(" failed!");
  else {
    tempAvail = true;
    Serial.println(" OK!");
  }
  
  Serial.print("Initializing SD card...");
  SPIClass* hspi = new SPIClass(HSPI);
  hspi->begin(14,2,15,13);
  //hspi->begin(18,2,23,13);
  if (!SD.begin(13,*hspi))
  // SPIClass* hspi = new SPIClass(HSPI);
   // if (!SD.begin(13,*hspi))
  // SPIClass SDSPI(HSPI);
  // SDSPI.begin(14, 2, 15);
  // if (!SD.begin(13,SDSPI))
    Serial.println(" failed!");
  else {
    File dataFile;
    // if (SD.exists(fileName)) dataFile = SD.open(fileName, FILE_APPEND);
    // else dataFile = SD.open(fileName, FILE_WRITE);
    // if (dataFile) {
    //   dataFile.println("Init");
    //   dataFile.close();
    //   sdAvail = true;
    //   Serial.println(" OK!");
    // }
    // else Serial.println("error opening " + String(fileName));
    if (SD.exists(fileName)) dataFile = SD.open(fileName, FILE_READ);
    while (dataFile.available())
      anemoi_encode(dataFile.read());

    //Serial.println("Anemoi_encodedCharCount " + String(Anemoi_encodedCharCount));
    Serial.println("Anemoi_failedChecksumCount " + String(Anemoi_failedChecksumCount));
    Serial.println("Anemoi_passedChecksumCount " + String(Anemoi_passedChecksumCount));

  }
  

  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  Serial.print("Initializing IMU...");
  if (!mpu.setup(0x68, setting))
    Serial.println(" failed!");
  else
    Serial.println(" OK!");
    
  // calibrate anytime you want to
  // Serial.println("Accel Gyro calibration will start in 5sec.");
  // Serial.println("Please leave the device still on the flat plane.");
  // mpu.verbose(true);
  // delay(5000);
  // mpu.calibrateAccelGyro();

  // Serial.println("Mag calibration will start in 5sec.");
  // Serial.println("Please Wave device in a figure eight until done.");
  // delay(5000);
  // mpu.calibrateMag();

  // print_calibration();
  // mpu.verbose(false);


  tft.init();
  tft.setRotation(3);
  tft.setPivot(64, 79);

  AhrsSprite.createSprite(AHRS_Width, AHRS_Height);
  AhrsSprite.setColorDepth(8);

  WbkSprite.createSprite(128, 158);
  WbkSprite.setColorDepth(1);
  
  InfoSprite.createSprite(150, 55);
  InfoSprite.setColorDepth(1);
  InfoSprite.setFreeFont(&FreeMonoBold18pt7b);
  InfoSprite.setTextColor(TFT_WHITE);

  Serial.print("Initializing Button handler...");
  Button.begin();
  Button.onPressed(buttonPressed);
  Button.onSequence(3, 1000, buttonThreeTimes);
  Button.onPressedFor(3000, buttonPressedThreeSeconds);
  if (Button.supportsInterrupt())
  {
    Button.enableInterrupt(buttonISR);
    Serial.print(" Button will be used through interrupts,");
  }
  Serial.println(" done!");
  
  Serial.println("setup done");
  logging_ms = millis();
}

void loop() {
  tft.fillScreen(TFT_BLACK);

  WbkSprite.drawBitmap(0, 0, M2_DATA, 128, 158, TFT_BLACK, TFT_WHITE);
  WbkSprite.pushRotated(90);
  WbkSprite.deleteSprite();

  int k = 0;
  int l = 0;
  int framecount = 0;

  while (1)
  {
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
        int roll = mpu.getRoll();
        int pitch = mpu.getPitch();
        //Serial.println("Roll: " + String(roll) + ", Pitch: " + String(pitch));
        AhrsSprite.fillSprite(AHRS_SkyColor);
        //drawHorizon(50*sin(1.*k/180*PI), 15*sin(1.3*l/180*PI+1));
        drawHorizon(roll, pitch);
        drawCrosshair(CrosshairSize);
        AhrsSprite.pushSprite(160, 0);
        prev_ms = millis();
      }
    }
    
    framecount++;
    
    // k = k+1;
    // l = l+1;
    // if (k>=360) k=0;
    // if (1.3*l>=360) l=0;

    Button.update();

    while (Serial1.available()>0) {
      gps.encode(Serial1.read());
    }
    while (Serial2.available()>0) {
      //Serial.print(Serial2.read());
      anemoi_encode(Serial2.read());
    }

    unsigned long current_millis = millis();

    if (current_millis - wbk_sens_ms> 200)
    {
      wbk_sens_ms = current_millis;
      SensorValue = GetSmoothSensorValue();
      valueString.clear();
      valueString.print(SensorValue, 0);
      valueString += " Cts";
      //Serial.println(valueString);

      //History:
      // alt: L 2940, +2 1810, +1 1152, 0 762, -1 622, -2 553-575, S 500, S1 455
      if (SensorValue<510) { //S1:1   S:2   -2:3   -1:4   0:5   +1:6   +2:7   L:8
        if (wbkValue!=1) wbkChanged=true;
        wbkValue = 1;
      }
      else if (SensorValue<575) {
        if (wbkValue!=2) wbkChanged=true;
        wbkValue = 2;
      }
      else if (SensorValue<625) {
        if (wbkValue!=3) wbkChanged=true;
        wbkValue = 3;
      }
      else if (SensorValue<700) {
        if (wbkValue!=4) wbkChanged=true;
        wbkValue = 4;
      }
      else if (SensorValue<910) {
        if (wbkValue!=5) wbkChanged=true;
        wbkValue = 5;
      }
      else if (SensorValue<1380) {
        if (wbkValue!=6) wbkChanged=true;
        wbkValue = 6;
      }
      else if (SensorValue<2300) {
        if (wbkValue!=7) wbkChanged=true;
        wbkValue = 7;
      }
      else {
        if (wbkValue!=8) wbkChanged=true;
        wbkValue = 8;
      }
    }

    if (current_millis - screen_update_ms > 500)
    {
      InfoSprite.fillSprite(TFT_BLACK);
      InfoSprite.drawString(String(framecount * 2) + "fps", 0,0);
      InfoSprite.pushSprite(0, 185);
      //Serial.println("Framerate " + String (framecount * 2));
      framecount = 0;
      screen_update_ms = current_millis;
      // // Show time in milliseconds to draw and then push 1 sprite to TFT screen
      // numberBox( 10, 10, (millis()-dt)/500.0 );
    }

    if (tempAvail) {
      if (current_millis - temp_sens_ms> 2000)
      {
        temp_sens_ms = current_millis;
        tempString.clear();
        // humidString.clear();
        pressString.clear();
        tempString.print(bme.readTemperature(),2);
        tempString += " *C";    
        // humidString.print(bme.readHumidity(),2);
        // humidString += " %rH";  
        baro = bme.readPressure()/100;
        pressString.print(baro,1);
        pressString += " hPa";    
      }
    }


  }
}

void drawHorizon(int roll, int pitch)
{
  if (roll>180)
    roll = roll-360;
  // while (roll<-180)
  //   roll += 180;

  int a = tan(-roll/180.0*PI)*AHRS_Width/2;
  int pitch_offset = -pitch*4;

  if (pitch_offset < -AHRS_Height/2 + 5)
    pitch_offset = -AHRS_Height/2 + 5;
  if (pitch_offset > AHRS_Height/2 - 5)
    pitch_offset = AHRS_Height/2 - 5;

  float slope = -2.0*a/AHRS_Width;
  int intercept = AHRS_Height/2+a+pitch_offset;
  
  if (roll==90)
  {
    AhrsSprite.fillRect(0,0,AHRS_Width/2, AHRS_Height, AHRS_GroundColor);
  }
  else
  if (roll==-90)
  {
    AhrsSprite.fillRect(AHRS_Width/2,0,AHRS_Width/2, AHRS_Height, AHRS_GroundColor);
  }
  else
  if (abs(roll)<90)
  {
    for (int k=0; k<AHRS_Width; k++)
    {
      if (k*slope+intercept < 0)
        AhrsSprite.drawFastVLine(k, 0, AHRS_Height, AHRS_GroundColor);
      else if (k*slope+intercept < AHRS_Height)
        AhrsSprite.drawFastVLine(k, k*slope+intercept, AHRS_Height, AHRS_GroundColor);
    }
  }
  else
  if (abs(roll)>90)
  {
    for (int k=0; k<AHRS_Width; k++)
    {
      if (k*slope+intercept > AHRS_Height)
        AhrsSprite.drawFastVLine(k, 0, AHRS_Height, AHRS_GroundColor);
      else if (k*slope+intercept > 0)
        AhrsSprite.drawFastVLine(k, 0, k*slope+intercept, AHRS_GroundColor);
    }
  }
}

void drawCrosshair(int Size)
{
  int Length = 2*Size+1;
  AhrsSprite.drawFastHLine(0, AHRS_Height/2, AHRS_Width, TFT_BLACK);
  AhrsSprite.drawFastHLine(AHRS_Width/2-Size, AHRS_Height/2+1, Length, TFT_BLACK); //AhrsSprite.drawFastHLine(0, AHRS_Height/2+1, AHRS_Width, TFT_BLACK);
  //AhrsSprite.drawFastHLine(AHRS_Width/2-Size, AHRS_Height/2-2, Length, TFT_BLACK);
  AhrsSprite.drawFastHLine(AHRS_Width/2-Size, AHRS_Height/2-1, Length, TFT_BLACK);
  //AhrsSprite.drawFastHLine(AHRS_Width/2-Size, AHRS_Height/2+2, Length, TFT_BLACK);
  //AhrsSprite.drawFastVLine(AHRS_Width/2+2, AHRS_Height/2-Size, Length, TFT_BLACK);
  AhrsSprite.drawFastVLine(AHRS_Width/2+1, AHRS_Height/2-Size, Length, TFT_BLACK);
  AhrsSprite.drawFastVLine(AHRS_Width/2, AHRS_Height/2-Size, Length, TFT_BLACK);
  AhrsSprite.drawFastVLine(AHRS_Width/2-1, AHRS_Height/2-Size, Length, TFT_BLACK);
  //AhrsSprite.drawFastVLine(AHRS_Width/2-2, AHRS_Height/2-Size, Length, TFT_BLACK);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

// #########################################################################
// Draw a number in a rounded rectangle with some transparent pixels
// #########################################################################
// void numberBox(int x, int y, float num )
// 
//   // Size of sprite
//   #define IWIDTH  80
//   #define IHEIGHT 35
//   // Create a 8 bit sprite 80 pixels wide, 35 high (2800 bytes of RAM needed)
//   img.setColorDepth(8);
//   img.createSprite(IWIDTH, IHEIGHT);
//   // Fill it with black (this will be the transparent colour this time)
//   img.fillSprite(TFT_BLACK);
//   // Draw a background for the numbers
//   img.fillRoundRect(  0, 0,  80, 35, 15, TFT_RED);
//   img.drawRoundRect(  0, 0,  80, 35, 15, TFT_WHITE);
//   // Set the font parameters
//   img.setTextSize(1);           // Font size scaling is x1
//   img.setTextColor(TFT_WHITE);  // White text, no background colour
//   // Set text coordinate datum to middle right
//   img.setTextDatum(MR_DATUM);
//   // Draw the number to 3 decimal places at 70,20 in font 4
//   img.drawFloat(num, 3, 70, 20, 4);
//   // Push sprite to TFT screen CGRAM at coordinate x,y (top left corner)
//   // All black pixels will not be drawn hence will show as "transparent"
//   img.pushSprite(x, y, TFT_BLACK);
//   // Delete sprite to free up the RAM
//   img.deleteSprite();
// }
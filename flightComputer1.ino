#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ICM_20948.h>
#include <Servo.h>
#include <SD.h>


//  I2C CONFIGURATION
#define WIRE_PORT Wire
#define AD0_VAL 1


// SERVO & SENSOR PINS & SD Card
#define SERVO_PIN 9
#define POT_PIN A0

// SENSOR OBJECTS
ICM_20948_I2C myICM;
Adafruit_BMP280 bmp;
Servo myservo;


//CALIBRATION & BASELINE
float initalP;

// FLIGHT STATE VARIABLES
uint32_t launchTime = 0;
uint32_t deploymentTime = 0;
uint32_t landingTime = 0;


bool fActive = false;      // Recording data
bool launched = false;          // Launch detected
bool deployed = false;          // Parachute deployed
bool landed = false;            // Landing detected
bool failSafe = false; // Tilt failsafe triggered


// Acceleration spike detection
unsigned long startT = 0;


//FLIGHT PARAMETERS
const float launchThreshold = 2000.0;   // 2G in mg
const float altThreshold = 1.0;          // feet
const unsigned long DEPLOY_DELAY = 7000;       // 7 seconds after launch
const unsigned long servoT = 1000;  // 1 second servo movement
const float armedThreshold = 100.0;      // mg (threshold for "zero" acceleration)
const float FAILSAFE_ANGLE = 90;        // degrees from vertical
const float GRAVITY_MG = 1000.0;        //approx g

//SD SETUP
const int chipSelect = 10;
const char filename[] = "FLIGHT.txt";
File myFile;
String dataBuffer;
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);
  Serial.println(F("Flight Computer"));
  myservo.write(88);
  // Initialize I2C
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  // Initialize SD CARD
  dataBuffer.reserve(128);
    while (!Serial);
    if (!SD.begin(chipSelect)) {
      Serial.println(" SD card initialization failed.");
      while (true);
    }
    myFile = SD.open(filename, FILE_WRITE);
    if (!myFile) {
      Serial.print(F("error opening file "));
      while (true);
    }
    Serial.println(F("Starting to write to file..."));
    myFile.println(F("Time(ms) | Alt(ft) | AccelMag(mg) | Tilt(°)"));
    myFile.println(F("--------|---------|--------------|---------"));
  // Initialize BMP280 (Barometer)
  if (!bmp.begin()) {
    Serial.println(F("Could not find BMP280"));
    while (1) delay(10);
  }


  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);


  // Initialize ICM-20948 (IMU)
  bool imuInital = false;
  while (!imuInital) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    
    if (myICM.status == ICM_20948_Stat_Ok) {
      imuInital = true;
    } else {
      Serial.println(F("Error: Retrying ICM..."));
      delay(500);
    }
  }

  // Calibrate baseline pressure
  initalP = bmp.readPressure() / 100;
 
  // Attach servo and set to neutral
  myservo.attach(SERVO_PIN);
  myservo.write(88); // Neutral position
 
  Serial.println(F("All systems ready. Awaiting launch...\n"));
  Serial.println(F("Started"));
}

// SENSOR READING FUNCTIONS
float filtAlt() {
  float avgAlt = 0;
 
  for (int i = 0; i < 5; i++) {
    avgAlt += bmp.readAltitude(initalP) * 3.28084; // Convert to feet
    delay(5);
  }
  return avgAlt / 5;
}

float CheclAccMag() {
  if (myICM.dataReady()) {
    myICM.getAGMT();
   
    float accX = myICM.accX();
    float accY = myICM.accY();
    float accZ = myICM.accZ();
    float accMag = sqrt(accX * accX + accY * accY + accZ * accZ);
   
    return accMag;
  }
  return 0.0;
}


float getTiltAngle() {
  if (myICM.dataReady()) {
    myICM.getAGMT();
   
    float accX = myICM.accX();
    float accY = myICM.accY();
    float accZ = myICM.accZ();
   
    // Calculate total acceleration magnitude
    float totalAcc = sqrt(accX * accX + accY * accY + accZ * accZ);
   
    // (0° = upright, 90° = horizontal)
    // Calculate angle from vertical accounting for all three axes
    float tiltAngle = acos(accX / totalAcc) * 180.0 / PI;
   
    return tiltAngle;
  }
  return 0.0;
}



void saveFlightData(float alt, float accelMag, float tiltAngle) {
  unsigned long elapsed = fActive ? (millis() - launchTime) : 0;

  myFile.print(elapsed);
  myFile.print(F(" | "));
  myFile.print(alt, 2);
  myFile.print(F(" | "));
  myFile.print(accelMag, 1);
  myFile.print(F(" | "));
  myFile.print(tiltAngle, 1);
  myFile.print(F(" | "));
  myFile.flush();
}


//MAIN LOOP

void loop() {
  float curAlt = filtAlt();
  float accMag = CheclAccMag();
  float tiltAngle = getTiltAngle();
  unsigned int chunkSize = 128;
 
  //TILT FAILSAFE CHECK=
  if (fActive && !deployed && tiltAngle >= FAILSAFE_ANGLE) {
    if (!failSafe) {
      failSafe = true;
      deployed = true;
      deploymentTime = millis();
      myservo.write(0); // Deploy servo
     
      
      Serial.println(F(">>> TILT FAILSAFE TRIGGERED <<<"));
      myFile.println(F(">>> TILT FAILSAFE TRIGGERED <<<"));
      myFile.print(F("Tilt Angle: "));
      myFile.print(tiltAngle, 1);
      myFile.print(F("Deployment Time: "));
      myFile.print(millis() - launchTime);
      myFile.println(F(" ms (from launch)"));
      myFile.print("\n");
      myFile.flush();
    }
  }
 
  // LAUNCH DETECTION 
  if (!fActive && accMag >= launchThreshold) {
    fActive = true;
    launched = true;
    launchTime = millis();
    startT = 0;
    failSafe = false;
   
    
    Serial.println(F(">>> LAUNCH DETECTED - FLIGHT STARTED <<<"));
    myFile.println(F(">>> LAUNCH DETECTED - FLIGHT STARTED <<<"));
    myFile.print(F("Launch Time: "));
    myFile.print(launchTime);
    myFile.println(F(" ms"));
    
    myFile.flush();
    
  }
 
  //RECORDING PHASE 
  if (fActive && !landed) {
    unsigned long flightElapsed = millis() - launchTime;
    int status = 0;
   
    // Check for deployment condition (normal deployment, not failsafe)
    if (!deployed && curAlt >= altThreshold && flightElapsed >= DEPLOY_DELAY) {
      deployed = true;
      deploymentTime = millis();
      myservo.write(0); // Deploy servo
      
      
      Serial.println(F(">>> PARACHUTE DEPLOYED <<<"));
      myFile.println(F(">>> PARACHUTE DEPLOYED <<<"));
      myFile.print(F("Deployment Time: "));
      myFile.print(flightElapsed);
      myFile.println(F(" ms (from launch)"));
      myFile.flush();
      
    }
   
    // Retract servo after 1 second
    if (deployed && status == 0) {
      unsigned long servoElapsed = millis() - deploymentTime;
      if (servoElapsed >= servoT) {
        myservo.write(88); // Retract
        
      }
    }
   
    // Check for landing (0 acceleration for 5+ seconds)
    if (accMag <= armedThreshold) {
      if (startT == 0) {
        startT = millis();
      } else if ((millis() - startT) >= 500) {
        landed = true;
        landingTime = millis();
        unsigned long flightT = landingTime - launchTime;
       
        
        Serial.println(F(">>> LANDING DETECTED <<<"));
        myFile.println(F(">>> LANDING DETECTED <<<"));
        myFile.print(F("Landing Time: "));
        myFile.print(landingTime);
        myFile.println(F(" ms"));
        myFile.print(F("Total Flight Time: "));
        myFile.print(flightT);
        myFile.println(F(" ms"));
        myFile.flush();
        
       
       
        if (failSafe) {
          myFile.println(F("Flight ended with failsafe\n"));
        }
      }
    } else {
      startT = 0; // Reset timer if acceleration detected
    }
   

    
    saveFlightData(curAlt, accMag, tiltAngle);
  }
 
  //IDLE PHASE
  if (!fActive) {
    
    
    saveFlightData(curAlt, accMag, tiltAngle);
  }
  if (chunkSize && dataBuffer.length() >= chunkSize) {
    
    dataBuffer.remove(0, chunkSize);
  }
 
  delay(50);
  myFile.flush();
}


# project.github
#Neo 7-M GPS module Code with distance measuring between two point ( between two longitude and latitude)
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include<math.h>





//motor pin declaration
//MOTOR 1 COMBINATION
// Motor A connections

int in1 = 2;
int in2 = 3;
// Motor B connections

int in3 = 4;
int in4 = 5;

//MOTOR 2 COMBINATION
// Motor A connections

int in21 = 10;
int in22 = 11;
// Motor B connections

int in23 = 12;
int in24 = 13;


/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 8(rx) and 9(tx) and a HMC5883 Magnetic Compass
   connected to the SCL/SDA pins.
*/

static const int RXPin = 8, TXPin = 9;
static const uint32_t GPSBaud = 9600;
//SoftwareSerial ss(9, 8);

// Assign a Uniquej ID to the HMC5883 Compass Sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;
TinyGPS gps1;

// The serial connection to the NEO-6m GPS module
SoftwareSerial ss(RXPin, TXPin);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  ss.begin(GPSBaud);

  Serial.println(F("Simple Test with TinyGPS++ and attached NEO-6M GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  displaySensorDetails();

   //setup for motors
   // Set all the motor control pins to outputs
  
 // Set all the motor control pins to outputs
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

     // Set all the motor control pins to outputs
  
  pinMode(in21, OUTPUT);
  pinMode(in22, OUTPUT);
  pinMode(in23, OUTPUT);
  pinMode(in24, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in21, LOW);
  digitalWrite(in22, LOW);
  digitalWrite(in23, LOW);
  digitalWrite(in24, LOW);
  
}
void loop()
{

  // This sketch displays information every time a new sentence is correctly encoded from the GPS Module.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayGpsInfo();
//      Serial.println("First1 while");
//      Serial.println(ss.available());

   float deslat=21.184738,deslong=72.784744;
  while (ss.available())
  {
//    Serial.println("2 while");
    int c = ss.read();
    if (gps.encode(c))
    {
//      Serial.println("IF LOOP");
float lat,lng;
//      gps1.f_get_position(&lat,&lng);
      lat = gps.location.lat();
      lng = gps.location.lng();
      Serial.print("Latitude :");
      Serial.println(lat, 6);
      Serial.print("Longitude:");
      Serial.println(lng, 6);
      float distancelat,distancelong;
    /*distancelat=deslat-lat;
  distancelong=deslong-lng;
  distance=sqrt((distancelat*distancelat) + (distancelong*distancelong));*/
  
  /*Serial.print("logitude distance :");
    Serial.println(distancelong, 6);
      Serial.print("lattitude distance ");
    Serial.println(distancelat, 6);*/
 
     /*float haversine(float lat1, float lon1, float lat2, float lon2) {*/
    const float rEarth = 6371000.0; // in meters
    float x = pow( sin( ((deslat - lat)*M_PI/180.0) / 2.0), 2.0 );
    float y = cos(deslat*M_PI/180.0) * cos(lat*M_PI/180.0);
    float z = pow( sin( ((deslong - lng)*M_PI/180.0) / 2.0), 2.0 );
    float a = x + y * z;
    float c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    float distance = rEarth * c;
    Serial.print("distance ");
    Serial.println(distance, 6);


    //going forward
    if(distance>=10)
     {
      digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  digitalWrite(in21, LOW);
  digitalWrite(in22, HIGH);
  digitalWrite(in23, LOW);
  digitalWrite(in24, HIGH);
     }
    }
  }
     




    
}
   
void displayGpsInfo()
{
  // Prints the location if lat-lng information was recieved
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  // prints invalid if no information was recieved in regards to location.
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  // prints the recieved GPS module date if it was decoded in a valid response.
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    // prints invalid otherwise.
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  // prints the recieved GPS module time if it was decoded in a valid response.
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    // Print invalid otherwise.
    Serial.print(F("INVALID"));
  }
  Serial.println();
  if(mag.begin())
  {
    displayCompassInfo();
  }
}

void displayCompassInfo()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.00319976971;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  
  delay(500);
}

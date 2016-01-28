/* 
   SKETCH FOR READING PMOD GYRO BASED ON
   http://arduino.cc/en/Tutorial/BarometricPressureSensor
   http://arduino.cc/en/Tutorial/SPIEEPROM
   http://lbaigy.wordpress.com/2013/05/01/a-example-sketch-for-arduino-spi-communication/
*/

//GPS and ACL communicate by SPI
#include <SPI.h>
//GPS Library
#include "TinyGPS.h"
//ROS libraries
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

TinyGPS gps; // create a TinyGPS object

//Most used registers in GYRO L3G4200D
const byte CTRL_REG1 = 0b00100000;
const byte GYRO_AXIS = 0b00101000;

//Most used registers in ACL ADXL362
const byte PWR_CONTROL = 0x2D;
const byte RESET_REG = 0x1F;
const byte ACL_AXIS = 0x0E;

//GYRO COMMANDS
const byte GYRO_DEFAULT_POWER_UP = 0b01011111; //200HZ ODR with 25 Cut-Off
const byte GYRO_READ = 0b10000000;
const byte GYRO_WRITE = 0b01111111;
const byte moreBytes = 0b01000000;
const byte lastByte = 0b10111111;

//ACL COMMANDS
const byte ACL_DEFAULT_POWER_UP = 0x12;
const byte RESET = 0x52;
const byte ACL_READ = 0x0B;
const byte ACL_WRITE = 0x0A;

//Pins used to connect to the sensors
//const int dataReady=6; //FUTURE IMPLEMENTATION
const int GYRO_chipSelect=46;
const int ACL_chipSelect=42;
const int sonarSignal=7;

//Variables for reading
byte GYRO_values[10];
byte ACL_values[10];

//LOOP CONTROL TO LIMIT SONAR READING FREQUENCY
int last_reading=0;

//ROS initialization
ros::NodeHandle nh;

geometry_msgs::Twist twist_msg;
std_msgs::Int64 lat_msg;
std_msgs::Int64 lon_msg;
std_msgs::Int64 fix_msg;
std_msgs::String NMEA_msg;
sensor_msgs::Range distance_msg;



ros::Publisher twist_pub("TWIST", &twist_msg);
ros::Publisher lat_pub("lat", &lat_msg);
ros::Publisher lon_pub("lon", &lon_msg);
ros::Publisher fix_pub("fix", &fix_msg);
ros::Publisher NMEA_pub("NMEA", &NMEA_msg);
ros::Publisher distance_pub("distance_m", &distance_msg);


void setup()
{
  //start the SPI library
  SPI.begin();
  //start the Serial at 9600 baud rate
  Serial3.begin(9600); // Default PMODGPS Baud Rate
  
  //initialize the data mode of the pins
  pinMode(GYRO_chipSelect, OUTPUT);
  digitalWrite(GYRO_chipSelect, HIGH);
  pinMode(ACL_chipSelect, OUTPUT);
  digitalWrite(ACL_chipSelect, HIGH);
  pinMode(sonarSignal, INPUT);
  
  //GYRO REGISTER CONFIGURATION
  gyroWriteRegister(CTRL_REG1, GYRO_DEFAULT_POWER_UP);
  //ACL REGISTER CONFIGURATION
  aclWriteRegister(RESET_REG, RESET);
  delay(10);
  aclWriteRegister(PWR_CONTROL, ACL_DEFAULT_POWER_UP);
  
  //ROS INITIALIZATION
  nh.initNode();
  
  nh.advertise(twist_pub);
  nh.advertise(lat_pub);
  nh.advertise(lon_pub);
  nh.advertise(fix_pub);
  nh.advertise(NMEA_pub);
  nh.advertise(distance_pub);
  distance_msg.radiation_type=0;
  distance_msg.min_range = 0.2;
  distance_msg.max_range = 3;
}

char vetor[100], count=0;

void loop()
{
  /* START OF GPS ROUTINE */
  last_reading++;
  if(last_reading>50)
  {
    last_reading=0;
    float tempo, dist;
    tempo = pulseIn(sonarSignal, HIGH);
    dist = ((tempo * 350)/1000000)/2;
    distance_msg.range = (float)(dist);
    distance_pub.publish( &distance_msg );
  }	
  
  if(Serial3.available())
  {
    int c = Serial3.read();
    // Encode() each byte
    // Check for new position if encode() returns "True"
    if(c==36 && count!=0)
    {
      vetor[count]='\0';
      NMEA_msg.data = vetor;
      NMEA_pub.publish ( &NMEA_msg );
      count=0;
    }
    vetor[count++]=c;
    if (gps.encode(c))
    {
      long lat, lon;
      unsigned long fix_age;
      gps.get_position(&lat, &lon, &fix_age);
      
      //ROS MESSAGES
      lat_msg.data = lat;
      lon_msg.data = lon;
      fix_msg.data = fix_age;
      //Print the results to ROS
      lat_pub.publish( &lat_msg );
      lon_pub.publish( &lon_msg );
      fix_pub.publish( &fix_msg );
      
      /*
      if(fix_age == TinyGPS::GPS_INVALID_AGE)
      {
        //NO FIX EVER DETECTED
      }
      else if(fix_age > 2000)
      {
        //STALE DATA
      }
      else
      {
        //VALID DATA
        if (lat < 0) // Southern Hemisphere?
          digitalWrite(HEMISPHERE_PIN, HIGH);
        else
          digitalWrite(HEMISPHERE_PIN, LOW);
      }
      */
      
    }
  }
  /* START OF GYRO READING ROUTINE */
  gyroReadRegister(GYRO_AXIS, 6, GYRO_values);


  //The L3G4200D gives 16-bit values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  
  //DEFAULT FOR READING IS 250dps, and, in this configuration, the sensibility is approximatelly 114 LSB per degree. Multiplying that by pi and dividing by 180 you achieve the angular velocity, in rad/s.
  twist_msg.linear.x = (float)((((int)GYRO_values[1]<<8)|(int)GYRO_values[0]))*0.0001527163;
  twist_msg.linear.y = (float)((((int)GYRO_values[3]<<8)|(int)GYRO_values[2]))*0.0001527163;
  twist_msg.linear.z = (float)((((int)GYRO_values[5]<<8)|(int)GYRO_values[4]))*0.0001527163;
  
  /* END OF GYRO READING ROUTINE*/
  
  /* START OF ACL READING ROUTINE */
  aclReadRegister(ACL_AXIS, 6, ACL_values);
  
  //DEFAULT FOR READING IS +-2g, and, in this configuration, the sensibility is approximatelly 1000LSB per g.
  twist_msg.angular.x = (float)((((int)ACL_values[1]<<8)|(int)ACL_values[0]))*0.00980665;
  twist_msg.angular.y = (float)((((int)ACL_values[3]<<8)|(int)ACL_values[2]))*0.00980665; 
  twist_msg.angular.z = (float)((((int)ACL_values[5]<<8)|(int)ACL_values[4]))*0.00980665;
  
  //Print the results to ROS
  twist_pub.publish ( &twist_msg );
  
  nh.spinOnce();
  //delay(1);
}


void gyroReadRegister(byte registerAddress, int numBytes, byte * values){
  //configure SPI Data mode for the L3G4200D - CPOL=CPHA=1
  SPI.setDataMode(SPI_MODE3);
  //Since we're performing a read operation, the most significant bit (bit 7(counting starts from bit 0)) of the register address should be set high.
  byte dataToSend = GYRO_READ | registerAddress;
  //If we're doing a multi-byte read, bit 6 (counting starts from bit 0)needs to be set high as well.
  if(numBytes > 1)
  {
    dataToSend = dataToSend | moreBytes;
  }
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(GYRO_chipSelect, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(dataToSend);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  //SPI is full duplex transsmission. Every signle time SPI.transfer can and can only transmit 8 bits. the input parameter will be delived to slave, and it's return will be 
  // the data that master device and get from the slave device. The data returned during master device's write operation makes no sense, hence always been ignored by not 
  // doing any assignment operation.
  for(int i=0; i<numBytes; i++){
    GYRO_values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(GYRO_chipSelect, HIGH);
}

void gyroWriteRegister(byte thisRegister, byte thisValue)
{
  //configure SPI Data mode for the L3G4200D - CPOL=CPHA=1
  SPI.setDataMode(SPI_MODE3);
  byte dataToSend = thisRegister & GYRO_WRITE;
  digitalWrite(GYRO_chipSelect,LOW);
  
  SPI.transfer(dataToSend);
  SPI.transfer(thisValue);
  
  digitalWrite(GYRO_chipSelect, HIGH);
}

void aclReadRegister(byte registerAddress, int numBytes, byte * values){
  //configure SPI Data mode for the ADXL362 - CPOL=0 CPHA=0
  SPI.setDataMode(SPI_MODE0);
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(ACL_chipSelect, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(ACL_READ);
  SPI.transfer(registerAddress);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  //SPI is full duplex transsmission. Every signle time SPI.transfer can and can only transmit 8 bits. the input parameter will be delived to slave, and it's return will be 
  // the data that master device and get from the slave device. The data returned during master device's write operation makes no sense, hence always been ignored by not 
  // doing any assignment operation.
  for(int i=0; i<numBytes; i++)
  {
    ACL_values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(ACL_chipSelect, HIGH);
}

void aclWriteRegister(byte thisRegister, byte thisValue)
{
  //configure SPI Data mode for the ADXL362 - CPOL=0 CPHA=0
  SPI.setDataMode(SPI_MODE0);
  
  digitalWrite(ACL_chipSelect,LOW);
  
  SPI.transfer(ACL_WRITE);
  SPI.transfer(thisRegister);
  SPI.transfer(thisValue);
  
  digitalWrite(ACL_chipSelect, HIGH);
}



    
  
    
    
  

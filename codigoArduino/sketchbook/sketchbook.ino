/* 
   SKETCH FOR READING PMOD GYRO BASED ON
   http://arduino.cc/en/Tutorial/BarometricPressureSensor
   http://arduino.cc/en/Tutorial/SPIEEPROM
   http://lbaigy.wordpress.com/2013/05/01/a-example-sketch-for-arduino-spi-communication/
*/

//GPS and ACL communicate by SPI

#include "acl_ADXL362.h"
#include "gyro_L3G4200D.h"
#include "kinect.h"
#include "sonar.h"

#include <SPI.h>
#include <Servo.h>
#include <TimerOne.h>

#include <TinyGPS.h>

//ROS libraries
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#define NUMREADINGS 100

const int MAX_DISTANCE = 300;

//LOOP CONTROL TO LIMIT SONAR READING FREQUENCY
int last_reading=0;

TinyGPS gps; // create a TinyGPS object

//ROS initialization
ros::NodeHandle nh;


//initializing the range and intensities size of the vectors -> arduino exclusive
float rangesVetor[NUMREADINGS];

std_msgs::Int64 lat_msg;
std_msgs::Int64 lon_msg;
std_msgs::Int64 fix_msg;
std_msgs::String NMEA_msg;

/*********************** SENSORES *********************/

//Pins used to connect to the sensors
const int GYRO_chipSelect=46;
const int ACL_chipSelect=42;
const int sonarPin = 7; 

geometry_msgs::Twist acl_gyro_msg;
//sensor_msgs::LaserScan sonar_msg;
std_msgs::Float32 sonar_msg;

ros::Publisher acl_gyro_pub("acl_gyro_data", &acl_gyro_msg);
ros::Publisher sonar_pub("sonar_data", &sonar_msg);

ros::Publisher lat_pub("gps_lat_data", &lat_msg);
ros::Publisher lon_pub("gps_lon_data", &lon_msg);
//ros::Publisher fix_pub("fix", &fix_msg);
//ros::Publisher NMEA_pub("NMEA", &NMEA_msg);

/*********************** KINECT ***********************/

const int kinectServoPin = 5;

void kinectCb( const std_msgs::Int32& kinect_state )
{
  if(kinect_state.data >= 0 && kinect_state.data < KINECT_NUM_STATES)
  {
     kinect_setState((kinect_states) kinect_state.data);
  }
}
ros::Subscriber<std_msgs::Int32> kinect_sub("kinect_state", kinectCb);

std_msgs::Float32 kinect_angle_msg;
ros::Publisher kinect_angle_pub("kinect_horizontal_angle", &kinect_angle_msg);

/********************************************************/

int num_readings = 1;
double sonar_frequency = 20;

void setup()
{
  SPI.begin();
  // Serial.begin(115200);
  
  //initialize the data mode of the pins
  pinMode(GYRO_chipSelect, OUTPUT);
  digitalWrite(GYRO_chipSelect, HIGH);
  pinMode(ACL_chipSelect, OUTPUT);
  digitalWrite(ACL_chipSelect, HIGH);
//  pinMode(sonarPin, INPUT);
  
  //inits
  
  gyro_init(GYRO_chipSelect);
  
  kinect_init(kinectServoPin);
  
  Timer1.initialize(15000);         // initialize timer1, and set a 15ms period
  Timer1.attachInterrupt(kinect_update);  // attaches callback() as a timer overflow interrupt

  //ROS INITIALIZATION
  nh.initNode();
  
  nh.subscribe(kinect_sub);
  
  nh.advertise(sonar_pub);
  nh.advertise(acl_gyro_pub);
  nh.advertise(kinect_angle_pub);
  nh.advertise(lat_pub);
  nh.advertise(lon_pub);
//  nh.advertise(fix_pub);
//  nh.advertise(NMEA_pub);
//  nh.advertise(twist_pub);

}

char vetor[100], count=0;
float x = 0,y = 0,z = 0;

void readGps()
{
  if(Serial3.available())
  {
    int c = Serial3.read();
    // Encode() each byte
    // Check for new position if encode() returns "True"
    if(c==36 && count!=0)
    {
      vetor[count]='\0';
//      NMEA_msg.data = vetor;
//      NMEA_pub.publish ( &NMEA_msg );
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
      //fix_pub.publish( &fix_msg );
      
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
}

unsigned long lastTimeLoop = 0;
void loop()
{
  unsigned long time = millis();
  if(time - lastTimeLoop > 50)
  {
    lastTimeLoop = time;
    float distSonar = sonar_read(sonarPin);

// Float 32    
    sonar_msg.data = distSonar;
    sonar_pub.publish(&sonar_msg);
/* **Laser Scan
    ros::Time scan_time = nh.now();

    sonar_msg.header.stamp = scan_time;
    sonar_msg.header.frame_id = "sonar_frame";
    sonar_msg.angle_min = -0.1;
    sonar_msg.angle_max = 0.1;
    sonar_msg.angle_increment = (sonar_msg.angle_max - sonar_msg.angle_min) / NUMREADINGS;
    sonar_msg.time_increment = (1 / sonar_frequency) / (NUMREADINGS);
    sonar_msg.range_min = 0.20;
    sonar_msg.range_max = 1.5;
  
    sonar_msg.ranges_length = NUMREADINGS;
    sonar_msg.ranges = rangesVetor;
    for(int i =0; i < NUMREADINGS; i++)
    {
       rangesVetor[i] = distSonar;
    }
  */  
      
    acl_read_xyz(&x, &y, &z);  
    acl_gyro_msg.linear.x = x;
    acl_gyro_msg.linear.y = y;
    acl_gyro_msg.linear.z = 0;

    gyro_read_xyz(&x, &y, &z);
    acl_gyro_msg.angular.x = x;
    acl_gyro_msg.angular.y = y;
    acl_gyro_msg.angular.z = z;
  
    acl_gyro_pub.publish(&acl_gyro_msg);

    void readGps();

    kinect_angle_msg.data = kinect_currentAngle();
    kinect_angle_pub.publish(&kinect_angle_msg);
  
    nh.spinOnce();
  }
}


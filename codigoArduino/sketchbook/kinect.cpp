
#include "kinect.h"

static Servo g_kinectServo;
static int g_currentAngle = 0;
static kinect_states g_currentState = KINECT_STOP;
int sweep_side = 0;
int sweep_counter = 0;

void kinect_init(const int servoPin)
{
  g_kinectServo.attach(servoPin);

  kinect_setAngle(0);
}

void kinect_update()
{
  switch(g_currentState)
  {
    case KINECT_LEFT: 
      if(g_currentAngle < 115)
      { 
        g_currentAngle+=1;
        g_kinectServo.write(g_currentAngle);
      }
    break;
    case KINECT_RIGHT:
      if(g_currentAngle > 5) 
      {
        g_currentAngle-=1;
        g_kinectServo.write(g_currentAngle);
      }
    break;
    case KINECT_SWEEP:
      if(g_currentAngle == 5)
      {
        sweep_counter++;
	if(sweep_counter < 80)
	  return;
	else
	  sweep_counter = 0;
        if(sweep_side) 
        { 
          sweep_side = 0;
        }
        else
        {
          sweep_side = 1;
        }
      }
      else if(g_currentAngle == 60)
      {
	sweep_counter++;
	if(sweep_counter < 80)
	  return;
	else
          sweep_counter = 0;
      }
      else if(g_currentAngle == 115)
      {
        sweep_counter++;
	if(sweep_counter < 80)
	  return;
	else
	  sweep_counter = 0;
        if(sweep_side) 
        { 
          sweep_side = 0;
        }
        else
        {
          sweep_side = 1;
        }
      }
      if(sweep_side) 
      { 
	g_currentAngle++;
      }
      else
      {
	g_currentAngle--;
      }
      g_kinectServo.write(g_currentAngle);
    break;
    case KINECT_RESET:
      g_currentAngle = 60;
      g_kinectServo.write(g_currentAngle);
    break;
  }
}

void kinect_setState(kinect_states state)
{
  g_currentState = state;
}

kinect_states kinect_currentState()
{
  return g_currentState;
}

float kinect_currentAngle()
{
  return g_currentAngle - 60;
}

void kinect_setAngle(float angle)
{
  angle += 60;
  if(angle < 120 && angle > 0)
  {
    g_currentAngle = angle;
    g_kinectServo.write(g_currentAngle);
  }
}

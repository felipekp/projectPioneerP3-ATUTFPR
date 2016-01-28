#include "sonar.h"

static float g_lastDist = 0;
static unsigned long g_lastTime = 0;

float sonar_read(const int sonarPin)
{
  unsigned long time = millis();
  
  if(time - g_lastTime > 50)
  {
    g_lastTime = time;
    float tempo = pulseIn(sonarPin, HIGH);
    float tempDist = (tempo / float(5877));
    
    if(tempDist > 0)
    {
      g_lastDist = tempDist;
      return tempDist;
    }
  }
  return g_lastDist;
}

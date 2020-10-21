/**************************************************************
This file contains GPS data converted from geodesic system to cartesian coordinate
**************************************************************/
void buildCartesianCoordinates(TinyGPS &gps, float &lat, float &lon){
  float *coordinateXvalue;
  float *coordinateYvalue;
  float *coordinateZvalue;

  lat = lat * PI/180.0;
  lon = lon * PI/180.0;
  
  coordinateX = radius * cos(lat) * cos(lon);
  coordinateY = radius * cos(lat) * sin(lon);
  coordinateZ = radius * sin(lat);

  coordinateXvalue = &coordinateX;
  coordinateYvalue = &coordinateY;
  coordinateZvalue = &coordinateZ;

  if(counter == 5){
    initialX =  absoluteValue(*coordinateXvalue);
    initialY =  absoluteValue(*coordinateYvalue);
    initialZ =  absoluteValue(*coordinateZvalue);
  }
  if(counter > 5){

    GPS_pos[0] = absoluteValue(absoluteValue(coordinateX) - initialX);
    GPS_pos[1] = absoluteValue(absoluteValue(coordinateY) - initialY);
    GPS_pos[2] = absoluteValue(absoluteValue(coordinateZ) - initialZ);
  }
  else{
    GPS_pos[0] = 0;
    GPS_pos[1] = 0;
    GPS_pos[2] = 0;
  }
  
  counter ++;
}

void getGPSData(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  float coordinateX, coordinateY, coordinateZ;

  gps.get_position(&lat, &lon, &age);

  gps.f_get_position(&flat, &flon, &age);
  buildCartesianCoordinates(gps, flat, flon);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
    Serial.print('-');
    number = -number;
 }
  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

//This file handle parsing GPS data 
   int gpsval = 0;
   float latitude  = 0.000000;
   float longitude = 0;
   float gAltitude = 0;
   int gps_sat = 0;
   String gpsTime  = "";

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec, int gpsval);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

void softwareSerialEvent() // this is called at the end of loop()
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
//  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6, 1);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6, 2);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2, 3);
  //print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2, 0);
 // print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2, 0);
 // print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
//  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2, 0);
 // print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
 // Serial.println(); 
  smartdelay(0);
}

static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
} 
static void print_float(float val, float invalid, int len, int prec, int gpsval)
{
  if (val == invalid)
  {
    if (gpsval == 1) latitude = 1111.0000;
    else if (gpsval == 2) longitude = 1111.0000;
    else if (gpsval == 3) gAltitude = 1.000000;
  }
  else
  {
   // Serial.print(val, prec);
    if (gpsval == 1) latitude = val;
    else if (gpsval == 2)  longitude = val;
    else if (gpsval == 3) gAltitude = val;
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    gps_sat = 5;
  else
    gps_sat = val;
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
 gpsTime = "";
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", hour, minute, second);

    gpsTime = sz;
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

/*static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
  smartdelay(0);
}*/



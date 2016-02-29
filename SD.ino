// output to the sd card in the following format
//SCHANGE complete
// must be changed to Serial
void output_sd()
{
    Serial.print("#"); // this sign is used to indicate a new entry but you can get rid of it if you want
    Serial.print(TO_DEG(yaw)); Serial.print(",");
    Serial.print(TO_DEG(pitch)); Serial.print(",");
    Serial.print(TO_DEG(roll)); Serial.print(",");
    Serial.print(BMPTemp); Serial.print(",");
    Serial.print(BMPPressure); Serial.print(",");
    Serial.print(latitude,6); Serial.print(",");
    Serial.print(longitude,6); Serial.print(",");
    Serial.print(gps_sat); Serial.print(",");
    Serial.print(gAltitude); Serial.print(",");
    Serial.print(gpsTime); Serial.print(",");
    
    Serial.print(geiger_count); Serial.print(",");
    Serial.print(temperature); Serial.print(",");
    Serial.print(humidity); Serial.print(",");

    Serial.println(timestamp);
}


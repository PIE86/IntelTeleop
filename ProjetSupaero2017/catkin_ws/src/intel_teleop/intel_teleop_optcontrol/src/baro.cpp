

float getStandardPressure(float altitude /* meters */)   //return Pa
      {
        //Below 51km: Practical Meteorology by Roland Stull, pg 12
        //Above 51km: http://www.braeunig.us/space/atmmodel.htm
        //Validation data: https://www.avs.org/AVS/files/c7/c7edaedb-95b2-438f-adfb-36de54f87b9e.pdf
    
        altitude = altitude / 1000.0f;  //convert meters to km
        float geopot_height = getGeopotential(altitude);
        
        float t = getStandardTemperature(geopot_height);
    
        if (geopot_height <= 11)
          return  101325 * pow(288.15f / t, -5.255877f);
        else if (geopot_height <= 20)
          return 22632.06 * exp(-0.1577f * (geopot_height - 11));
        else if (geopot_height <= 32)
          return 5474.889f * pow(216.65f / t, 34.16319f);
        else if (geopot_height <= 47)
          return 868.0187f * pow(228.65f / t, 12.2011f);
        else if (geopot_height <= 51)
          return 110.9063f * exp(-0.1262f * (geopot_height - 47));
        else if (geopot_height <= 71)
          return 66.93887f * pow(270.65f / t, -12.2011f);
        else if (geopot_height <= 84.85)
          return 3.956420f * pow(214.65f / t, -17.0816f);
    
        throw std::out_of_range("altitude must be less than 86km.");    
      }
    
      //geopot_height = earth_radius * altitude / (earth_radius + altitude) /// all in kilometers
      //temperature is in Kelvin = 273.15 + Celsius
      float getStandardTemperature(float geopot_height) 
      {
        //standard atmospheric pressure
        //Below 51km: Practical Meteorology by Roland Stull, pg 12
        //Above 51km: http://www.braeunig.us/space/atmmodel.htm
        if (geopot_height <= 11)          //troposphere
          return 288.15f - (6.5 * geopot_height);
        else if (geopot_height <= 20)     //Staroshere starts
          return 216.65f;
        else if (geopot_height <= 32)
          return 196.65f + geopot_height;
        else if (geopot_height <= 47)       
          return 228.65f + 2.8 * (geopot_height - 32);
        else if (geopot_height <= 51)     //Mesosphere starts
          return 270.65f;
        else if (geopot_height <= 71)       
          return 270.65f - 2.8 * (geopot_height - 51);
        else if (geopot_height <= 84.85)    
          return 214.65f - 2 * (geopot_height - 71);    
        //Thermospehere has high kinetic temperature (500c to 2000c) but temperature
        //as measured by thermometer would be very low because of almost vacuum
        throw std::out_of_range("geopot_height must be less than 84.85km.");
      }
    
      float getGeopotential(float altitude_km)
      {
        constexpr float EARTH_RADIUS =  6356.766; //km
        return EARTH_RADIUS * altitude_km / (EARTH_RADIUS + altitude_km);
      }

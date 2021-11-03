

#include "gaz.hpp"


MICS_VZ_89TE outGaz;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  outGaz.begin();

    Serial.println(  "nFuk l'initialisation" );

}

void loop() {
  // put your main code here, to run repeatedly: 

  delay(1000);

  outGaz.readSensor();


  Serial.print( "C02 : ");
  Serial.print(outGaz.getCO2());
  Serial.print("  -  Gaz : ");
  Serial.println(outGaz.getVOC());
  Serial.println();

  
}

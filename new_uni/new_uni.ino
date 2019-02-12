#include "ACS712.h"
#include <VoltageReference.h>
#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"
#include <lm35.h> // include the library header


float tempC = 0;
float tempC1 = 0;
//float tempC2 = 0;
//float tempC3 = 0;

lm35 temps(2);
lm35 temps1(3);
lm35 temps2(6);
lm35 temps3(7);

#define DHTPIN 2
#define DHTTYPE DHT11

BH1750 lightMeter;
VoltageReference vRef;
ACS712 sensor(ACS712_30A, A1);
DHT dht(DHTPIN, DHTTYPE);

void S_lm35 ();
void S_current();
void S_voltage ();
void S_light ();
void S_temp ();
void sensor_read ();
void displ_data ();

int offset =5;// set the correction offset value

float cunt = 0,powe = 0,temp = 0,humd = 0,luxx = 0;
float luxwatt=0;
double volt =0;
int sn=0;
void setup() {
  Serial.begin(9600);
  sensor.calibrate();

  while (!Serial);
  Serial.println("Calibrating voltage reference");
  vRef.begin();

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));

  Serial.println("DHTxx test!");
  dht.begin();

  


  Serial.println("        Current       Voltage         Power         Tempreture        Humidity        Flux         Illuminance        LM35-1      LM35-2        SNo.      LM35-3      LM35-4");
  }

  #define DETECT_PIN A0
// set this to true for voltages below 1V
#define LOW_VOLTAGE false


////////////////////////////////////////////////////////////////SENSOR CODE////////////////////////////////////////////////////////////////////////////
void S_current() {

  /*
  // Get current from sensor
  float I = sensor.getCurrentDC();
  //float I = analogRead(A1);
  // Send it to serial
  //Serial.println(String("I = ") + I + " A");
  // Wait one second before the new cycle
  cunt = I;
 // cunt = I-509;
  delay(500);
*/
  unsigned int x=0;
float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++){ //Get 150 samples
  AcsValue = analogRead(A0);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
}
AvgAcs=Samples/150.0;//Taking Average of Samples

//((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
//2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
//out to be 2.5 which is out offset. If your arduino is working on different voltage than 
//you must change the offset according to the input voltage)
//0.100v(100mV) is rise in output voltage when 1A current flows at input
AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.100;
//AcsValueF = AcsValueF*100;
cunt = AcsValueF;
delay(500);
}

void S_voltage (){
   /*
   #if (LOW_VOLTAGE)
 analogReference(INTERNAL);
#endif
  //float analog = analogRead(DETECT_PIN);
  //int vcc = vRef.readVcc();
  //Serial.print("Board voltage is ");
  //Serial.print(vcc);
  //Serial.print("mV, analog pin voltage is ");
#if (LOW_VOLTAGE)
  //Serial.print(vRef.internalValue() * analog / 1023);
  //volt =(vRef.internalValue() * analog / 1023);
#else
  //Serial.print(vcc * analog / 1023);
  //volt =(vcc * analog / 1023) ;
#endif
  //Serial.print("mV");
*/
  int volt1 = analogRead(DETECT_PIN);// read the input
  double voltage = map(volt1,0,1023, 0, 2500) + offset;// map 0-1023 to 0-2500 and add correction offset
  
  voltage /=100;// divide by 100 to get the decimal values
  
  volt = voltage;
  //volt= analog/51;
  delay(50);
  
}

void S_light (){
  uint16_t lux = lightMeter.readLightLevel();
  
  
  luxx = lux;
  luxwatt= lux * 0.0079;
  delay(500);
}

void S_temp (){
  // Wait a few seconds
  delay(500);
  
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

 
  temp = t;
  humd = h;
  
}

void S_lm35 (){

  temps.MeasureTemp(); // start measuring temperature
  tempC = temps.TempInCelcius ;
  
  temps1.MeasureTemp(); // start measuring temperature
  tempC1 = temps1.TempInCelcius;


  delay (500);
}



void sensor_read (){
 S_current();
 S_voltage ();
 S_light ();
 S_temp ();
 S_lm35 ();
  sn = sn+1;
 powe=cunt*volt;
}

void displ_data (){
  Serial.println(); // End the line
Serial.print("        ");
Serial.print(cunt);
Serial.print("*A");
Serial.print("        ");
Serial.print(volt);
Serial.print("*mV");
Serial.print("        ");
Serial.print(powe);
Serial.print("*Watt");
Serial.print("        ");
Serial.print(temp);
Serial.print("*C");
Serial.print("        ");
Serial.print(humd);
Serial.print("%");
Serial.print("        ");
Serial.print(luxx);
Serial.print("*LX");
Serial.print("        ");
Serial.print(luxwatt);
Serial.print("*W/m2");

Serial.print("        ");
Serial.print(tempC);
Serial.print("*C");

Serial.print("        ");
Serial.print(tempC1);
Serial.print("*C");

Serial.print("        ");
Serial.print(sn);

 Serial.println();    // End the line
}

void loop (){
   sensor_read ();
   displ_data ();
}


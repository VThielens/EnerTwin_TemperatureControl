// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

// Define the pin
#define ONE_WIRE_BUS 2 // read temperature on PIN 2
#define PIN_OUTPUT 3 // send digital signal HIGH/LOW for the switch

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//Define Variables for PID
double Setpoint, Input, Output;
const double MAX_DURATION = 100; // maximal duration 5000 ms
//Tuning parameters of PID
const double Kp=2, Ki=5, Kd=1;
// create PID instance
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double mapWithResolution(double x, double fromLow, double fromHigh,double toLow,double toHigh,double resolution){
  // check if not out of range
  if (x>fromHigh){
    x = fromHigh;
  }
  // convert a value x from the range [fromLow, fromHigh] to the range [toLow, toHigh] with the resolution
  double y = map(x,fromLow,fromHigh,toLow,toHigh); // map x in the range
  y = int(y/resolution)*resolution; // round y with the resolution
  return y;
}

void thermalOutput(double percentage, double maxDuration, double pinOut, double frequence){
  // send on the pin a percentage of the maximal duration depending on the frequence of commutation
  // the maxDuration in second should not take a too high value otherwise there is a risk that the heater heats for too long
  // the frequence should not be higher than 200 Hz otherwise the commutation will not be seen by the switch (zero crossing method)
  double time = mapWithResolution(percentage,0,100,0,maxDuration,1/frequence); // computation of the duration
  digitalWrite(pinOut, HIGH); // switch on the heater
  delay(time); // time delay during which the heater is switched on
  digitalWrite(pinOut,LOW); // switch off the heater
  delay(maxDuration-time);
}



void setup() {
  // put your setup code here, to run once:
  // start serial port
  Serial.begin(9600);
  Setpoint = -9999; // we initilize with a negative temperature
  pinMode(PIN_OUTPUT,  OUTPUT);
  sensors.begin();
  // read temperature
  sensors.requestTemperatures();
  Input = sensors.getTempCByIndex(0);
  // set mode of PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read temperature
  sensors.requestTemperatures();
  Input = sensors.getTempCByIndex(0);
   if (Serial.available() > 0) { // if temperature level is provided, read it
    char buffer[32]; // adjust the buffer size as needed
    Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // null-terminate the string
    Setpoint = atof(buffer);
   }
  myPID.Compute();
  if (Setpoint <= 0){
    Output = 0;
  }
  // security as working with percentage
  if (Output > 100){
    Output = 100;
  }
    // test if input higher than a given value
  if (Input > 50){
    Output = 0;
  }
  // write the thermal 
  thermalOutput(Output,MAX_DURATION,PIN_OUTPUT,200);
  // return parameters
  Serial.print("Kp:");
  Serial.print(Kp);
  Serial.print(",");
  Serial.print("Ki:");
  Serial.print(Ki);
  Serial.print(",");
  Serial.print("Kd:");
  Serial.print(Kd);
  Serial.print(",");
  Serial.print("Percentage:");
  Serial.print(Output);
  Serial.print(",");
  Serial.print("T_set:");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.print("T_measured:");
  Serial.println(Input);
}

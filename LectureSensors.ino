#include <DHT.h>
#include <DHT_U.h>

int sensor = 2;
int temp;
int humidity;

const int MQ2   = A0;                             // Define analogic entrance of MQ2
const int MQ5   = A1;                             // Define analogic entrance of MQ5
const int MQ7   = A2;                             // Define analogic entrance of MQ7
const int MQ135 = A3;                             // Define analogic entrance of MQ135
const float RL = 1.0;                             // We're using MQ Modules, this modules has their own resistances
const float air[4] = {9.83, 6.50, 27.10, 3.65};   // Aproximated values of air in MQs datasheets
float RS;                                         // kohm
float RO;                                         // kohm
float ratio;                                      // RS/RO
int raw_value;                                    // Values of analogic entrance between 0 to 1023
DHT dht(sensor,DHT22);

// We define 4 variables to detect their values, they´re smoke, LPG, CO and NH4
#define   SMOKE      0
#define   LPG        1
#define   CO         2
#define   NH4        3

/* How we calibrate?
 To calibrate the sensors, first we need to see the datasheet of the sensor, then you can see a graph which has different components (CO, Amoniac, etc)
 The graph has 2 values: Rs/Ro and PPM. So we're goint to calculate the component we want to choose, in this case que use smoke for the MQ2.
 Now to get the 3 most important values for the concentration of a gas, we need to detect the X, Y and slope values, which will be important for the calibration
 For example, in the MQ2 datasheet, to get a precise value, we take 5 points of the graph in a excel archive so we can calculate the slope, then after that, we 
 need to calcule the value of the line or the straight ("Y = mX + B"). 
 NOTE: THE GRAPH IS BASED IN A LOGARITMIC FUNCTION WITH BASE 10
 Here is an example to calculate the slope, the line and using the logaritmic function, we're goint to calcule the smoke ppm values for mq2
 Two points for the slope and its calculation
 (200,1.6) and (10000,0.26) --- transforming it to log(10) ----> (log(200), log(1.6)) y (log(10000), log(0.26)) 
 We apply "m = [log(y) - log(y0)] / [log(x) - log(x0)]", getting -0.44340257 (-0.44). NOTE: IT DEPEDNS HOW YOU TAKE THE POINT OF THE GRAPH
 Then we calculate "b" value of the line, so que take the point (5000,0.91) to calculate "b", to do the next equation
"log(0.36) = -0.47 * log(5000) + b" where "b" value is 1.5991
 */

// Point on every curve, each sensor has their own calibration in their datasheet
float SMOKEcurve[3]  = {2.30, 0.53, -0.44}; 
float LPGcurve[3]    = {2.30, -0.15, -0.37};
float COcurve[3]     = {1.70, 0.24, -0.677};
float NH4curve[3]    = {1.00, 0.46, -0.235};

float ROsmoke = 10;
float ROlpg   = 10;
float ROco    = 10;
float ROnh4   = 10;
// NOTE: first we need to find R0 value to calibrate the sensors because this values needs to be constant

void setup() {
  
 Serial.begin(115200);
 dht.begin();
 pinMode(13,OUTPUT);
 digitalWrite(13,HIGH);
 Serial.print("Calibrating...");

 ROsmoke = calibratingSensor(MQ2, air[0]);
 ROlpg   = calibratingSensor(MQ5, air[1]);
 ROco    = calibratingSensor(MQ7, air[2]);
 ROnh4   = calibratingSensor(MQ135, air[3]);
 digitalWrite(13,LOW);

 //Serial.println("Calibration finished!");
 //Serial.print("RO-SMOKE = ");
 //Serial.print(ROsmoke);
 //Serial.print("RO-lpg = ");
 //Serial.print(ROlpg);
 //Serial.print("RO-co = ");
 //Serial.print(ROco);
 //Serial.print("RO-nh4 = ");
 //Serial.print(ROnh4);
 delay(1000);
}

void loop() {
  long ppm_smoke   = 0;
  long ppm_lpg     = 0;
  long ppm_co      = 0;
  long ppm_nh4     = 0;
  
  ppm_smoke   = obtain_gas_perc(MQlecture(MQ2)/ROsmoke, SMOKE);
  ppm_lpg     = obtain_gas_perc(MQlecture(MQ5)/ROlpg, LPG);
  ppm_co      = obtain_gas_perc(MQlecture(MQ7)/ROco, CO);
  ppm_nh4     = obtain_gas_perc(MQlecture(MQ135)/ROnh4, NH4);

  temp = dht.readTemperature();
  humidity = dht.readHumidity();

  Serial.println("**********         PPM VALUES          **********");
  Serial.print("SMOKE: ");
  Serial.print(ppm_smoke);
  Serial.println(" ppm");
  
  Serial.print("LPG: ");
  Serial.print(ppm_lpg);
  Serial.println(" ppm");
  
  Serial.print("CO: ");
  Serial.print(ppm_co);
  Serial.println(" ppm");

  Serial.print("NH4: ");
  Serial.print(ppm_nh4);
  Serial.println(" ppm");

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print("°C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  delay(1000);
}

// FUNCTIONS
// Get RS value
float calculateRS(int raw){
  return (((float) RL * (1023 - raw) / raw));
}

// Get R0 value
float calibratingSensor(int pin, float sensorAir){
  int i;
  float val=0;
              
  for (i=0; i<100; i++){
    val += calculateRS(analogRead(pin));
    delay(50);
  }
  val = val/100;
  val = val/sensorAir;
  return val;
}

// This function is for calculating the average RS of the first 50 RS values
float MQlecture(int pin){
  int i;
  float rs=0;

  for (i=0; i<50; i++){
    rs += calculateRS(analogRead(pin));
    delay(50);
  }
  rs = rs/50;
  return rs;
}

// This function determines which curves we want to be detected, so we define what the sensor will do and the component that will be read
long obtain_gas_perc(float ratio_rs_ro, int id_gas){
  if (id_gas == SMOKE){
    return obtain_perc_sensor(ratio_rs_ro,SMOKEcurve);
  } else if (id_gas == LPG){
    return obtain_perc_sensor(ratio_rs_ro,LPGcurve);
  } else if (id_gas == CO){
    return obtain_perc_sensor(ratio_rs_ro,COcurve);
  } else if (id_gas == NH4){
    return obtain_perc_sensor(ratio_rs_ro,NH4curve);
  }
  return 0;
}

// This function allows to get the PPM values according the data sheet of each MQ sensor
long obtain_perc_sensor(float ratio_rs_ro, float *pcurve){
  
  return (pow(10,( ((log(ratio_rs_ro)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

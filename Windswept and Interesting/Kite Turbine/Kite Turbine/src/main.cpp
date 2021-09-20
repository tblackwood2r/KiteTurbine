/*
Rod Reads Ground Station control software for Rotary Kite Network Airborne Wind Energy by Windswept and Interesting Ltd

Looking to up the speed of the main loop() by removing print parts etc into object oriented...
still need to do to this and change emergency overdrive into an interrupt and flag change.

A max amount of regen current is decided by dial, 
then the amount actually demanded is decided via addition of 3 calculated quotient parts 
Speed trend, TSR and TTR

(TSR)Tip Speed Ratio dial set point will be compared to rotor speed and wind speed data.

Similarly there's a (TTR) Tension Torque Ratio setpoint using tension readings and regen current data

VESC speed trend data will also be taken into account


Problem with VESC interface software   ---   only works on old VESC firmware ~V 3.8   Latest VESC fw > v 6

  Original VESC UART interface work :Copyright 2016 Tobias Sachs Tobias.Sachs@onlinehome.de

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 created by:  Tobias Sachs 
 messed with by ROD READ for kite ground station controller
 Name:        
 Created:   22/09/2016  messed with again until now
 Author:    TS then Rod R

  by David A. Mellis  <dam@mellis.org>
  modified 9 Apr 2012
  by Tom Igoe
  http://www.arduino.cc/en/Tutorial/Smoothing


//has emergency drive forward override This is going up to 8 amps from 2 - 3 dec 2019  ---  MAKES no difference to VESC SPeed though

//You have to configure the VESC to APP "UART" and set the baudrate to  115200 
// 
//You have to uncommend the line "#define USE_PATCHED_VESC_FW_2_18" in the file VescUartControl.h
// I got rid of the getLimits parts
//
//Could do with getting a more relevant comms set to and from the VESC... I think there are other VESC commands available
//A firmware upgrade for the VESC is available now too...

 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
 Arduino pin 6 -> HX711 CLK
 Arduino pin 5 -> HX711 DOUT
 Arduino pin 5V -> HX711 VCC
 Arduino pin GND -> HX711 GND 
*/

#include <Arduino.h>
#include <VescUartControl.h>

VescController Vesc1;

struct bldcMeasure measuredVESC1;

uint16_t counts = 0;  //unsure... something to do with VESC comms

#include <HX711.h>

float calibration_factor = 10; // this calibration factor is adjusted according to my load cell
float tensionunits = NULL;              // approx = grams

float TSRsetpoint = NULL;
//float TSRactual;
float TSRaveraged = NULL;
float TSRQuota =0.1;

float TTRsetpoint = NULL;
float TTRactual = NULL;
float TTRMaxValue = 20.0;
float TTRQuota =0.1;

float tipspeed = NULL;
float windspeed = 2.0;

float SpeedTrendQuota =0.1;

#define DOUT 5
#define CLK 6

HX711 scale;

//HX711 scale(5, 6);

const int EMbuttonPin = 8;    // the pushbutton pin for emergency drive forward       DOUBLE CHECK YOU HAVE THIS WIRED
int EM_FW_buttonState = 0;    // variable for reading the pushbutton status

const int StartButtonPin = 2; // the pushbutton pin for Brake at start
int StartButtonState = 0;     // variable for reading the Brake at start pushbutton status

int sensorPin1 = A0;          // select the input pin for brakecurrent ramp up average compare setpoint
float sensorValue1TSR = 5.0;  // variable to store the value coming from the sensor

int sensorPin2 = A5;          // select the input pin for the brakecurrent ramp up rate
float sensorValue2TTR = 5.0;  // variable to store the value coming from the sensor

int sensorPin7 = A7;          // select the input pin for the brakecurrent max level
float sensorValue7 = 0.1;     // variable to store the value coming from the sensor

float maxbrakecurrent = 0.5;  // A limit value for the maximum allowable current
int StartBrakeCurrent = 16.0; // The initial hold brake current before release to generate up from 14

const int numReadingsRPM = 4;       // size of array for smoothing rpm readings 
const int numReadingsAvRPM = 15;    // size of long array for smoothing rpm readings 

const int numReadingsTen = 4;       // size of array for smoothing tension readings 
const int numReadingsWind = 3;      // size of array for smoothing windspeed readings 

float readingsRPM[15];  // the rpm array
int readIndexRPM = 0;               // the index of the current rpm reading
float totalRPM = 0.0;               // the running total rpm
float averageRPM = 0.0;             // the average rpm

float readingsAvRPM[15];  // the Long rpm array
int readIndexAvRPM = 0;                 // the index of the current rpm reading
float totalAvRPM = 0.0;                 // the running total rpm
float averageAvRPM = 0.0;               // the average rpm

float readingsTen[15];      // the tension array
int readIndexTen = 0;                   // the index of the current tension reading
float totalTen = 0.0;                   // the running total tension
float averageTens = 0.0;                // the average tension

float readingsWind[15];    // the winspeed array
int readIndexWind = 0;                  // the index of the current windspeed reading
float totalWind = 0.0;                  // the running total windspeed
float averageWind = 0.0;                // the average windpseed

#define DEBUG
unsigned long count;          //not needed? Maybe to do with VESC comms
unsigned long count2;         //not needed? Maybe to do with VESC comms

float turnrate = NULL;        //erpm converted into approx rpm below
float RAD_Per_Sec;            //and then probably miles off a radpersec value

float brakeCurrent = 0.1;    //desired braking level at moment

float TSRstepUP = 0.02;       //amount higher we raise brake current per itteration if TSR is too high
float TSRstepDOWN = 0.2;      //amount higher we lower brake current per itteration if TSR is too low

float TTRstepUP = 0.02;       //amount higher we raise brake current per itteration if TTR is too high
float TTRstepDOWN = 0.2;      //amount higher we lower brake current per itteration if TTR is too low


String inputString = "";      // a String to hold incoming wind data on serial 3
bool stringComplete = false;  // whether the wind data string is complete

float propData[15];

struct avDataReturned{
  int index;
  float avVal;
  float* propDataAddr;
};

/*
  Use thise to initialize a kite object instance with a 
  variable name.
*/
class kiteObj {
  /*
    Declared variables and functions
  */
  public:    
    /* 
      Function to clear all the arrays for smoothing 
    */ 
    void clearArrays(bool useNull = true){
      float clearValue = 0;

      if (useNull){
        float clearValue = NULL;
      }
      memset(_tensSensVals, clearValue, sizeof(_tensSensVals)); // clear tension smooth array   
      memset(_windSensVals, clearValue, sizeof(_windSensVals)); //clear the windspeed smooth array   
      memset(_vescSensVals, clearValue, sizeof(_vescSensVals)); //clear rpm smooth array 
      memset(_vescLongAvVals, clearValue, sizeof(_vescLongAvVals)); //clear longer rpm smooth array 
    }
    
    // --- Gather "measurable values" from readable inputs ---
    /*
      Get wind speed reading from Serial3
    */
    float getWindSpeed(){
      _windspeed = Serial3.parseFloat();
      return _windspeed;
    }

    /* 
      Get the tension by taking 'numReadings' number of readings (default = 1).
      'fabs()' returns the 'absolute value of a float' - the positive magnitude 
      rather than a negative value.
      'calFactor' is the Calibration factor or Number of measurements being taken
    */
    float getTension(float calFactor = 10){
      float tmpTens; 
      scale.set_scale(calFactor); 
      tmpTens = scale.get_units();
      _tension = fabs(tmpTens);
      return _tension;
    }

    /* 
      Takes current connection with the vesc through inVesc and the data recieved through 
      measureVesc and grabs the RPM to convert and return  to 'turnrate', and '_isVescData' 
      is set to 'true' (because there is Vesc data).

      If connection cannot be established then the returned 'turnrate' is set to 'NULL' and 
      the associated value of '_isVescData' is set to false (because there is no Vesc data).
    */
    float getVescData(VescController inVesc, bldcMeasure measureVesc){
      if (inVesc.UartGetValue(measureVesc)) {
        // Vesc is running and setting '_isVescData' to 'true'
        _isVescData = true;
        _turnrate = measureVesc.rpm / (920000*3.8); //20khz x 46 poles  (23 pairs) why 3.8? -fudge factor to get the rpm roughly calibrated (odd because gear ratio is 47:22)
        return _turnrate;
      } else { 
        _isVescData = false;
        return NULL;
      }
    }

    /*
      Read the value from rotary knob sensor 1 (A0) 
      TSR setpoint map limit TSR min to 2.5 top possible setpoint value to ~5.5, 
    */
    float getTSR(){
      _setTSR = (map (analogRead(A0), 0,1023,250,550)) / 100.0; 
      return _setTSR;
    }

    /*
      Read the value from the sensor 2 (A5)
      TTR (kg pull / amps braking) want it to be ~ around ~ 0.7 I guess, max here is 20
    */
    float getTTR(){
      float tmpTTR = (map (analogRead(A5), 0,1023,10,2000)) / 100.0;
        if (tmpTTR > _maxTTR){
          tmpTTR = _maxTTR;
        }
      _setTTR = tmpTTR; 
      return tmpTTR;
    }

    /*
      Read the value from the sensor 7 (A7) for max current
      dial on A7 limiting max current from 0.1 to 13.0
    */
    float getMaxBrakeCurrent(){
      _maxBrakeCurrent = (map (analogRead(A7), 0,1023,1,1300)) / 100.0; 
      return _maxBrakeCurrent;
    }
    
    /* 
      Single call to gather and update input and sensor values
    */
    void getData(VescController inVesc, bldcMeasure measureVesc){
      
      getWindSpeed();
      getTension();

      getVescData(inVesc, measureVesc);
      
      getTSR();
      getTTR();
      getMaxBrakeCurrent();
    }

    /*
      The array to be updated is passed in and using a pointer to point at the addressed 
      for the index value to be used, the array at index value is updated. The index is then
      checked to see if it's at it's designated size, if so it is reset back to 0, if not, it is 
      incremented. 
    */
    void arrayUpdate(float measureVal, float* propData, int* prop_i, int propArrLen = 4){
      // If we're at the last index value reset it, else add one to the index value
      if ((*prop_i >= (propArrLen - 1))){
        // if we're at the end of the array...wrap around to the beginning:
        *prop_i = 0;
      } else {
        *prop_i++;
      }
      propData[*prop_i] = measureVal; // Replace oldest value with newest value 
    }

    /*
      Calculating Average value data.
      Set up function for averages and then run the function relative to what we want to calculate.
      Sum the values in the array and then divide them by the array length.
    */
    float averageData(float* propData, int propArrLen = 4){
      float tmpTotal = 0;                      // Set a temporary total value
      
      for (int i=0; i <= (propArrLen - 1); i++){       
        tmpTotal = tmpTotal + propData[i];     // Sum the the property data
      }

      float propAv = tmpTotal/propArrLen;      // Calculating the average value
         
      delay(1);
      return propAv;
    }

    /*
      This function calculates the average value for each of the sensors and stores the result in a local variable. 
    */
    void getAvVals(){
      arrayUpdate(_windspeed, _windSensVals, &_windspeed_i, _windSensValsLen);
      arrayUpdate(_tension, _tensSensVals, &_tensSens_i, _tensSensValsLen);
      arrayUpdate(_turnrate, _vescSensVals, &_vescSens_i, _vescSensValsLen);
      arrayUpdate(_turnrate, _vescLongAvVals, &_vescLong_i, _vescLongAvValsLen);
      // arrayUpdate(_espRPM_measure, _espRPMVals, &_espRPM_i, _espRPMAvValsLen); 

      _avWind = averageData(_windSensVals, _windSensValsLen);
      _avTens = averageData(_tensSensVals, _tensSensValsLen);
      _vescAvRPM  = averageData(_vescSensVals, _vescSensValsLen);
      _vescLongAvRPM = averageData(_vescLongAvVals, _vescLongAvValsLen); 
      // _avEspRPM = averageData(_espRPMVals, _espRPMAvValsLen);
    }

    // --- Esoteric functions ---
    /*
      Returns the tip speed from the average RPM, 'vescAvRPM' with 
      respect to rotor radius 'rtrTipRadius'.

      The average vesc RPM (vescAvRPM) is multiplied by (2 * pi / 60 = 0.10472)
      to return the radians per  second. This value is multiplied by the 
      rotor tip radius (rtrTipRadius) to give the tip speed. 
    */
    float calcTipspeed(float vescAvRPM = NULL, float rtrTipRadius = 2.25){
      // --- If no value given, get class value ---
      if (vescAvRPM == NULL){
        vescAvRPM = _vescAvRPM;
      }
      // ------------------------------------------

      _tipspeed = (vescAvRPM * 0.10472) * rtrTipRadius; // rtrTipRadius is the rotor tip outer radius, default 2.25m    
      
      return _tipspeed;
    }

    /*
      Returns the average Tip Speed Ratio from the '_tipspeed' divided 
      by the average wind '_avWind'.
    */
    float calcTSR(float tipspeed = NULL, float avWind = NULL){
      // --- If no value given, get class value ---
      if (tipspeed == NULL){
        tipspeed = _tipspeed;
      }
      
      if (avWind == NULL){
        avWind = _avWind;
      }
      // ------------------------------------------

      _avTSR = tipspeed / avWind;
      return _avTSR; 
    }

    /*
      Returns the Tension-Torque Ratio from the (scaled down by 1000) 
      average tension '_avTens' divided by the brake current '_brakeCurrent'.
    */
    float calcTTR(float avTens = NULL, float brakeCurrent = NULL, float maxTTR = 20.0){      
      // --- If no value given, get class value ---
      if (avTens == NULL){
        avTens = _avTens;
      }
      
      if (brakeCurrent == NULL){
        brakeCurrent = _brakeCurrent;
      }
      // ------------------------------------------

      float tmpTTR = (_avTens / 1000.0) / _brakeCurrent;
      if (tmpTTR > maxTTR){
        tmpTTR = maxTTR; 
      }
      _actTTR = tmpTTR;
      return _actTTR;
    }

    /* 
      This is where we list the functions to calculate the values derived from the measurements.

      Values are saved back into the class.
    */
    void calcDerived(){
      calcTipspeed(); // Get the Tip Speed
      calcTSR(); // Get the average TSR
      calcTTR();

      getSysState();
    }

    // --- Quota functions ---
    /* 
      Simple clip function to clip between a lower and a higher value.
      if no value given for lower and upper, 0.1 and 1.0 are used
      respectively
    */
    float clip(float n, float lower = 0.1, float upper = 1.0){
      return max(lower, min(n, upper));
    }

    /* 
      Function to set the quotas for TSR, TTR, and speed.
      Takes in the measured value, the control value, the quota value, and the upper 
      and lower "tuning" values. 
      
      If the actual/average value is found to be 1.1 times the set/control value then 
      the quota is inceased by the upperTuning value. 
      
      If the actual/average value is found to be 0.85 times lower than than the set/control 
      value then the quota is lowered by the lowerTuning value.

      The quota value is clipped to be between 0.1 and 1.0 before being returned. 
    */
    float setQuotaFunc(float actualVal, float controlVal, float quotaVal, float upperTuning = 0.01, float lowerTuning = 0.04, float maxMultiplier = 1.1, float minMultiplier = 0.85){
      if (actualVal > (controlVal * maxMultiplier)){
        quotaVal = (quotaVal + upperTuning);
      }

      if (actualVal < (controlVal * minMultiplier)){
        quotaVal = (quotaVal - lowerTuning);
      }
      quotaVal = clip(quotaVal);
      return quotaVal;
    }

    /*
      Function to make a batch call to set the quotas for TSR, TTR, and speed. 
    */
    void setQuotas(){
      _quotaTSR = setQuotaFunc(_avTSR, _setTSR, _quotaTSR, 0.01, 0.02); // Set the TSR Quota
      _quotaTTR = setQuotaFunc(_actTTR, _setTTR, _quotaTTR);            // Set the TTR Quota
      _quotaSpd = setQuotaFunc(_vescAvRPM, _vescLongAvRPM, _quotaSpd);  // Set the Speed Quota
    }

    /*
      Return the Tip Speed Ratio (TSR) Quota based on the conditions between the 
      calculated average value of TSR and the setTSR.
    */
    float setTSRQuota(float avTSR = NULL, float setTSR = NULL, float quotaTSR = NULL){
      // --- If no value given, get class value ---
      if (avTSR == NULL){
        avTSR = _avTSR;
      }

      if (setTSR == NULL){
        setTSR = _setTSR;
      }

      if (quotaTSR == NULL){
        quotaTSR = _quotaTSR;
      }
      // ------------------------------------------
      
      if (avTSR > (setTSR * 1.1)){
        quotaTSR = (quotaTSR + 0.01);
      }
      
      if (_avTSR < (_setTSR * 0.85)){
        quotaTSR = (quotaTSR - 0.02);
      }
      
      quotaTSR = clip(quotaTSR);
      _quotaTSR = quotaTSR; // Set class value
      return quotaTSR;
    }

    /*
      Return the Tension Torque Ratio (TTR) Quota based on the conditions between the 
      calculated actual TTR value and the setTTR value.
    */
    float setTTRQuota(float actTTR = NULL, float setTTR = NULL, float quotaTTR = NULL){      
      // --- If no value given, get class value ---
      if (actTTR == NULL){
        actTTR = _actTTR;
      }

      if (setTTR == NULL){
        setTTR = _setTTR;
      }
      
      if (quotaTTR == NULL){
        quotaTTR = _quotaTTR;
      } 
      // ------------------------------------------

      if (actTTR > (setTTR * 1.1)){
        quotaTTR = (quotaTTR + 0.01);
      }
      
      if (actTTR < (setTTR * 0.85)){
        quotaTTR = (quotaTTR - 0.04);
      }

      quotaTTR = clip(quotaTTR);
      _quotaTTR = quotaTTR; // Set class value
      return quotaTTR;
    }

    /*
      Return the Speed Quota based on the conditions between the 
      short RPM average and the long RPM average.
    */
    float setSpdQuota(float vescAvRPM = NULL, float vescLongAvRPM = NULL, float quotaSpd = NULL){      
      // --- If no value given, get class value --- 
      if (vescAvRPM == NULL){
        vescAvRPM = _vescAvRPM;
      } 

      if (vescLongAvRPM == NULL){
        vescLongAvRPM = _vescLongAvRPM;
      }

      if (quotaSpd == NULL){
        quotaSpd = _quotaSpd;
      }
      // ------------------------------------------

      if (vescAvRPM > (vescLongAvRPM * 1.1)){
        quotaSpd = (quotaSpd + 0.01);
      }
      
      if (vescAvRPM < (vescLongAvRPM * 0.85)){
        quotaSpd = (quotaSpd - 0.04);
      }
      
      quotaSpd = clip(quotaSpd);
      _quotaSpd = quotaSpd; // Set class value
      return quotaSpd;
    }

    /*
      Single call for the class to get all quotas and store the values 
      internally to the class. 
    */
    void getQuotas(){
      setTSRQuota();
      setTTRQuota();
      setSpdQuota();
    }
    
    /*
      Set the brake current '_brakeCurrent' based on the max brake current '_maxBrakeCurrent', 
      speed quota '_quotaSpd', tension torque ratio '_quotaTTR', and tip speed ratio '_TSRQuota'.
    */
    float calcBrakeCurrent(float maxBrakeCurrent = NULL, float quotaSpd = NULL, float quotaTTR = NULL, float quotaTSR = NULL){
      // --- If no value given, get class value --- 
      if (maxBrakeCurrent == NULL){
        maxBrakeCurrent = _maxBrakeCurrent;
      }

      if (quotaSpd == NULL){
        quotaSpd = _quotaSpd;
      }

      if (quotaTTR == NULL){
        quotaTTR = _quotaTTR;
      }

      if (quotaTSR == NULL){
        quotaTSR = _quotaTSR;
      }
      // ------------------------------------------

      float brakeCurrent = maxBrakeCurrent * ((quotaSpd * 0.3) + (quotaTTR * 0.2) + (quotaTSR * 0.5));

      brakeCurrent = clip(brakeCurrent, 0.15, maxBrakeCurrent);
      _brakeCurrent = brakeCurrent; // Set class value
      return _brakeCurrent;
    }

    // --- Prepping for output functions ---
    /*
      Return a string with the state of the system dependent on the values
      current values of the system. 

      States:
      1. "Start Brake On"
      2. "Emergency FW: Software"
      3. "Too Slow to Gen"
      4. "Weak Lift Tension"
      5. "OK to Gen"
      6. "No Vesc Data"
      7. "Emergency FW: Button"
    */
    String getSysState(int emFWButton = NULL, int strtButton = NULL, bool isVescData = NULL, float vescAvRPM = NULL, float avTens = NULL, bool returnText = false){
      if (emFWButton == NULL){
        emFWButton = _emFWButton;
      }
      if (strtButton == NULL){
        strtButton = _strtButton;
      }
      if (isVescData == NULL){
        isVescData = _isVescData;
      }
      if (vescAvRPM == NULL){
        vescAvRPM = _vescAvRPM;
      }
      if (avTens == NULL){
        avTens = _avTens;
      }

      if (emFWButton == HIGH){
      // Check if emergency forward button is pressed (LOW)

        if (strtButton == HIGH){
        // Check if the Start Button has been pressed (LOW)
          _sys_state_t = "Start Brake On";
          _sys_state = 1;
        } else {
        // If the start button has been pressed...
          
          if (isVescData){
          // If Vesc data is available...     
            if (vescAvRPM < 65.0){
            // If the RPM is lower than 65...
              if (avTens > 50000){
              // if the Tension is greater than 50,000 then the software should initialize emergency forward.
                _sys_state_t = "Emergency FW: Software";
                _sys_state = 2;
              } else {
              // otherwise the wind is too slow to generate.
                _sys_state_t = "Too Slow to Gen";
                _sys_state = 3;
              }  
            } else {
            // If the RPM is good...
              if (avTens < 1800){
              // If the tension is too low signal that there's weak lift
                _sys_state_t = "Weak Lift Tension";
                _sys_state = 4;
              }
              if (avTens > 1500){
              // Otherwise System is okay to generate!
                _sys_state_t = "OK to Gen";
                _sys_state = 5;
              }
            }
          } else {
          // Otherwise the Vesc isn't talking
            _sys_state_t = "No Vesc Data";
            _sys_state = 6;
          }
        }
      } else {
      // Otherwise the Emergency Forward button has been pressed
        _sys_state_t = "Emergency FW: Button";
        _sys_state = 7;
      }
      if (returnText) {
        return _sys_state_t;
      } else {
        return String(_sys_state);      
      }
    }

    /*
      Calling this funciton prints the data to the serial connection.
      Data seperated by tabs and commas
    */
    void printDataToSerial(){  
      Serial.println(
        "State, " + _sys_state_t +               // print state
        ",\tWind, " + String(_windspeed) +       // Wind Speed
      
        ",\tRPM, " + String(_turnrate) +         // RPM
      
        ",\tgenA, " + String(_brakeCurrent) +    // Brake Current
        ",\tAmax, " + String(_maxBrakeCurrent) + // Max Brake Current
      
        ",\tTipSpeed, " + String(_tipspeed) +    // Tip Speed
        ",\tTSR, " + String(_avTSR) +            // TSR Average
        ",\tTSRset, " + String(_setTSR) +        // TSR Set-point
      
        ",\tTen-g, " + String(_tension/1000) +   // Tension (converted to kg here)
        ",\tTTR, " + String(_actTTR) +           // TTR Actual
        ",\tTTRset, " + String(_setTTR) +        // TTR Set-point

        ",\tSpdQ, " + String(_quotaSpd) +         // Speed Trend Quota
        ",\tTSRQ, " + String(_quotaTSR) +         // TSR Quota
        ",\tTTRQ, " + String(_quotaTTR) +         // TTR Quota
        ","); 
      delay(200);
    }

    /*
      Calling this funciton prints the raw data to the serial connection
      These are only numerics seperated by commas. Similar to 
      `printDataToSerial()` but without tags.
    */
    void printRawData(){  
      Serial.println(
        String(_sys_state) + "," +        // print state
        
        String(_windspeed) + "," +        // Wind Speed
        String(_turnrate) + "," +         // RPM        
        String(_tipspeed) + "," +         // Tip Speed
        String(_tension/1000) + "," +     // Tension (converted to kg here)

        String(_brakeCurrent) + "," +     // Brake Current
        String(_maxBrakeCurrent) + "," +  // Max Brake Current

        String(_avTSR) + "," +            // TSR Average
        String(_setTSR) + "," +           // TSR Set-point
        
        String(_actTTR) + "," +           // TTR Actual
        String(_setTTR) + "," +           // TTR Set-point
        
        String(_quotaSpd) + "," +         // Speed Trend Quota 
        String(_quotaTSR) + "," +         // TSR Quota 
        String(_quotaTTR) + ","           // TTR Quota 
      );
      delay(200);
    }

  private:
    int _emFWButton = digitalRead(8); // Check if emergency forward button is pressed (LOW)
    int _strtButton = digitalRead(2); // Check if the Start Button has been pressed (LOW)
    
    // Rod's unsure about setting all array sizes to 4 as changing the array 
    // size affects the average value and stability of the system

    float _windspeed;                 // Actual windspeed
    int _windspeed_i = 0;             // Windspeed index Value
    const int _windSensValsLen = 3;   // Number of windspeed values to be recorded 
    float _windSensVals[15];          // Array for windspeed values
    float _avWind;                    // Average windspeed

    float _tension;                   // Actual Tension
    int _tensSens_i = 0;              // Tension index value
    const int _tensSensValsLen = 4;   // Number of tension load-cell readings to take for average
    float _tensSensVals[15];          // Array for tension load-cell readings
    float _avTens;                    // Average Tension

    float _brakeCurrent;              // Braking current/current generated
    float _maxBrakeCurrent = 0.5;     // Maximum set operating brake Current from dial
    
    bool _isVescData = false;         // Boolean to declare if Vesc is connected
    float _turnrate;                  // Actual RPM from Vesc
    
    int _vescSens_i = 0;              // Short Vesc index value
    const int _vescSensValsLen = 4;   // Number of sensor readings to be recorded for short Vesc average
    float _vescSensVals[15];          // Array for short Vesc sensor values
    float _vescAvRPM;                 // Average short Vesc RPM
    
    int _vescLong_i = 0;              // Long Vesc index value
    const int _vescLongAvValsLen = 15;// Number of sensor readings to be recorded for a long Vesc average
    float _vescLongAvVals[15];        // Array for long Vesc sensor values
    float _vescLongAvRPM;             // Average long Vesc RPM
     
    float _tipspeed;                  // Actual Tip Speed
    float _avTSR;                     // TSR calculated from (tipspeed / average windspeed)
    float _setTSR;                    // Set TSR read from dial
    
    const float _maxTTR = 20;         // Maximum Tension Torque Ratio
    float _actTTR;                    // Actual Tension Torque Ratio
    float _setTTR;                    // Set Tension Torque Ratio from dial
    
    float _quotaSpd = 0.1;            // Speed Quota
    float _quotaTSR = 0.1;            // Tip Speed Ratio Quota
    float _quotaTTR = 0.1;            // Tension Torque Ratio Quota

    float _sys_state;                 // Numerical system state
    String _sys_state_t;              // Text system state
    
};



// Referencing the class object above and declaring it under the name kiteData.
kiteObj kiteData;


//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup()
{
  Serial.setTimeout(5);
  
  pinMode(StartButtonPin, INPUT);  // initialize the pushbutton pin as an input: 
  
  Serial.begin(115200); //begin Serial Console
  delay(100);
  Serial2.begin(115200); //begin VESC Serial Console
  delay(200);
  Serial3.begin(115200); //begin wind data Serial port
  delay(100);

  // reserve 200 bytes for the inputString: wind data
  inputString.reserve(200);

  Vesc1.begin(&Serial2);
  Vesc1.UartSetCurrent(0.0);
  Vesc1.UartSetCurrentBrake(0.02);
  delay(50);
 

  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare();  //Reset the scale to 0
  delay(30);

  long zero_factor = scale.read_average(5); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
  delay(30);

  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  delay(30);

  //clear rpm smooth array 
  memset(readingsRPM, NULL, sizeof(readingsRPM));
  
  //clear longer rpm smooth array 
  memset(readingsAvRPM, NULL, sizeof(readingsAvRPM));

  // clear tension smooth array   
  memset(readingsTen, NULL, sizeof(readingsTen));

  //clear the windspeed smooth array    
  memset(readingsWind, NULL, sizeof(readingsWind));
  
  // This only resets memory in the class.
  kiteData.clearArrays();

}

//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
     
void loop(){
  String sys_state = "";
  int run_delay = 0;
  
  // ---- Get wind speed ----
  windspeed = kiteData.getWindSpeed(); // Get Wind Speed reading     
  // ---- Get average Tension ----
  tensionunits = kiteData.getTension();

  // check if the emergency drive forward pushbutton is pressed. If pressed the buttonState is LOW: end of loop routine (long loop = really hard to read program sorry)
  // Button wired on 2021/08/24 to have HIGH state when pressed
  while (digitalRead(EMbuttonPin) == LOW){   
    // check if the StartBrakeCurrent pushbutton is pressed. If it is, the buttonState is HIGH:
    if (digitalRead(StartButtonPin) == HIGH){
      // It's startup routine ... brakes high... Not time for spin up & regen yet
      Vesc1.UartSetCurrentBrake(StartBrakeCurrent);   

      //Serial.print("erpm_VESC1: "); Serial.print(",");
      sys_state = "Startup Brake on";
      run_delay = 200;
    } else {
      // System has started-up, get load cell readings.           
      // ---------------------------- Calcualte average wind ---------------------------
      // Lets find the average windspeed
      kiteData.arrayUpdate(windspeed, readingsWind, &readIndexWind, numReadingsWind);
      averageWind = kiteData.averageData(readingsWind, numReadingsWind);
      
      kiteData.arrayUpdate(tensionunits, readingsTen, &readIndexTen, numReadingsTen);
      averageTens = kiteData.averageData(readingsTen, numReadingsTen);
      //not worrying about including tension calibration now lets get knob set points
      // -------------------------------------------------------------------------------
      
      // read the value from rotary knob sensor 1:
      //TSR setpoint map limit TSR min to 2.5 top possible setpoint value to ~5.5, 
      TSRsetpoint = kiteData.getTSR();                                                             
      delay (2);
      // read the value from the sensor 2:
      // TTR (kg pull / amps braking) want it to be ~ around ~ 0.7 I guess, max here is 20
      TTRsetpoint = kiteData.getTTR();                                                                  
      delay (2);
      // read the value from the sensor 7: for max current
      // dial on A7 limiting max current from 0.1 to 13.0
      maxbrakecurrent = kiteData.getMaxBrakeCurrent();
      delay (2);

      turnrate = kiteData.getVescData(Vesc1, measuredVESC1);
      // ---------------------------- *Look at this* Calcualte RPM ---------
      if (Vesc1.UartGetValue(measuredVESC1)) {
      
        float voltage = measuredVESC1.inpVoltage;
        float ampHrChrg = measuredVESC1.ampHoursCharged;

        turnrate = (measuredVESC1.rpm / (920000*3.8)); 
        // 20khz x 46 poles  (23 pairs) why 3.8? -fudge factor to get the rpm roughly calibrated 
        // (odd because gear ratio is 47:22)

        // Lets find the short average turn rate
        kiteData.arrayUpdate(turnrate, readingsRPM, &readIndexRPM, numReadingsRPM);
        averageRPM = kiteData.averageData(readingsRPM, numReadingsRPM);
        
        // and a longer average turn rate
        kiteData.arrayUpdate(turnrate, readingsAvRPM, &readIndexAvRPM, numReadingsAvRPM);
        averageAvRPM = kiteData.averageData(readingsAvRPM, numReadingsAvRPM);
        
        // -------------------------------------------------------------------------------
                                                
        //Tip Speed Ratio
        tipspeed = kiteData.calcTipspeed(averageRPM);

        TSRaveraged = kiteData.calcTSR(tipspeed, averageWind); // tipspeed uses rpm average ... averageWind is windspeed av    
              
        TTRactual = kiteData.calcTTR(averageTens, brakeCurrent, TTRMaxValue); // Calculate the Torque Tension Ratio, max value is 20
                        
        if (averageRPM < 65.0){
          //slow states check
          
          if (averageTens > 50000){    
            //Worried this could be runaway overtwist because turn rate is low tension is high
            Vesc1.UartSetCurrentBrake(0.0);   //No braking
            Vesc1.UartSetCurrent(8.0);        //unwind for a while
            
            // WARNING THIS MAY KICKSTART A ROTOR IN 'READY TO GEN' Condition with strong lift tension
            sys_state = "Emergency forward software"; 
            delay(200); //delay(200);  
            Vesc1.UartSetCurrent(0.0);
            Vesc1.UartSetCurrentBrake(0.02);  //Start from very low again
          } else {
            brakeCurrent = 0.02; // turn rate is too slow now lets not try braking
            sys_state = "Too Slow 4 gen";
            delay(1);  
          }
        } else { 
          //fast enough
          if (averageTens < 1800){  
            //a 1.8kg minimum pull on the line (because readings are gippy but the linkage is gippy at best)
            brakeCurrent = 0.1;
            sys_state = "Weak lift tension";
          }

          if (averageTens > 1500){
            //TSR first
            TSRQuota = kiteData.setQuotaFunc(TSRaveraged, TSRsetpoint, TSRQuota, 0.01, 0.02);
            
            // TTR next
            TTRQuota = kiteData.setQuotaFunc(TTRactual, TTRsetpoint, TTRQuota);
            
            // Speed Quota
            SpeedTrendQuota = kiteData.setQuotaFunc(averageRPM, averageAvRPM, SpeedTrendQuota);
            
            // ammount contribution of max brake allowed from each component .3x spd trend + .2x TTR + .5x TSR
            brakeCurrent = kiteData.calcBrakeCurrent(maxbrakecurrent, SpeedTrendQuota, TTRQuota, TSRQuota);                              
            sys_state = "OK to Gen";
          }  
          Vesc1.UartSetCurrentBrake(brakeCurrent);  
        }                    
      } else {
        sys_state = "No data from VESC1!"; 
      }             
    }
    kiteData.printDataToSerial(); 
    kiteData.printRawData();
  }
  brakeCurrent = 0.0;
  Vesc1.UartSetCurrentBrake(brakeCurrent); // drive forward button pressed
  Vesc1.UartSetCurrent(16.0);
  
  TTRQuota = 0.1;
  SpeedTrendQuota = 0.1;
  TSRQuota = 0.1;
  
  sys_state = "Emergency forward button pressed";
  run_delay = 150; 
  
  kiteData.printDataToSerial();
  
  turnrate = NULL;
  tipspeed = NULL; 
  TSRaveraged = NULL; 
  TSRsetpoint = NULL;
  brakeCurrent = 0.1;
  TTRactual = NULL; 
  TTRsetpoint = NULL;
  maxbrakecurrent = 0.5;
  SpeedTrendQuota = 0.1; 
  TTRQuota = 0.1; 
  TSRQuota = 0.1;
  
  run_delay = 0;
}

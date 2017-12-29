/* nano-cc-el.ino

    Nano V3 based Constant Current Electronic Load
    10V full scale voltage measurement

*/


#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/////////////////////////////////////////////////
#define SWR_VERSION "ver 0.5.0"
#define WELCOME_MSG "ENP252 CC E-LOAD"


/////////////////////////////////////////////////
//// SYSTEM CONSTANTS
//
// POWER SUPPLY
#define DAC_NOMINAL_VCC_MV  4830  // when powered from USB, 4830, else 5000
//
// PWM DAC
#define DAC_RSENSE_CONDUCTANCE 10 // 0.1 Ohm sense resistor
#define DAC_NOMINAL_LSB_MV  (DAC_NOMINAL_VCC_MV / 1101 / 255)
#define DAC_NOMINAL_MSB_MV  (DAC_NOMINAL_VCC_MV / 11 / 255 )
#define DAC_NOMINAL_MSB_MA  (DAC_NOMINAL_MSB_MV * DAC_RSENSE_CONDUCTANCE)
#define DAC_NOMINAL_LSB_MA  (DAC_NOMINAL_LSB_MV * DAC_RSENSE_CONDUCTANCE)
//
// ADC acquisition
#define ADC_VREF_MV 2500
#define ADC_VDUT_ATTENUATION    4 //[V/V]
#define ADC_IDUT_GAIN           1 //[A/V]
#define ADC_VIREF_ATTENUATION   1 //[V/V]
#define ADC_WINDOW_AVG_WEIGHT   16
#define ADC_NOMINAL_LSB_MV      (ADC_VREF_MV/1023)
#define ADC_NOMINAL_LSB_MA      (ADC_NOMINAL_LSB_MV/ADC_IDUT_GAIN)
//
// ASCII User Interface constants
#define UI_DELIMITERS " ,="
#define UI_INPUT_BUFFER_SIZE 30
#define UI_MAX_TOKENS   (2*10)
//

/////////////////////////////////////////////////
//// HARDWARE IO DEFINITIONS
#define PIN_DAC_LSB 10
#define PIN_DAC_MSB 9
#define PIN_PS_PWM 3
#define PIN_EN_VIN_FET 5
//
#define PIN_LED_GRN 4
#define PIN_LED_RED_13  13
//
#define PIN_SW_S2  8 // currently used by OLED RST
#define PIN_SW_CTR A6
#define PIN_SW_UP 6
#define PIN_SW_DN 7
#define PIN_SW_RT A3
#define PIN_SW_LT A7

#define PIN_ADC_IDUT  A0
#define PIN_ADC_VDUT  A1
#define PIN_ADC_V_IREF A2
//
// OLED RST PIN. - we actually don't have one...
// wire a 10uF ceramic SMD cap between the RST pinMode
// and GND pin of the OLED module. This will cause the
// the RST pin to linger LOW for a period of time after
// power-up, and seems to work.
#define OLED_RESET 4
//
// There is a J1 SPI connector that is currently not used.
// Pins associated with it are:
#define PIN_SPI_MISO    12
#define PIN_SPI_MOSI    11
#define PIN_SPI_SCK     13
// and pin D8, which is shared with PIN_SW_S2
//
// There is also a J2 I2C connector that is currently not used
// However, no Defines are required as Arduino already knows
// about SDA and SCL being hardwired to A4 and A5.
// The OLED display used SDA and SCL, but not via J2.
//

/////////////////////////////////////////////////
// GLOBAL DATA DECLARATIONS:
//
// the OLED display:
// https://github.com/adafruit/Adafruit_SSD1306/blob/master/Adafruit_SSD1306.h
Adafruit_SSD1306 display(OLED_RESET);
//
// User Input data:
char input_buffer[UI_INPUT_BUFFER_SIZE + 1];
char num_bytes;
char * pch;
char * tokens[UI_MAX_TOKENS];
//
// system timer data:
unsigned int ticks_1000ms;
unsigned int ticks_100ms;
unsigned int ticks_10ms;
unsigned int ticks;
//
// adc acquisition
unsigned int adc_nVDDxW;
unsigned int adc_nVDUTxW;
unsigned int adc_nIDUTxW;
unsigned int adc_nVIREFxW;

//modifications
unsigned int userCurrent;
unsigned int dac_pwm_msb;
unsigned int dac_pwm_lsb = 128;

bool isTesting = false;
bool en_status;


//-----------------------------------------------------------------------------
void setup() {

  // setup Serial comm
  Serial.begin(115200);
  Serial.setTimeout(200);
  Serial.println(F(WELCOME_MSG));
  Serial.println(F(SWR_VERSION));

  //
  // configure ADC to use external ADC_VREF_MV Ref
  analogReference(EXTERNAL);
  //
  // init the io pins:
  io_init();
  //
  // init the oled display:
  oled_init();
  //
  // init the charge-pump bi-polar power supplies:
  ps_init();  // init the ps charge pumps
  //


  dac_setIrefMa(0);
  dac_off();

}

/////////////////////////////////////////////////
// ADC functions
/////////////////////////////////////////////////
//
// data:
unsigned int adc_viref_mv, adc_vdut_mv, adc_idut_ma;
unsigned int adc_viref_os, adc_vdut_os, adc_idut_os;
//

void adc_service( void ) {
  static unsigned int nv_iref;
  static unsigned int nv_dut;
  static unsigned int ni_dut;
  unsigned int temp;

  // use pin 13 to probe the frequency at which ADC service is running.
  //digitalWrite(PIN_LED_RED_13,HIGH);

  adc_wavg((unsigned int)analogRead(PIN_ADC_V_IREF), &nv_iref);
  adc_wavg((unsigned int)analogRead(PIN_ADC_VDUT), &nv_dut);
  adc_wavg((unsigned int)analogRead(PIN_ADC_IDUT), &ni_dut);

  /* Serial.println("--N*ADC--");
    Serial.println(nv_iref);
    Serial.println(nv_dut);
    Serial.println(ni_dut);

    Serial.println("--raw ADC--");
    Serial.println(analogRead(PIN_ADC_V_IREF));
    //Serial.println(100);
    Serial.println(analogRead(PIN_ADC_VDUT));
    Serial.println(analogRead(PIN_ADC_IDUT)); */

  //Serial.println("--SI--");
  temp = (((unsigned long)nv_iref / ADC_WINDOW_AVG_WEIGHT) * ADC_VREF_MV * ADC_VIREF_ATTENUATION) / 1023;
  //Serial.println(temp);
  adc_viref_mv = (unsigned int)temp;
  temp = (((unsigned long)nv_dut / ADC_WINDOW_AVG_WEIGHT) * ADC_VREF_MV * ADC_VDUT_ATTENUATION) / 1023;
  //Serial.println(temp);
  adc_vdut_mv = (unsigned int) temp;
  temp = (((unsigned long)ni_dut / ADC_WINDOW_AVG_WEIGHT) * ADC_VREF_MV / ADC_IDUT_GAIN) / 1023;
  //Serial.println(temp);
  adc_idut_ma = (unsigned int) temp;
  //Serial.println(""); Serial.println("");

  /* Serial.println("--SI--");
    Serial.println(adc_viref_mv);
    Serial.println(adc_vdut_mv);
    Serial.println(adc_idut_ma); */

  //digitalWrite(PIN_LED_RED_13, LOW);
  
  // Alex Wardlow's Code
  int change = (userCurrent - adc_idut_ma);
  if( en_status ){ 
    if (change > 3) {
      dac_increment_lsb();
      if (dac_get_lsb() > 255) {
        dac_decrement_lsb();
        dac_increment_msb();
      }
    }
    else if (change < -3) {
      if (dac_get_lsb() != 0) {
        dac_decrement_lsb();
      }
      else if (dac_get_msb() != 0) {
        dac_decrement_msb();
      }
    }
  }
}

/* adc_tare(void){


  } */


unsigned int adc_getVIREF_mV(void) {
  return adc_viref_mv;
}

unsigned int adc_getVDUT_mV(void) {
  return adc_vdut_mv;
}

unsigned int adc_getIDUT_mA(void) {
  return adc_idut_ma;
}

//-----------------------------------------------------------------------------
unsigned int adc_wavg(unsigned int x, unsigned int *yw) {
  unsigned int y;

  // 1st order digital IIR filter
  //
  /*  Implementing an 1st order window averaging filter
      of the form: y[n] = ( 1 * x[n] + (WEIGHT - 1) * y[n-1] )/WEIGHT
      where WEIGHT is an integer (preferably a power of 2 int).
      Actual integer based implementation is:
   	  yw[k] = x[k] + yw[k-1] - yw[k-1]/WEIGHT;
        y[k] = yw[k] / WEIGHT;
      yw is the stored state, and must be able to hold
  	the product WEIGHT * X_FULL_SCALE, e.g. 50,000 * 1023,
      or, if yw is unsigned long, the MAX_WEIGHT = (2^8)^4 / 1023 = 4.198M
  */

  if (ADC_WINDOW_AVG_WEIGHT) {
    *yw = x + *yw - *yw / ADC_WINDOW_AVG_WEIGHT;
    y = *yw / ADC_WINDOW_AVG_WEIGHT;
  }
  // if weight is 0
  else {
    *yw = x;
    y = *yw;
  }

  return y;

}

/////////////////////////////////////////////////
// DAC functions
/////////////////////////////////////////////////
//
// data:
unsigned int dac_niref;
byte         dac_pwm_msb_offset;
unsigned int dac_mv;
byte         dac_tune_viref_state;
byte         dac_tune_iref_state;

// LOTS OF MAGIC NUMBERS TO REMOVE HERE...!
void dac_setIrefMa(unsigned int ma) {
  dac_mv = ma / DAC_RSENSE_CONDUCTANCE;
  if ((127 * (unsigned long)dac_mv) / 53 < (128 / 25))
    dac_pwm_msb = 0;
  else
    dac_pwm_msb = ((127 * (unsigned long)dac_mv) / 53 - (128 / 25)) / 4;

  analogWrite(PIN_DAC_LSB, dac_pwm_lsb);

  if (dac_pwm_msb <= 255 - dac_pwm_msb_offset)
    analogWrite(PIN_DAC_MSB, dac_pwm_msb + dac_pwm_msb_offset);

}

// call as often as adc_service is called.
// returns:
//   0 if not tuned yet
//   1 if tuned
//   -1 if in between tunings.
//
char dac_tune_viref() {
  static unsigned int counter;
  // need 4 * WEIGHT minimum for 2% settling time, use 5 for safety...
  // Nsettling2% = 4/(T*alpha) where alpha = -1/T * ln ( (Wt-1)/Wt )
  // As Wt grows, Nsettling2% approaches 4.  At Wt = 4, Nsettling2% = 3.48
  // At Wt = 64,, Nsettling2% = 3.97.  So just use 5
#define DAC_TUNE_DECIMATION_COUNT   (5 * ADC_WINDOW_AVG_WEIGHT)

  int viref_error_mv = dac_mv - adc_viref_mv;


  byte err_threshold = DAC_NOMINAL_MSB_MV * 6;
  if (dac_mv > 100)
    err_threshold = DAC_NOMINAL_MSB_MV * (6 + dac_mv / 16);

  if (abs(viref_error_mv) < err_threshold && dac_tune_viref_state == 1) {
    //digitalWrite(PIN_LED_GRN,HIGH);
    return 1;
  }

  //digitalWrite(PIN_LED_GRN,LOW);

  if (++counter < DAC_TUNE_DECIMATION_COUNT)
    return -1;

  // code execution reaches this point only every DAC_TUNE_DECIMATION_COUNT'th
  // call to this function.
  // This is necessary to ensure that the dac tune does not run any faster
  // than the window averaging. i.e. it takes time for the window averaging
  // adc measurements to settle out, and the dac tune should only respond
  // to steady state adc readings.

  counter = 0;

  if (abs(viref_error_mv) > 4 * DAC_NOMINAL_MSB_MV) {
    dac_tune_viref_state = 0;
  } else if (dac_tune_viref_state) {
    return dac_tune_viref_state;
  }

  // for debug only:
  Serial.print("er_mv=");
  Serial.println(viref_error_mv);

  if (viref_error_mv > 2 * DAC_NOMINAL_MSB_MV && dac_pwm_msb < 255) {
    dac_pwm_msb++;
  } else if (viref_error_mv < -2 * DAC_NOMINAL_MSB_MV && dac_pwm_msb > 1) {
    dac_pwm_msb--;
  } else {
    dac_tune_viref_state = 1;
  }
  dac_update_msb();
  return dac_tune_viref_state;

}


// Autotuning for iref.
//
char dac_tune_iref() {
  static unsigned int counter;

  // need to tune viref first
  if (!dac_tune_viref_state)
    return 0;

  //#define DAC_TUNE_DECIMATION_COUNT   (5 * ADC_WINDOW_AVG_WEIGHT)
  // DAC_TUNE_DECIMATION_COUNT is defined in dac_tune_vref() but
  // used here as well.  See explanation there...

  int iref_error_ma = dac_mv * DAC_RSENSE_CONDUCTANCE - adc_idut_ma;

  // adc-ma-lsb is 2.048 mA assuming a 2.048V AVREF.
  // if 2.50V reference, then 2.5mA precision.
  //
  // dac-ma-lsb is determined by controlling PWM_LSB duty
  // and is nominally 0.178mA (assuming VCC = 5000mV, 2.048Vref)
  //
  // dac-ma-msb is nominally 17.8mA (assuming VCC=5000mV, 2.048Vref)
  //
  // Thus, assuming the nominal LSB PWM duty of 128/255, one can tune
  // iref by +/- 18mA by adjusting LSB PWM duty between ~101+128 and
  // ~128-101, or 229 to 27.
  //
  // This routine adjusts the PWM LSB Duty up or down until the
  // current error (iref_error_ma) is less than 2 times the adc-ma-lsb,
  // or nominally 4ma.
  //


  if (abs(iref_error_ma) < 4 * ADC_NOMINAL_LSB_MA &&
      dac_tune_iref_state == 1)
    return 1;

  if (++counter < DAC_TUNE_DECIMATION_COUNT)
    return -1;

  // code execution reaches this point only every DAC_TUNE_DECIMATION_COUNT'th
  // call to this function.
  // This is necessary to ensure that the dac tune does not run any faster
  // than the window averaging. i.e. it takes time for the window averaging
  // adc measurements to settle out, and the dac tune should only respond
  // to steady state adc readings.

  counter = 0;

  if (abs(iref_error_ma) > 4 * ADC_NOMINAL_LSB_MA) {
    dac_tune_iref_state = 0;
  } else if (dac_tune_iref_state) {
    return dac_tune_iref_state;
  }

  Serial.print("er_ma="); Serial.println(iref_error_ma);

  if (iref_error_ma > 2 * ADC_NOMINAL_LSB_MA && dac_pwm_lsb < (255 - 8)) {
    dac_pwm_lsb = dac_pwm_lsb + 8;
  } else if (iref_error_ma < -2 * ADC_NOMINAL_LSB_MA && dac_pwm_lsb > (8)) {
    dac_pwm_lsb = dac_pwm_lsb - 8;
  } else {
    dac_tune_iref_state = 1;
  }

  dac_update_lsb();

  return dac_tune_iref_state;


}
////////////////////////////
//Battery Status Functions//
//Scotti Bozarth          //
//Josh Clement            //
///////////////////////////

int batStat(int avgVolt) { 
  int percentageArray[101];
  //200ma load Array decleration
  percentageArray[0]   = 1468;  percentageArray[1]  = 1440; percentageArray[2]  = 1412; percentageArray[3]  = 1390  ;
  percentageArray[4]   = 1372;  percentageArray[5]  = 1358; percentageArray[6]  = 1344; percentageArray[7]  = 1331  ;
  percentageArray[8]   = 1320;  percentageArray[9]  = 1308; percentageArray[10] = 1297; percentageArray[11] = 1290  ;
  percentageArray[12]  = 1282;  percentageArray[13] = 1274; percentageArray[14] = 1268; percentageArray[15] = 1262  ;
  percentageArray[16]  = 1256;  percentageArray[17] = 1251; percentageArray[18] = 1247; percentageArray[19] = 1242  ;
  percentageArray[20]  = 1236;  percentageArray[21] = 1231; percentageArray[22] = 1228; percentageArray[23] = 1222  ;
  percentageArray[24]  = 1218;  percentageArray[25] = 1215; percentageArray[26] = 1211; percentageArray[27] = 1206  ;
  percentageArray[28]  = 1203;  percentageArray[29] = 1198; percentageArray[30] = 1194; percentageArray[31] = 1191  ;
  percentageArray[32]  = 1188;  percentageArray[33] = 1184; percentageArray[34] = 1182; percentageArray[35] = 1180  ;
  percentageArray[36]  = 1178;  percentageArray[37] = 1174; percentageArray[38] = 1172; percentageArray[39] = 1171  ;
  percentageArray[40]  = 1167;  percentageArray[41] = 1165; percentageArray[42] = 1164; percentageArray[43] = 1161  ;
  percentageArray[44]  = 1157;  percentageArray[45] = 1155; percentageArray[46] = 1152; percentageArray[47] = 1149  ;
  percentageArray[48]  = 1147;  percentageArray[49] = 1145; percentageArray[50] = 1142; percentageArray[51] = 1139  ;
  percentageArray[52]  = 1137;  percentageArray[53] = 1135; percentageArray[54] = 1132; percentageArray[55] = 1130  ;
  percentageArray[56]  = 1128;  percentageArray[57] = 1125; percentageArray[58] = 1122; percentageArray[59] = 1120  ;
  percentageArray[60]  = 1117;  percentageArray[61] = 1114; percentageArray[62] = 1113; percentageArray[63] = 1110  ;
  percentageArray[64]  = 1107;  percentageArray[65] = 1104; percentageArray[66] = 1101; percentageArray[67] = 1097  ;
  percentageArray[68]  = 1094;  percentageArray[69] = 1092; percentageArray[70] = 1088; percentageArray[71] = 1085  ;
  percentageArray[72]  = 1083;  percentageArray[73] = 1080; percentageArray[74] = 1075; percentageArray[75] = 1071  ;
  percentageArray[76]  = 1068;  percentageArray[77] = 1065; percentageArray[78] = 1062; percentageArray[79] = 1056  ;
  percentageArray[80]  = 1049;  percentageArray[81] = 1045; percentageArray[82] = 1041; percentageArray[83] = 1037  ;
  percentageArray[84]  = 1033;  percentageArray[85] = 1026; percentageArray[86] = 1021; percentageArray[87] = 1015  ;
  percentageArray[88]  = 1006;  percentageArray[89] = 1000; percentageArray[90] = 992;  percentageArray[91] = 983 ;
  percentageArray[92]  = 974 ;  percentageArray[93] = 965;  percentageArray[94] = 956;  percentageArray[95] = 944 ;
  percentageArray[96]  = 926 ;  percentageArray[97] = 909;  percentageArray[98] = 882;  percentageArray[99] = 847 ;
  percentageArray[100] = 812 ;
  //This is a simple solution due to time constraints, it is not particularly efficient or elegant, but it works.
  //Goes through the loop and if it is greater than a value it stops thier and outputs the associated percentage where it stops
  for (int j = 0; j <= 100; ++j) {
    if (avgVolt > percentageArray[j]) {
      return 100 - j;
      break;
    }
  }
  return 0;
}

byte dac_get_state() {
  return digitalRead(PIN_EN_VIN_FET);
}

void dac_on() {
  digitalWrite(PIN_EN_VIN_FET, HIGH);
  en_status = true;
}

void dac_off() {
  digitalWrite(PIN_EN_VIN_FET, LOW);
  en_status = false;
}

void dac_set_lsb(byte d) {

  dac_pwm_lsb = d;
  analogWrite(PIN_DAC_LSB, dac_pwm_lsb);

}

void dac_update_lsb(void) {
  analogWrite(PIN_DAC_LSB, dac_pwm_lsb);
}

void dac_update_msb(void) {
  analogWrite(PIN_DAC_MSB, dac_pwm_msb + dac_pwm_msb_offset);
}

//alex's modified code
unsigned int dac_get_lsb() {
  return dac_pwm_lsb;
}

unsigned int dac_get_msb() {
  return dac_pwm_msb;
}

void dac_increment_lsb() {
  dac_pwm_lsb++;
  dac_update_lsb();
}

void dac_decrement_lsb() {
  dac_pwm_lsb--;
  dac_update_lsb();
}

void dac_increment_msb() {
  dac_pwm_msb++;
  dac_update_msb();
}

void dac_decrement_msb() {
  dac_pwm_msb--;
  dac_update_msb();
}

void dac_set_msb_offset(byte os) {

  dac_pwm_msb_offset = os;
  if (dac_pwm_msb <= 255 - dac_pwm_msb_offset)
    analogWrite(PIN_DAC_MSB, dac_pwm_msb + dac_pwm_msb_offset);

}

//-----------------------------------------------------------------------------
void io_init(void) {

  // setup output pins
  pinMode(PIN_DAC_LSB, OUTPUT);
  pinMode(PIN_DAC_MSB, OUTPUT);
  pinMode(PIN_PS_PWM, OUTPUT);
  pinMode(PIN_EN_VIN_FET, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  pinMode(PIN_LED_RED_13, OUTPUT);

  digitalWrite(PIN_EN_VIN_FET, LOW); // make sure EL is not enabled.

  // turn on pull-ups on switch input pins
  digitalWrite(PIN_SW_S2, HIGH);
  digitalWrite(PIN_SW_CTR, HIGH);
  digitalWrite(PIN_SW_UP, HIGH);
  digitalWrite(PIN_SW_DN, HIGH);
  digitalWrite(PIN_SW_RT, HIGH);
  digitalWrite(PIN_SW_LT, HIGH);

}

//-----------------------------------------------------------------------------
void oled_init(void) {
  // init the OLED display
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //display.display();
  //delay(2000);
  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.println(F(WELCOME_MSG));
  display.println(F(SWR_VERSION));
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();

}

//-----------------------------------------------------------------------------
void ps_init(void) {
  // turn on switchcap power supplies
  // Modify TIMER2 config:
  // output pins for timer2: D3 and D11
  // 16MHz/2/ps/256 = 31.25kHz / ps = 488.28 (or 1/2.048ms) when ps=64.
  // The divide by two is due to phase-correct PWM.
  // TCCR2B = (1<<CS22) // ps = 1/64, the Arduino default
  TCCR2B = (1 << CS20); // ps = 1, yielding 31.25kHz

  // set duty cycle to about 50%
  analogWrite(PIN_PS_PWM, 128);

}


//Global Declerations for batStat and batTest
int loopCounter = -1;
unsigned int averageVolt = 0;
int lifePercentage;

//-----------------------------------------------------------------------------
void loop() {

  ticks = (unsigned int)millis();


  // do 10ms stuff here:
  if ( (ticks - ticks_10ms) > 10) {
    ticks_10ms += 10;

    // service the ADC, update viref_mv, vdut_mv, idut_ma:
    adc_service();


    // DAC TUNING CODE
    // STILL IN DEBUGGING, SO MAY WANT TO COMMENT OUT.
    /*
      if(dac_tune_viref() == 1){
        ;//digitalWrite(PIN_LED_GRN,HIGH);
      } else
        ;//digitalWrite(PIN_LED_GRN,LOW);

      if(dac_get_state()){
      if(dac_tune_iref() == 1){
        digitalWrite(PIN_LED_GRN,HIGH);
      } else
        digitalWrite(PIN_LED_GRN,LOW);
      }
    */

  }


  // do 100ms stuff here.
  if ( (ticks - ticks_100ms) > 100) {
    ticks_100ms += 100;
    if (!digitalRead(PIN_SW_S2) && !isTesting) {//if the button is pushed and is already not testing
      isTesting = true;
    }
    ui_service();


  }

  // do 1000ms stuff here:
  if ( (ticks - ticks_1000ms) > 1000) {
    ticks_1000ms += 1000;

    if (isTesting) { //if the command has been issued or button pressed test battery
      testBat();
    }
    else {
      uo_service();//if bat stat is called dont display
    }

    static unsigned int n;
    //Serial.println(n++);

    if (digitalRead(PIN_LED_RED_13))
      digitalWrite(PIN_LED_RED_13, LOW);
    else
      digitalWrite(PIN_LED_RED_13, HIGH);


  }

}
/////////////////////////////
//Battery Testing Functions//
//Scotti Bozarth           //
//Josh Clement             //
/////////////////////////////
void testBat() {
  ++loopCounter;
      display.clearDisplay();
      display.setCursor(1,1);
      char str[20];
      if(loopCounter < 7) {
        sprintf(str, "Testing Battery");
        display.println(str);
        display.display();
      }
      
      if (loopCounter == 0) { //what happens when the button is pressed or function is called, need to have some display that says please wait
        dac_on();
        dac_setIrefMa(200);
        userCurrent = 200;
      }
      else if (loopCounter == 12) { //set everything back to normal, in here so the screen doesnt go away imediatly
        loopCounter = -1;
        averageVolt = 0;
        isTesting = false;
        dac_off();
        //button pressed = false
      }
      else if (loopCounter == 7) { //we take the measurement here and after;
        averageVolt += adc_vdut_mv;
        Serial.print("\nAverage Voltage: ");
        Serial.print(averageVolt/5);
        Serial.print("\n");
        lifePercentage = batStat(averageVolt/5);//returns int of percent left
        dac_setIrefMa(0);
        userCurrent = 0;
        Serial.print("Battery Life Remaining: ");
        Serial.print(lifePercentage);
        Serial.print("%\n");
      }
      else if (loopCounter > 7 && loopCounter < 12) {
        display.clearDisplay();
        display.setCursor(0,0);
        char str[32];
        sprintf(str, "Battery Life\nRemaining:\n%d%%", lifePercentage);
        display.println(str);
        display.display();
        
      }
      else if (loopCounter >= 3){
         averageVolt += adc_vdut_mv;        
      }
      else if (loopCounter > 0){
      }
      else {//just in case
        loopCounter = -1;
        averageVolt = 0;
        isTesting = false;
      }
}


void uo_service(void) {

  display.clearDisplay();
  display.setCursor(0, 0);


  //adc_viref_mv = analogRead(PIN_ADC_V_IREF);
  //adc_idut_ma = analogRead(PIN_ADC_IDUT);
  //adc_vdut_mv = analogRead(PIN_ADC_VDUT);


  static unsigned int i = 0;
  char str[64];
  sprintf(str, "VIREF = %d mV\nVDUT = %d mV\nIDUT = %d mA\n%u",
          adc_viref_mv, adc_vdut_mv, adc_idut_ma, ++i ); //changed percent d to u
  display.println(str);
  display.display();
  if (i % 5 == 0) {
    Serial.print(i);
    Serial.print(", ");
    Serial.print(adc_vdut_mv);
    Serial.print(", ");
    Serial.print(adc_idut_ma);
    Serial.println();
  }


  /*
    sprintf(str,"%d\tVIREF = %d mV\tVDUT = %d mV\tIDUT = %d mA\n",
                  i++, adc_viref_mv, adc_vdut_mv,adc_idut_ma );
    Serial.write(str);
  */


}


void ui_service(void) {
  char str[64];

  byte n = ui_service_uart();
  if (n) {

    sprintf(str, "VIREF = %d mV\tVDUT = %d mV\tIDUT = %d mA\n", adc_viref_mv, adc_vdut_mv, adc_idut_ma );
    Serial.write(str);

    ui_service_commands();

    if (1) {
      delay(3);
      sprintf(str, "VIREF = %d mV\tVDUT = %d mV\tIDUT = %d mA\n", adc_viref_mv, adc_vdut_mv, adc_idut_ma );
      Serial.write(str);
    }

  }



  /*  if(n){
      Serial.print("num tokens = ");
      Serial.println(n);
    }
    for (byte i=0; i<n; ++i){
       Serial.write(tokens[i]);
       if(i== n-1)
           Serial.println("!");
       else
           Serial.print(", ");
    } */


}


typedef enum {
  CMD_END = 0,
  CMD_SET_STATE = 1,
  CMD_SET_EN,
  CMD_SET_IREF_AMPLITUDE_MA,
  CMD_SET_IREF_OFFSET_MA,
  CMD_SET_IREF_DUTY,
  CMD_SET_IREF_PERIOD,
  CMD_SET_IREF_NUM_PULSES,
  CMD_SET_DAC_LSB,            // 7

  CMD_SET_ADC_AVERAGING_WEIGHT,
  CMD_SET_PS_DUTY,
  CMD_SET_TEMP_LIMIT_DEGC,
  CMD_SET_ADC_IDUT_OFFSET_COUNTS,
  CMD_SET_ADC_VDUT_OFFSET_COUNTS,
  CMD_EXEC_DAC_TUNE ,
  CMD_DAC_MSB_OFFSET,
  CMD_TEMP_1,                  // 14
  CMD_TEST,
  CMD_TEST1
} cmd_enum_t;

#define CMD_MAX_SIZE_OF_COMMAND_LIST    36
const char* ui_string_cmds[] = {
  "state",    // states: 1 or 0
  "en",       // enable: same as state, currently (2017-04-17) at least
  "ia",       // iref amplitude in mA
  "io",       // iref offset in mA
  "id",       // iref duty in pct 0~100
  "ip",       // iref period in sec
  "in",       // iref number of pulse: 0 = inf, 1 thru 2^16-1
  "dl",       // set dac lsb.
  "aw",       // set adc averaging weight = 0 thru ?
  "pd",       // set power supply duty cycle
  "ot",       // set overtemp limit in degC
  "aio",      // set adc idut offset counts
  "avo",      // set adc vdut offset counts
  "xt",       // execute a DAC tune
  "dmo",      // DAC MSB offset
  "temp1",     // temp1
  "test",
  "test1",
  NULL        // needed for ending the list of strings
};

cmd_enum_t ui_enum_cmds[] = {
  CMD_SET_STATE,                  // states: 1 (on), 0 (off)
  CMD_SET_EN,                     // enable: 1 (on), 0 (off)
  CMD_SET_IREF_AMPLITUDE_MA,      // iref amplitude in mA
  CMD_SET_IREF_OFFSET_MA,         // iref offset in mA
  CMD_SET_IREF_DUTY,              // iref duty in pct 0~100
  CMD_SET_IREF_PERIOD,            // iref period in sec
  CMD_SET_IREF_NUM_PULSES,        // iref number of pulse: 0 = inf, 1 thru 2^16-1
  CMD_SET_DAC_LSB,                // set dac lsb.
  CMD_SET_ADC_AVERAGING_WEIGHT,   // set adc averaging weight = 0 thru ?
  CMD_SET_PS_DUTY,                // set power supply duty cycle
  CMD_SET_TEMP_LIMIT_DEGC,        // set overtemp limit in degC
  CMD_SET_ADC_IDUT_OFFSET_COUNTS, // set adc idut offset counts
  CMD_SET_ADC_VDUT_OFFSET_COUNTS, // set adc vdut offset counts
  CMD_EXEC_DAC_TUNE,              // execute a DAC tune
  CMD_DAC_MSB_OFFSET,
  CMD_TEMP_1,
  CMD_TEST,  // command to see if correct command is recieved
  CMD_TEST1,  // command to see if we can sucsesfully output inputted number
  CMD_END   // needed, to match the NULL entry in ui_string_cmds[]
};




typedef struct cmd_s {
  cmd_enum_t   cmd;
  unsigned int    param;
}   cmd_t;


typedef struct cmd_parse_s {
  const char        cmd_s[6];
  cmd_enum_t  cmd_e;

} cmd_parse_t;

cmd_parse_t    ui_cmd_parse[] = {
  {"state",           CMD_SET_STATE                       },
  {"en",              CMD_SET_EN,                         },
  {"ia",              CMD_SET_IREF_AMPLITUDE_MA,          },
  {"io",              CMD_SET_IREF_OFFSET_MA,             },
  {"id",              CMD_SET_IREF_DUTY,                  },
  {"ip",              CMD_SET_IREF_PERIOD,                },
  {"in",              CMD_SET_IREF_NUM_PULSES,            },
  {"dl",              CMD_SET_DAC_LSB,                    },
  {"aw",              CMD_SET_ADC_AVERAGING_WEIGHT,       },
  {"pd",              CMD_SET_PS_DUTY,                    },
  {"ot",              CMD_SET_TEMP_LIMIT_DEGC,            },
  {"aio",             CMD_SET_ADC_IDUT_OFFSET_COUNTS,     },
  {"avo",             CMD_SET_ADC_VDUT_OFFSET_COUNTS,     },
  {"xt",              CMD_EXEC_DAC_TUNE,                  },
  {"dmo",             CMD_DAC_MSB_OFFSET,                 },
  {"tmp1",            CMD_TEMP_1,                         },
  {"test",            CMD_TEST,                           },
  {"test1",           CMD_TEST1,                          }

};



#define CMD_STACK_SIZE  20
cmd_t   cmd_stack[CMD_STACK_SIZE];
byte    cmd_stack_num_items = 0;


byte ui_cmd_push(struct cmd_s* pCommand) {
  if (cmd_stack_num_items < CMD_STACK_SIZE) {
    cmd_stack_num_items++;
    cmd_stack[cmd_stack_num_items - 1].cmd = pCommand->cmd;
    cmd_stack[cmd_stack_num_items - 1].param = pCommand->param;
  }

  return cmd_stack_num_items;
}


struct cmd_s* ui_cmd_pop(void) {
  if (cmd_stack_num_items == 0) {
    // nothing on the stack
    return NULL;
  }

  cmd_stack_num_items--;
  return &cmd_stack[cmd_stack_num_items];
}

struct cmd_s* ui_cmd_ith(byte ith) {
  if (ith < cmd_stack_num_items) {
    return &cmd_stack[ith + 1];
  } else
    return NULL;
}

void ui_clear_stack(void) {
  cmd_stack_num_items = 0;
}

void ui_cmd_flip_stack(void) {
  cmd_t cmd;
  byte N = cmd_stack_num_items / 2;
  Serial.print("cmds:"); Serial.println(cmd_stack_num_items);
  //Serial.println("flipping stack...");
  for (byte i = 0; i < N; ++i) {
    cmd = cmd_stack[i];

#ifdef DEBUG
    Serial.print(i); Serial.print("::"); Serial.print(cmd_stack[i].cmd);
    Serial.print("="); Serial.print(cmd_stack[i].param);
    Serial.print("|");
#endif //DEBUG

    cmd_stack[i] = cmd_stack[cmd_stack_num_items - i - 1];
    cmd_stack[cmd_stack_num_items - i - 1] = cmd;

#ifdef DEBUG
    Serial.print(cmd_stack[i].cmd);
    Serial.print("="); Serial.println(cmd_stack[i].param);
#endif // DEBUG
  }

}

// note: argument e should be of type cmd_enum_t but
// getting an error...
const char* ui_get_cmd_text(int e) {
  byte i = 0;
  while (*ui_string_cmds[i] && i < CMD_MAX_SIZE_OF_COMMAND_LIST) {
    if (e == ui_enum_cmds[i])
      return ui_string_cmds[i];
    ++i;
  }
  return NULL;
}

void uo_print_str(const char* str) {
  Serial.println(str);

}


void uo_print_param(const char* str1, const char* str2, unsigned int param, const char* str3) {
  if (str1)    Serial.print(str1);
  if (str2)    Serial.print(str2);
  Serial.print(param);
  if (str3)    Serial.print(str3);

}

void ui_service_commands(void) {

  cmd_t* pCommand;
  ui_cmd_flip_stack(); // flip order of stack
  while (cmd_stack_num_items) {
    pCommand =  ui_cmd_pop();
    unsigned int p = pCommand->param;
    /* Serial.print(pCommand->cmd); Serial.print(":");Serial.println(p); */
    const char* cc = ui_get_cmd_text(pCommand->cmd);
    uo_print_param(cc, "=", p, "\n");

    switch (pCommand->cmd) {

      case CMD_SET_STATE:
      case CMD_SET_EN:

        if (p == 1) {
          dac_on();
          uo_print_str("set:ON\n");
        } else {
          dac_off();
          uo_print_str("set:OFF\n");
        }

        break;

      case CMD_SET_IREF_AMPLITUDE_MA:


        uo_print_param("set:IREF = ", "", p, "\n");
        dac_setIrefMa(p);
        userCurrent = p;

        break;

      case CMD_SET_IREF_OFFSET_MA:


        break;

      case CMD_SET_IREF_DUTY:
        break;

      case CMD_SET_IREF_PERIOD:
        break;

      case CMD_SET_IREF_NUM_PULSES:
        break;

      case CMD_SET_DAC_LSB:
        uo_print_param("set:LSB = ", "", p, "\n");
        dac_set_lsb(p);
        break;

      case CMD_SET_ADC_AVERAGING_WEIGHT:
        break;

      case CMD_SET_PS_DUTY:
        break;

      case CMD_SET_TEMP_LIMIT_DEGC:
        break;

      case CMD_SET_ADC_IDUT_OFFSET_COUNTS:
        break;

      case CMD_SET_ADC_VDUT_OFFSET_COUNTS:
        break;

      case CMD_EXEC_DAC_TUNE:
        break;

      case CMD_DAC_MSB_OFFSET:
        uo_print_param("set:MSB OS = ", "", p, "\n");
        dac_set_msb_offset(p);
        break;
      case CMD_TEST:
        Serial.print("Testing Battery\n");
        isTesting = true;
        break;
      case CMD_TEST1:
        Serial.print("Number ");
        Serial.print(p);
        break;

    }

  }

}



byte ui_service_uart(void) {

  // some useful string.h functions:
  // http://www.cplusplus.com/reference/cstring/strtok/
  //

  if (Serial.available()) {

    char i;
    byte size = Serial.readBytes(input_buffer, UI_INPUT_BUFFER_SIZE);
    input_buffer[size] = 0; // null terminate the string

    //Serial.write(input_buffer,strlen(input_buffer));
    //Serial.println();

    //// tokenize the received string:
    // first zero (null) the pointers to the tokens:
    for (i = 0; i < UI_MAX_TOKENS; ++i)  tokens[i] = 0;

    byte num_tokens = 0;
    pch = strtok (input_buffer, UI_DELIMITERS);
    while (pch != NULL && num_tokens < UI_MAX_TOKENS)
    {
      tokens[num_tokens++] = pch;
      pch = strtok (NULL, UI_DELIMITERS);
    }
    Serial.print(num_tokens); Serial.println(" tokens found. ");

    for (i = 0; i < num_tokens; i += 2) {
      //Serial.write(tokens[i],strlen(tokens[i]));
      //Serial.print(" | ");
      //Serial.println();
      char t[16]; strcpy(t, tokens[i]);
      char *p = t;
      for ( ; *p; ++p) *p = tolower(*p); // to lower case

      char j = 0;
      while (*ui_string_cmds[j] && j < CMD_MAX_SIZE_OF_COMMAND_LIST) {

        if (!strcmp(ui_string_cmds[j], t) && i < (num_tokens - 1)) {
          cmd_t    cmd;
          cmd.cmd = ui_enum_cmds[j];
          cmd.param = atoi(tokens[i + 1]);

          Serial.print("cmd.cmd="); Serial.print(cmd.cmd); Serial.print(" .param="); Serial.println(cmd.param);

          ui_cmd_push(&cmd);
          j =  CMD_MAX_SIZE_OF_COMMAND_LIST;
        } else {
          ++j;
        }


        /* char s[] = "iref";
          if(!strcmp(s,t)  && i<(num_tokens-1)){
            Serial.print("set:IREF = ");
            unsigned int ma = atoi(tokens[i+1]);
            Serial.println(ma);
            dac_setIrefMa(ma);
          } else if (!strcmp("on",t)){
            dac_on();
            Serial.println("set:ON");
          } else if (!strcmp("off",t)){
            dac_off();
            Serial.println("set:OFF");
          }  */



      }

    }

    //Serial.println();
    return num_tokens;
  }

  return 0;

}


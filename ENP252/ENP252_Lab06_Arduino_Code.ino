// INCTRUCTABLES ---------------------------------------------------------------------------------------------------------------------------------------------------------
/*
Arduino software that controls V_IREF, the ON/OFF modulator, and measures VDUT and IDUT via the Arduino IDE Serial monitor.

Functionality:

    Set V_IREF to a millivolt value of 0mV to 300mV using the Serial monitor.
    Set the modulator to ON or OFF via the Serial monitor.
    Set the VDUT and IDUT measurement rate (500ms to 5 sec period, or something else reasonably).
    Measure and report VDUT and IDUT periodically.  report in millivolts and milliamps or other appropriate units. Include a timestamp in appropriate units.


Submit syntax-highlighted commented code in a pdf document, as well as Serial monitor
*/

// PINOUTS ---------------------------------------------------------------------------------------------------------------------------------------------------------------
int MOD_PIN = 5; 			// D5
int MSB_PIN = 9; 			// D9
int LSB_PIN = 10; 			// D10
int PS_PWM_PIN = 11; 		// D11
int LED_PIN = 13; 			// D13
int IDUT_PIN = 14; 			// A0
int VDUT_PIN = 15; 			// A1
int ADC_IREF_PIN = 16; 		// A2

// INITIALIZE -----------------------------------------------------------------------------------------------------------------------------------------------------------
const double REFERENCE_VOLTAGE = 3.05;
unsigned int readmS;
int V_dac;
int pwmLSB = 0;


// SETUP -----------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
	// SET PIN I/O
	Serial.begin(9600);
	while( !Serial ) {}
	pinMode(MOD_PIN, OUTPUT);
	pinMode(MSB_PIN, OUTPUT);
	pinMode(LSB_PIN, OUTPUT);
	pinMode(PS_PWM_PIN, OUTPUT);
	pinMode(IDUT_PIN, INPUT);
	pinMode(VDUT_PIN, INPUT);
	pinMode(ADC_IREF_PIN, OUTPUT);

	analogReference(EXTERNAL);

	analogWrite(PS_PWM_PIN, 255/2);

	initCurrent();
	initModulator();
	initRate();
}


// Initialize part deux ----------------------------------------------------------------------------------------------------------------------------------------------------
String inputString;


// LOOP --------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {

	// put your main code here, to run repeatedly:

	// Input checker
	if( Serial.available() ) checkInput();
	if(millis() % readmS == 1) {
		printReadings(readVDUT(), readIDUT());
		delay(1);
	}
	settleDac(V_dac);
}


// FUNCTIONS -----------------------------------------------------------------------------------------------------------------------------------------------------------------
int initCurrent() {
	unsigned int currentValue;
	Serial.print("Please input a reference current (0-300mA): ");
	while(!Serial.available()) {} // wait for input

	currentValue = (Serial.readString()).toInt();
	Serial.print(currentValue);
	Serial.println("");
	setV_IRef( currentValue );
	return currentValue;
}

void setV_IRef( unsigned int current ) {
  // This sets the initial voltage, which will then be modulated using modDac() to get a more precise reading
  unsigned long setting = (current * 255) / 500;
  // Voltage will be between 0-300 (mV), 255 = 500mV, so 300mV = (3/5)* 255
  V_dac = setting * 4;
  Serial.println(setting);
  analogWrite(MSB_PIN, setting);
}

unsigned int initModulator() {
	unsigned int modValue;
	Serial.print("Please input a modulator rate (0-100%): ");
	while(!Serial.available()) {} // wait for input

	modValue = (Serial.readString()).toInt();
	Serial.print(modValue);
	Serial.println("");
	setMod( modValue );
	return modValue;
}

void setMod( int dutyCycle ) {
	dutyCycle = (dutyCycle*255)/100;
	analogWrite(MOD_PIN, 255-dutyCycle);
}

unsigned int initRate() {
	unsigned int rateValue;
	Serial.print("Please input a read interval (mS) : ");
	while(!Serial.available()) {} // wait for input

	rateValue = (Serial.readString()).toInt();
	Serial.print(rateValue);
	Serial.println("");
	setRate(rateValue);
	return rateValue;
}

void setRate( unsigned int rate ) {
	readmS = rate;
}

void checkInput() {
	inputString = Serial.readString();
	if (inputString == "set v_iref\n") {
		initCurrent();
	} else if (inputString == "set rate\n") {
		initRate();
	} else if (inputString == "set mod\n") {
		initModulator();
	} else if (inputString == "help\n") {
		displayCommands();
	} else if (inputString == "stop\n") {
		stop();
	} else {

	}
}

void stop() {
	setMod(0);
	setRate(0);
	setV_IRef(0);
}


void displayCommands(){
	Serial.println("Commands:");
	Serial.println("set v_iref - Sets reference voltage");
	Serial.println("set rate   - Sets frequency of measurements");
	Serial.println("set mod    - Sets modulation rate");
	Serial.println("stop       - Sets v_iref, rate, and mod to 0");
	Serial.println("help	   - Displays commands");
}


void settleDac( int voltage ) {
	int V_dac_actual = analogRead(ADC_IREF_PIN);
	if(V_dac_actual < voltage) {
		pwmLSB++;
	} else if ( V_dac_actual > voltage ) {
		pwmLSB--;
	}
	analogWrite(LSB_PIN, pwmLSB);
}

// actual voltage = (reading/1024)* REFERENCE_VOLTAGE * Divider (4)
int readVDUT() {
	long reading = analogRead(VDUT_PIN);
	reading *= REFERENCE_VOLTAGE; 	// for reference voltage
	reading *= 4;   				// for voltage divider
	reading *= 1000;				// adjust for accuracy and to be in mV
	reading /= 1024;				// adjusting for reading ratio
	return reading;
}


// actual current (reading/1024) * REFERENCE_VOLTAGE
int readIDUT() {
	long reading = analogRead(IDUT_PIN);
	reading *= REFERENCE_VOLTAGE;	// reference voltage
	reading *= 1000;				// adjust for mA
	reading /= 1024;				// adjust for reading ratio

	return reading;
}


void printReadings(int VDUT, int IDUT) {
	Serial.print("VDUT = ");
	Serial.print(VDUT);
	Serial.print(" mv");
	Serial.print("\t");
	Serial.print("\t");
	Serial.print("IDUT = 0.0");
	Serial.print(IDUT);
	Serial.print(" A");
	Serial.print("\t");
	Serial.print("\n");
}

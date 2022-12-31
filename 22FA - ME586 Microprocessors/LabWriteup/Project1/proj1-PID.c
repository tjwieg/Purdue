// DAC bounds are constants
#define OUTPUT_MAX 4095
#define OUTPUT_MIN 0

// Global variables
int ram_ptr[1000];        // measured data stored in memory
int stored_actual = 0;    // number of values actually written
float time_period = 10;   // in milliseconds
float setpoint = 0;
float error_prior = 0;
float integral = 0;

// PID Coefficients (D = 0, for PI control)
float Kp = -0.14411;
float Ki = -0.393539;
float Kd =  0.0;

// Timer interrupt routine
void timehand(void) {
    // Setup
    int measured_ADC;
	float measured_engvolt;
    float error;
    float derivative;
    float output;
	float reference;

    // Get position from ADC input
    measured_ADC = a_to_d(1);

    // Convert ADC count to engine voltage
    measured_engvolt = 0.0024414*measured_ADC - 5;

    // Convert ADC count to RPM and save to memory
    if (stored_actual < 1000) {
        ram_ptr[stored_actual] = -0.6984*measured_ADC + 1504;   
        stored_actual = stored_actual + 1;
    }
		
    // PID Controller
	reference = -0.0033*setpoint + 0.0762; // convert setpoint from RPM to engine voltage
    error = reference - measured_engvolt;
    integral = integral + (error * (time_period / 1000)); // T/1000 to convert ms to seconds
    derivative = (error - error_prior) / (time_period / 1000);
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    error_prior = error;

    // Convert output voltage to DAC count (within bounds)
    output = 204.8*output + 2048;
	if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;
	
    d_to_a(0, output);
}
void timehand(void) {
    // Setup
    int measured_ADC;
		float measured_angle;
    float error;
    float derivative;
    float output;

    // Get position from ADC input
    measured_ADC = a_to_d(1);
    measured_angle = 0.1729*measured_ADC - 535.34; // convert ADC count to degrees
    if (stored_actual < 1000) {
        ram_ptr[stored_actual] = measured_angle;   // store degrees to memory
        stored_actual = stored_actual + 1;
    }
    
    // PID Controller
    error = setpoint - measured_angle;
    error = 0.0024*error; 	// convert degrees to ADC voltage (no offset inside PID)
    integral = integral + (error * (time_period / 1000)); // convert ms to seconds
    derivative = (error - error_prior) / (time_period / 1000);
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;
    error_prior = error;
    output = output*205.52 + 2049.8; // convert voltage to DAC
    d_to_a(0, output);
}
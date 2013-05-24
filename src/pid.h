#pragma once

class PID {
    public:
        PID();
        PID(float* Input, float* Output, float* Setpoint, float* terms);
        void Compute();
        void IntegralReset();

    private:
        float *PID_input;
        float *PID_output;
        float *PID_setpoint;
        
        float *Kp, *Ki, *Kd;
        float *windupGuard;
        
        float previous_error;
        float integral;
        unsigned long last_time;
};
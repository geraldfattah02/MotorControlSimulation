#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

// consts
constexpr double dt = 0.01; //time step in secs
constexpr double motor_gain = 0.1; //motor gain (rad/s per pwm)
constexpr double time_const = 0.5; // motor time const (s) how quickly system repsonds to changes

class PID {
    public:
        PID(double kp, double ki, double kd)
        : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0) {}

        double update(double setpoint, double measured) {
            double error = setpoint - measured;
            integral += error * dt;
            double derivative = (error - prev_error) / dt;
            prev_error = error;
            return (kp*error)+ (ki * integral) + (kd*derivative);
        }
    private:
        double kp, ki, kd;
        double prev_error;
        double integral;
};

class Motor {
    public:
        Motor() : velocity(0), input_pwm(0) {}

        void apply_pwm(double pwm) {
            //clamping the pwm signal between -100 and 100
            if (pwm < -100.0)
                input_pwm = -100.0;
            else if (pwm > 100.0)
                input_pwm = 100.0;
            else
                input_pwm = pwm;
        }
        void update() {
            double target_velocity = motor_gain*input_pwm;
            velocity += (target_velocity - velocity)*(dt/ time_const); //first order system response
        }
        double get_velocity() const {return velocity;}
    private:
        double velocity;
        double input_pwm;
};

int main() {
   PID pid(1.2,0.5,0.1);
   Motor motor;
   double setpoint = 10.0; //target speed in rad/s

   for (int i = 0; i < 1000; ++i){
    double measured_velocity = motor.get_velocity();
    double control_signal = pid.update(setpoint, measured_velocity);

    motor.apply_pwm(control_signal);
    motor.update();

    std::cout << "Time: " << i*dt << "s\t"
              << "Setpoint" << setpoint << "\t"
              << "Velocity" << measured_velocity << "\t"
              << "PWM:" << control_signal << std::endl;
    
              std::this_thread::sleep_for(std::chrono::milliseconds(10));              
   }
   return 0;

}


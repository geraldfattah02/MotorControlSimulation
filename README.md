# Motor Controller Simulation

This C++ project simulates a simple closed-loop motor controller using a PID (Proportional-Integral-Derivative) control algorithm. It's designed to be a simulate a simple embedded controller without requiring any physical hardware.

## 📦 Features
- Simulated DC motor with a first-order response model
- Configurable PID controller (Kp, Ki, Kd)
- Real-time terminal output of setpoint, measured velocity, and control signal
- Easy to extend with noise, quantization, logging, or additional features

## 🧠 System Model
The motor is modeled as a first-order system:
```
velocity += (target_velocity - velocity) * (dt / time_constant);
```
Where:
- `target_velocity` is derived from the input PWM signal
- `dt` is the simulation timestep (10ms)
- `time_constant` determines how quickly the motor reaches the desired speed

## 🚀 How to Run
### Requirements
- [VS Code](https://code.visualstudio.com/)
- `g++` (e.g. via MinGW on Windows or GCC on Linux/macOS)

### Build & Run in Terminal
```bash
g++ -std=c++17 MotorControllerSim.cpp -o MotorControllerSim
./MotorControllerSim
```

### Using VS Code
1. Install the **C/C++** extension by Microsoft
2. Press **Ctrl+Shift+B** to build (with `tasks.json` configured)
3. Run the `MotorControllerSim` executable

## 📈 Example Output
```
Time: 0.00 s	Setpoint: 10.00	Velocity: 0.00	PWM: 12.00
Time: 0.01 s	Setpoint: 10.00	Velocity: 0.12	PWM: 11.88
...
Time: 0.50 s	Setpoint: 10.00	Velocity: 9.63	PWM: 0.24
```

## 📚 Future Ideas
- Add encoder quantization
- Log data to CSV for plotting
- Tune PID using Ziegler–Nichols method
- Add step or ramp changes to the setpoint

## 🔧 License
MIT License

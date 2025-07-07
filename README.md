# Dynamics and Control of the Reaction Wheel Inverted Pendulum

An educational and experimental platform for controlling a reaction wheel inverted pendulum (RWIP). The project includes a description of the system dynamics, control algorithms, and a guide for working with the hardware platform.

The pendulum is capable of operating in the following modes:
- Swing-up (Energy-Shaping Control)
- Stabilization in the upright position (LQR)
- Braking in the downward vertical position (Energy-Shaping Control)

The repository provides a seamless workflow from theory to hardware implementation. It is well-suited for control theory or robotics courses and projects, as well as for individuals interested in mastering the control of unstable systems.

[![RWIP](https://i9.ytimg.com/vi_webp/FLvGr2hywII/mq2.webp?sqp=CIyor8MG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGGUgWihOMA8=&rs=AOn4CLCTw3LyJUpaOBZD5oiOJYqrvIp1kw)](https://youtu.be/FLvGr2hywII?si=-k68ezkW-7m0eRbZ)

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/FLvGr2hywII/0.jpg)](https://youtu.be/FLvGr2hywII?si=-k68ezkW-7m0eRbZ)


## Key Components
- Theoretical manual
- Python notebook for modeling and control synthesis
- Fully functional microcontroller firmware

## Covered Theoretical Topics
- Derivation of the nonlinear model of the inverted pendulum using the Lagrange method
- Linearization of nonlinear models
- Discretization of continuous-time systems
- Controllability and stability criteria
- LQR controller synthesis
- Energy-Shaping Control

## Repository Structure

 - `Firmware`
   - `wheel_pendulum_practicum.ino` – microcontroller firmware for pendulum control
 - `Python`
   - `model.ipynb` – pendulum models + control synthesis
 - `Tutorials` – guides and manuals
   - `Ru_Installation_guide.pdf` – software installation guide (in Russian)
   - `Ru_Theory.pdf` – theoretical description of the dynamics and control + practical guide for working with the hardware platform (in Russian)
   - `Eng_Installation_guide.pdf` – software installation guide (in English)
   - `Eng_Theory.pdf` – theoretical description of the dynamics and control + practical guide for working with the hardware platform (in English)

## Installation & Usage
- Install all required software according to the instructions in files `Eng_Installation_guide.pdf` or `Ru_Installation_guide.pdf`
- You can immediately flash the microcontroller with the firmware `wheel_pendulum_practicum.ino`
- After powering the pendulum control board, wait until the motor initialization completes. Then press the **USR** button to start the control system.
- To better understand the pendulum control theory, study files ``Eng_Theory.pdf`` or ``Ru_Theory.pdf``. These files are written as lab manuals. For deeper understanding, complete all practical tasks and answer the review questions included in the text.

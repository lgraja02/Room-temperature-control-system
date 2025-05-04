# 🔥 Temperature Control Simulation – PI Controller (Streamlit)

This project is an interactive Python application built with **Streamlit**, simulating the behavior of an **incremental PI controller** used to regulate the temperature of a room.

## 📌 Description

The model is based on a simplified heat balance, taking into account:
- the thermal capacity of air in the room,
- heat loss through walls (using heat transfer coefficient `Uc`),
- the operation of multiple parallel heaters with defined power and efficiency.

The simulation demonstrates how the PI controller adjusts the heater power (control signal `u`) based on the difference between external and target temperatures (`setpoint`) to achieve thermal stability in the room.

## 🎮 Application Features

- Adjustable parameters: `Kp`, `Ti`, `Tp`, target and external temperatures, simulation time.
- Visualization of temperature progression, heater power, voltage, current, and control signal `u`.
- Realistic simulation of multiple parallel heaters.
- Step-by-step time simulation with regulation error tracking.

## 🧰 Required Libraries

To run the application, install the following Python libraries:

```bash
pip install streamlit numpy matplotlib

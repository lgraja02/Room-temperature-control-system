import streamlit as st
import matplotlib.pyplot as plt
import numpy as np

# ==================================== CONSTANTS ===============================================
Cp = 1295       # [J/(kg*K)]  specific heat of air
p = 101325      # [Pa]        room pressure
M = 0.0289      # [kg/mol]    molar mass of air
R_gas = 8.314   # [J/(mol*K)] universal gas constant
Uc = 0.56       # [W/(m²*K)]  heat transfer coefficient

room_length = 5     # [m]
room_width = 10     # [m]
room_height = 3     # [m]

# Room volume
V = room_length * room_width * room_height  # [m³]

# Wall surface area
A = (2 * room_length * room_height + 2 * room_width * room_height)  # [m²]

# Thermal resistance of walls
RT = 1 / (Uc * A)  # [K/W]

# ==================================== HEATER PARAMETERS =====================================
num_heaters = 4         # Number of heaters
heater_efficiency = 0.98  # Efficiency (98%)

heater_power = 2000     # [W] nominal power of a single heater
P_max = num_heaters * heater_power  # [W] total available power

V_nominal = 230.0

R_single = V_nominal**2 / heater_power
R_total = R_single / num_heaters

# ==================================== FUNCTIONS ================================================
def initialize_session_state(T_external, T_target):
    """
    Initializes session state variables.
    """
    st.session_state.time_history = [0]
    st.session_state.temperature_history = [T_external + 273.15]
    st.session_state.power_history = [0]
    st.session_state.last_time = 0
    st.session_state.setpoint_history = [T_target + 273.15]
    st.session_state.error_history = [T_target - T_external]
    st.session_state.control_signal_history = [0.0]
    st.session_state.u_history = [0.0]
    st.session_state.previous_error = 0.0
    st.session_state.previous_u = 0.0

def simulate_incremental_PI(Kp, Ti, Tp, T_setpoint_initial, T_out_initial, simulation_time_hours=10):
    total_steps = int((simulation_time_hours * 3600) / Tp)
    current_T_setpoint = T_setpoint_initial
    current_T_out = T_out_initial

    for step in range(total_steps):
        T_current = st.session_state.temperature_history[-1]
        current_time = st.session_state.last_time
        error = current_T_setpoint - T_current
        delta_error = error - st.session_state.previous_error
        delta_u = Kp * (delta_error + (Tp / Ti) * error)
        u_new = st.session_state.previous_u + delta_u
        u_new = np.clip(u_new, 0.0, 1.0)
        Q_in_real = u_new * P_max * heater_efficiency
        C = V * (p * M) / (R_gas * T_current) * Cp
        T_new = T_current + (Tp / C) * (Q_in_real - (T_current - current_T_out) / RT)
        new_time = current_time + Tp
        st.session_state.time_history.append(new_time)
        st.session_state.temperature_history.append(T_new)
        st.session_state.power_history.append(Q_in_real)
        st.session_state.u_history.append(u_new)
        st.session_state.control_signal_history.append(delta_u)
        st.session_state.error_history.append(error)
        st.session_state.last_time = new_time
        st.session_state.previous_error = error
        st.session_state.previous_u = u_new

# =============================== STREAMLIT APP =========================================
st.title("Temperature Control Simulation Using Incremental PI Controller")

# --- Sidebar parameters ---
with st.sidebar:
    st.header("PI Controller Settings")

    T_target = st.slider('Target temperature (°C):', 15.0, 30.0, 25.5)
    T_external = st.slider('External temperature (°C):', -5.0, 20.0, 10.0)
    Kp = st.slider('Proportional gain (Kp):', 0.0, 0.5, 0.10)
    Ti = st.slider('Integral time (Ti) [s]:', 1.0, 1000.0, 407.0)
    Tp = st.slider('Time step (Tp) [s]:', 1, 10, 1)
    simulation_time_hours = st.slider('Simulation time [h]:', 1, 12, 2)

    if st.button("Run Simulation"):
        initialize_session_state(T_external, T_target)
        st.session_state.simulation_running = True

    if st.button("Reset Simulation"):
        initialize_session_state(T_external, T_target)
        st.session_state.simulation_running = False
        st.success("Simulation has been reset!")

if 'simulation_running' in st.session_state and st.session_state.simulation_running:
    simulate_incremental_PI(Kp, Ti, Tp,
                            T_target + 273.15,
                            T_external + 273.15,
                            simulation_time_hours)
    st.session_state.simulation_running = False

# --------------------------------- DATA PREP ------------------------------------
if 'time_history' not in st.session_state:
    initialize_session_state(T_external, T_target)

T_C_history = np.array(st.session_state.temperature_history) - 273.15
time_hours = np.array(st.session_state.time_history) / 3600.0
power_history_total = np.array(st.session_state.power_history)
u_history = np.array(st.session_state.u_history)

power_history_one = power_history_total / num_heaters
voltage_history = u_history * V_nominal
current_history = voltage_history / R_total

# --------------------------------- PLOTTING --------------------------------------

# 1) Temperature plot
fig1, ax1 = plt.subplots()
ax1.plot(time_hours, T_C_history, label='Internal temperature [°C]', color='blue')
ax1.plot([time_hours[0], time_hours[-1]], [T_target, T_target], linestyle='--', color='red', label='Target temperature')
ax1.set_title('Temperature Control Simulation - Incremental PI')
ax1.set_xlabel('Time [h]')
ax1.set_ylabel('Temperature [°C]')
ax1.legend()
ax1.grid(True)
st.pyplot(fig1)

# 2) Power per heater
fig2, ax2 = plt.subplots()
ax2.plot(time_hours, power_history_one, label='Power per heater [W]', color='green')
ax2.set_title('Power of a Single Heater Over Time')
ax2.set_xlabel('Time [h]')
ax2.set_ylabel('Power [W]')
ax2.legend()
ax2.grid(True)
st.pyplot(fig2)

# 3) Control signal (u)
fig3, ax3 = plt.subplots()
ax3.plot(time_hours, u_history, label='u (0..1) - total power percentage', color='orange')
ax3.set_title('Control Signal (u) Over Time')
ax3.set_xlabel('Time [h]')
ax3.set_ylabel('u [0..1]')
ax3.legend()
ax3.grid(True)
st.pyplot(fig3)

# 4) Voltage plot
fig4, ax4 = plt.subplots()
ax4.plot(time_hours, voltage_history, label='Voltage [V]', color='purple')
ax4.set_title('Heater Supply Voltage Over Time (Simplified)')
ax4.set_xlabel('Time [h]')
ax4.set_ylabel('Voltage [V]')
ax4.legend()
ax4.grid(True)
st.pyplot(fig4)

# 5) Current plot
fig5, ax5 = plt.subplots()
ax5.plot(time_hours, current_history, label='Current [A]', color='brown')
ax5.set_title('Current Over Time (4 Heaters in Parallel)')
ax5.set_xlabel('Time [h]')
ax5.set_ylabel('Current [A]')
ax5.legend()
ax5.grid(True)
st.pyplot(fig5)

# ----------------------- SIDEBAR SUMMARY --------------------------------------
st.sidebar.markdown(f"""
---
### Current Parameters:
- **Target temperature**: {T_target:.2f} °C
- **External temperature**: {T_external:.2f} °C
- **Kp**: {Kp:.2f}
- **Ti**: {Ti:.2f} s
- **Tp**: {Tp} s
- **Simulation time**: {simulation_time_hours} h

### Last Values:
- **Last control signal (u)**: {u_history[-1]:.2f}
  (i.e. {u_history[-1]*100:.1f}% of total power)
- **Total power**: {power_history_total[-1]:.2f} W
- **Power per heater**: {power_history_one[-1]:.2f} W
- **Voltage (approx.)**: {voltage_history[-1]:.2f} V
- **Current (4 heaters)**: {current_history[-1]:.2f} A
""")

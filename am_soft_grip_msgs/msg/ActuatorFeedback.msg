# actuator_state constants
int8 CHARGING=0                        # actuator being actively charged
int8 OPEN_CIRCUIT=1                    # actuator in open circuit mode and nothing happening
int8 SHORT_CIRCUIT=2                   # actuator being shorted to ground
int8 MEASURING=3                       # actuator open circuit voltage being measured
# # #
int8 actuator_state                     # current actuator state
float32 shunt_charging_voltage          # latest measured shunt voltage drop while charging
float32 icl_charging_voltage            # latest measured icl voltage drop while charging
float32 icl_open_circuit_voltage        # latest measured icl voltage drop while open circuit
float32 icl_coulomb_counter             # total charge counted
time last_command                       # last command received
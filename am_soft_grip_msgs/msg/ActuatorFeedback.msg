# actuator_state constants
uint8 CHARGING=0                        # actuator being actively charged
uint8 OPEN_CIRCUIT=1                    # actuator in open circuit mode and nothing happening
uint8 SHORT_CIRCUIT=2                   # actuator being shorted to ground
uint8 MEASURING=3                       # actuator open circuit voltage being measured
# # #
uint8 actuator_state                    # current actuator state
float32 shunt_charging_voltage          # latest measured shunt voltage drop while charging
float32 icl_charging_voltage            # latest measured icl voltage drop while charging
float32 icl_open_circuit_voltage        # latest measured icl voltage drop while open circuit
float32 icl_coulomb_counter             # total charge counted
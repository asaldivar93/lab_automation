## PWM Channels

To control PWM channels you have to create an Output class instance for each channel. The board has up to 6 channels.

```C++
#define N_PWM 6

Output CH_0(0, PIN_CH0, "pwm", MANUAL, 0);
Output CH_1(1, PIN_CH1, "pwm", MANUAL, 0);
Output CH_2(2, PIN_CH2, "pwm", MANUAL, 0);
Output CH_3(3, PIN_CH3, "pwm", MANUAL, 0);
Output CH_4(4, PIN_CH4, "pwm", MANUAL, 0);
Output CH_5(5, PIN_CH5, "pwm", MANUAL, 0);
Output outputs[N_OUTPUTS] = {CH_0, CH_1, CH_2, CH_3, CH_4, CH_5};
```

Channels can be controled in 4 different modes: {MANUAL, TIMER, PID, ONOFF}. You can initialize this control mode through setter functions.

```C++
Output::set_manual_output(int value);
Output::set_timer(int time_on, int time_off, int value);
Output::set_pid(float *input_value, float setpoint);
Output::set_onoff(float *input_value, int lb, int ub, int value);
```


Example:

```C++
void setup(){
  ...
  outputs[2].set_timer(10, 60, 255);
  ...
}
```

Which is equivalent to doing this:

```C++
void setup(){
  ...
  CH_2.set_timer(10, 60, 255);
  ...
}
```

The PID and ONOFF setters expect a pointer to the measured variable:
```C++
void setup(){
  ...
  outputs[2].set_pid(&inputs[1].value, 50.5);
  outputs[3].set_pid(&inputs[3].value, 1, 1.5, 255);
  ...
}
```
### PID control


The PID controler has important parameters that need to be defined during MCU initialization. This parameters have default values:

- PWM_min_vaule = 0
- PWM_max_value = 255
- Sampling time in microseconds = 250000 (0.250s)
- PID constants:
  - Kp = 100, Ki = 0.2, Kd = 0.

The PID controler uses a simple g-h filter to smooth very noisy signals. The filter uses this formula:

```C++
filtered_input[k] = alpha * current_input + (1 - alpha) * filtered_input[k - 1]
```
where *alpha* [0, 1] controls the strength of the smoothing. *alpha* = 1 **no smoothing at all**, *alpha* close to 0 **very strongth smoothing**. By **default** *alpha* = 0.01.

The default values were tuned to control the temperature of a water reservoir using an external electrical resistance and have been tested to heat water volumes between 50mL and 2L.

These parameters are initialized through setter functions:

```C++
Output::set_sample_time_us(uint32_t sample_time_us);
Output::set_pid_tunings(float Kp, float Ki, float Kd);
Output::set_output_limits(float min, float max);
Output::set_gh_filter(float alpha);
```
example:

```C++
void setup(){
  ...
  outputs[2].set_sample_time_us(250000);
  outputs[2].set_pid_tunings(100, 0.2, 0.);
  outputs[2].set_output_limits(0, 240);
  outputs[2].set_gh_filter(0.01);
  outputs[2].set_pid(&inputs[1].value, 50.5);
  ...
}
```
## I2C Sensors

The board has a TCA9544 I2C multiplexer of 4 channels. This multiplexer allows reading sensors with the same address. You can control the multiplexer with the **Sensors** class

```C++
uint8_t multiplexer_address = 0x70;
float temperature;
Sensors sensors(multiplexer_address);

void setup(){
  Wire.begin();
  Serial.begin(9600);
}

void loop(){
  sensors.set_multiplexer_channel(0);
  temperature = sensors.read_sen0546_temperature();
  Serial.println(temperature);
}
```

Sensors API reference:

```C++
// SEN0546 Temperature and humidity sensor
Sensors::read_sen0546_temperature();
Sensors::read_sen0546_humidity();

// SEN0322 Oxygen Sensor
Sensors::read_sen0322_address_0();
Sensors::read_sen0322_address_1();
Sensors::read_sen0322_address_2();
Sensors::read_sen0322_default();

// MPRLS Pressure sensor
Sensors::read_mprls();
Sensors::set_mprls_range(float p_min, float p_max);

// SEN0343 differential pressure
Sensors::read_sen0343_diffpressure();
```

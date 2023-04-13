# Inventor HAT Mini Examples <!-- omit in toc -->

- [Function Examples](#function-examples)
  - [Read ADCs](#read-adcs)
  - [Set GPIOs](#set-gpios)
  - [Read GPIOs](#read-gpios)
  - [Read Encoders](#read-encoders)
  - [Read Internals](#read-internals)
  - [LED Rainbow](#led-rainbow)
  - [Reset Inventor](#reset-inventor)
- [Motor Examples](#motor-examples)
  - [Single Motor](#single-motor)
  - [Dual Motors](#dual-motors)
  - [Motor Wave](#motor-wave)
  - [Position Control](#position-control)
  - [Velocity Control](#velocity-control)
  - [Position on Velocity Control](#position-on-velocity-control)
  - [Reactive Encoder](#reactive-encoder)
  - [Position Wave](#position-wave)
  - [Driving Sequence](#driving-sequence)
- [Motor Tuning](#motor-tuning)
  - [Motor Profiler](#motor-profiler)
  - [Position Tuning](#position-tuning)
  - [Velocity Tuning](#velocity-tuning)
  - [Position on Velocity Tuning](#position-on-velocity-tuning)
- [Servo Examples](#servo-examples)
  - [Single Servo](#single-servo)
  - [Multiple Servos](#multiple-servos)
  - [Simple Easing](#simple-easing)
  - [Servo Wave](#servo-wave)
  - [Calibration](#calibration)
- [Audio Examples](#audio-examples)
  - [Play Sound - PyGame](#play-sound---pygame)
  - [Play Sound - Aplay](#play-sound---aplay)
  - [Motor Song](#motor-song)
  - [Random Droid](#random-droid)
- [Extra Examples](#extra-examples)
  - [GPIO Encoder](#gpio-encoder)
  - [Set Servo Pins](#set-servo-pins)
  - [Read Servo Pins](#read-servo-pins)
  - [Servo Pin Motor](#servo-pin-motor)
  - [Read Ultrasound](#read-ultrasound)
  - [Watchdog Reset](#watchdog-reset)


## Function Examples

### Read ADCs
[read_adcs.py](read_adcs.py)

Shows how to initialise and read the 4 ADC headers of Inventor HAT Mini.


### Set GPIOs
[set_gpios.py](set_gpios.py)

Shows how to initialise and 4 GPIO headers of Inventor HAT Mini as outputs and set them.


### Read GPIOs
[read_gpios.py](read_gpios.py)

Shows how to initialise the 4 GPIO headers of Inventor HAT Mini as inputs and read them.


### Read Encoders
[read_encoders.py](read_encoders.py)

Demonstrates how to read the angles of Inventor HAT Mini's two encoders.


### Read Internals
[read_internals.py](read_internals.py)

Shows how to read the internal sensors of Inventor HAT Mini.


### LED Rainbow
[led_rainbow.py](led_rainbow.py)

Displays a rotating rainbow pattern on Inventor HAT Mini's onboard LED bars.


### Reset Inventor
[reset_inventor.py](reset_inventor.py)

A simple program that resets Inventor HAT Mini, turning off its LEDs, Motors, Servos, and Audio.


## Motor Examples

### Single Motor
[motors/single_motor.py](motors/single_motor.py)

Demonstrates how to control a motor on Inventor HAT Mini.


### Dual Motors
[motors/dual_motors.py](motors/dual_motors.py)

Demonstrates how to control both motors on Inventor HAT Mini.


### Motor Wave
[motors/motor_wave.py](motors/motor_wave.py)

An example of applying a wave pattern to Inventor HAT Mini's motors and LEDs.


### Position Control
[motors/position_control.py](motors/position_control.py)

An example of how to move a motor smoothly between random positions, with the help of it's attached encoder and PID control.


### Velocity Control
[motors/velocity_control.py](motors/velocity_control.py)

An example of how to drive a motor smoothly between random speeds, with the help of it's attached encoder and PID control.


### Position on Velocity Control
[motors/position_on_velocity_control.py](motors/position_on_velocity_control.py)

An example of how to move a motor smoothly between random positions, with velocity limits, with the help of it's attached encoder and PID control.


### Reactive Encoder
[motors/reactive_encoder.py](motors/reactive_encoder.py)

A demonstration of how a motor with an encoder can be used as a programmable rotary encoder for user input, with force-feedback for arbitrary detents and end stops.


### Position Wave
[motors/position_wave.py](motors/position_wave.py)

A demonstration of driving both of Inventor HAT Mini's motor outputs between positions, with the help of their attached encoders and PID control.


### Driving Sequence
[motors/driving_sequence.py](motors/driving_sequence.py)

A demonstration of driving both of Inventor HAT Mini's motor outputs through a sequence of velocities, with the help of their attached encoders and PID control.


## Motor Tuning

### Motor Profiler
[motors/tuning/motor_profiler.py](motors/tuning/motor_profiler.py)

A program that profiles the speed of a motor across its PWM duty cycle range using the attached encoder for feedback.


### Position Tuning
[motors/tuning/position_tuning.py](motors/tuning/position_tuning.py)

A program to aid in the discovery and tuning of motor PID values for position control. It does this by commanding the motor to move repeatedly between two setpoint angles and plots the measured response.


### Velocity Tuning
[motors/tuning/velocity_tuning.py](motors/tuning/velocity_tuning.py)

A program to aid in the discovery and tuning of motor PID values for velocity control. It does this by commanding the motor to drive repeatedly between two setpoint speeds and plots the measured response.


### Position on Velocity Tuning
[motors/tuning/position_on_velocity_tuning.py](motors/tuning/position_on_velocity_tuning.py)

A program to aid in the discovery and tuning of motor PID values for position on velocity control. It does this by commanding the motor to move repeatedly between two setpoint angles and plots the measured response.


## Servo Examples

### Single Servo
[servos/single_servo.py](servos/single_servo.py)

Demonstrates how to control a single servo on Inventor HAT Mini.


### Multiple Servos
[servos/multiple_servos.py](servos/multiple_servos.py)

Demonstrates how to control all of the servos on Inventor HAT Mini.


### Simple Easing
[servos/simple_easing.py](servos/simple_easing.py)

An example of how to move a servo smoothly between random positions.


### Servo Wave
[servos/servo_wave.py](servos/servo_wave.py)

An example of applying a wave pattern to a group of servos and the LEDs.


### Calibration
[servos/calibration.py](servos/calibration.py)

Shows how to configure Inventor HAT Mini's servos with different common calibrations, as well as a completely custom one.


## Audio Examples

### Play Sound - PyGame
[audio/play_sound_pygame.py](audio/play_sound_pygame.py)

Play a WAV audio file from your Inventor HAT Mini, using PyGame!


### Play Sound - Aplay
[audio/play_sound_aplay.py](audio/play_sound_aplay.py)

Play a WAV audio file from your Inventor HAT Mini, using Aplay!


### Motor Song
[audio/motor_song.py](audio/motor_song.py)

A fun example of how to change a motor's frequency to have it play a song.


### Random Droid
[audio/random_droid.py](audio/random_droid.py)

Make your Inventor HAT Mini sound like an Astromech Droid!

This example uses the ttastromech library, for more details see https://pypi.org/project/ttastromech/


## Extra Examples

### GPIO Encoder
[extras/gpio_encoder.py](extras/gpio_encoder.py)

An example of how to read a rotary encoder connected to Inventor HAT Mini's GPIO pins.


### Set Servo Pins
[extras/set_servo_pins.py](extras/set_servo_pins.py)

Shows how to initialise and use Inventor HAT Mini's servo headers as 3.3V outputs and set them.


### Read Servo Pins
[extras/read_servo_pins.py](extras/read_servo_pins.py)

Shows how to initialise and use Inventor HAT Mini's servo headers as 3.3V inputs and read them.


### Servo Pin Motor
[extras/servo_pin_motor.py](extras/servo_pin_motor.py)

Shows how to use Inventor HAT Mini's Servo headers to control an externally connected motor driver.


### Read Ultrasound
[extras/read_ultrasound.py](extras/read_ultrasound.py)

Control a HC-SR04 style ultrasonic distance sensor using the UART header on InventorHATMini.


### Watchdog Reset
[extras/watchdog_reset.py](extras/watchdog_reset.py)

A demonstration of using Inventor HAT Mini's internal watchdog to stop motors from spinning when a simulated code lock-up occurs, by causing a board reset, and recovering from it.

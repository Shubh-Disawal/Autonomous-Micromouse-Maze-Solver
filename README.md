# Autonomous Micromouse Maze Solver (ESP32)

A self-autonomous micromouse robot that senses walls with ultrasonic sensors and navigates a maze on its own using left-wall following, powered by an ESP32.

## Features

- Fully autonomous, with no remote control
- Three HC-SR04 ultrasonic sensors (left, front, right) for wall detection
- Median-filtered distance readings to reduce sensor noise
- Left-wall following with a proportional (P) controller
- Front-wall slowdown zone and hard stop to avoid collisions
- Timed 90-degree pivot turns with a slow-down at the end for accuracy
- Built-in test modes to calibrate the motors, sensors, straight driving and turns separately

## Hardware

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 dev board | 1 | Main controller |
| HW-095 (L298N) motor driver | 1 | Drives both motors |
| HC-SR04 ultrasonic sensor | 3 | Left, front, right |
| DC gear motors with wheels | 2 | Differential drive |
| Battery pack | 1 | Add your battery details here |
| Resistors for voltage dividers | 3 pairs | One divider per ECHO pin |

> **Important:** the HC-SR04 ECHO pin outputs 5 V, but ESP32 GPIO pins are 3.3 V only. Every ECHO pin must go through a voltage divider before reaching the ESP32.

## Pin connections

| Function | GPIO |
|----------|------|
| Left sensor TRIG / ECHO | 22 / 23 |
| Front sensor TRIG / ECHO | 19 / 21 |
| Right sensor TRIG / ECHO | 5 / 18 |
| Left motor IN1 / IN2 / ENA | 26 / 25 / 33 |
| Right motor IN1 / IN2 / ENA | 14 / 27 / 13 |

If a wheel spins the wrong way, flip `LEFT_REVERSED` or `RIGHT_REVERSED` in the code instead of rewiring.

## How it works

### 1. Sensing distance

Each HC-SR04 gets a 10 microsecond trigger pulse. The ESP32 measures how long the echo takes to return using `pulseIn()` and converts it to distance:

```
distance (cm) = echo time (us) x 0.0343 / 2
```

The division by 2 is because the sound travels to the wall and back. Readings below 2 cm, above 120 cm, or timeouts are treated as "no wall" (120 cm).

Ultrasonic sensors give noisy readings, so each sensor is pinged three times and the **median** is used. The median ignores a single bad reading, which a plain average would not. A short delay between pings stops echoes from one reading showing up in the next.

### 2. Driving the motors

The L298N driver uses two direction pins (IN1, IN2) and one PWM enable pin (ENA) per motor:

- IN1 / IN2 set the direction (forward or reverse)
- PWM on ENA sets the speed (0 to 255)

One `setMotor()` function handles direction, speed and the reversed flag, and every higher-level move (`driveMotors`, `pivotLeft`, `pivotRight`, `stopMotors`) is built on it.

### 3. Left-wall following

The bot tries to stay about 8.4 cm from the left wall.

1. If the front distance is below 8 cm, the bot stops.
2. If there is no left wall (distance above 14 cm), it drives straight.
3. Otherwise it calculates `error = left distance - 8.4`.
4. The correction is `KP x error` (KP = 2.5), limited to +/- 30 PWM.
5. Wheel speeds become:
   - left wheel = base speed - correction
   - right wheel = base speed + correction

If the bot drifts **away** from the wall (error > 0), the left wheel slows and the right wheel speeds up, steering it back toward the wall. If it drifts **too close** (error < 0), the opposite happens.

When the front distance is below 18 cm, both wheels are slowed by 30 PWM so the bot has more room to react at higher speed.

### 4. Turning

Turns are timed pivots: one wheel forward, the other in reverse. The bot spins at a faster speed for most of the turn, then drops to a slower speed for the last 100 ms to reduce overshoot. Turn durations (`LEFT_TURN_90_MS`, `RIGHT_TURN_90_MS`) must be calibrated on the actual maze floor.

## Test modes

Set `TEST_MODE` at the top of the code before uploading:

| Mode | What it does |
|------|--------------|
| 0 | Motor test: each motor, both motors, and pivots |
| 1 | Ultrasonic test: prints left, front and right distances |
| 2 | Straight-drive test: drives forward for 10 seconds |
| 3 | Left-wall-following test |
| 4 | Turn test: a left and a right 90-degree turn |

Recommended order: 0, 1, 2, 4, then 3.

## Getting started

1. Install the Arduino IDE and the ESP32 board package (version 3.x, since the code uses `analogWrite`).
2. Wire the robot using the tables above, including the voltage dividers.
3. Open `code/micromouse_maze_solver/micromouse_maze_solver.ino`.
4. Set `TEST_MODE`, select your ESP32 board and port, and upload.
5. Open the Serial Monitor at **115200 baud** to see readings and debug output.

## What I learned

- **Voltage levels matter.** The HC-SR04 runs at 5 V but the ESP32 is a 3.3 V device, so the ECHO pins need voltage dividers to avoid damaging the board.
- **Sensor data is noisy.** Taking the median of three readings gave much steadier distances than a single reading.
- **Sign errors break feedback control.** An early version of my wall-following steered away from the wall instead of toward it because the correction signs were inverted, which made the drift grow. Working out which wheel must speed up or slow down for a given error fixed it.
- **Higher speed needs more reaction room.** When I increased the motor speed, the stop distance was no longer enough, so I added a slowdown zone before the hard stop.
- **Timed turns depend on speed.** Changing the turn speed changes how far the bot rotates in a fixed time, so turns must be recalibrated whenever speed or battery level changes.
- **Test in small pieces.** Separate test modes for motors, sensors, straight driving and turns made it much easier to find which part was wrong.

## Limitations and future improvements

- The current wall-following code stops at a front wall. Full maze solving needs junction logic that picks a turn (for example, the left-hand rule) and then continues.
- Turn timings are open-loop. Adding wheel encoders or an IMU (gyroscope) would give more accurate and repeatable turns.
- Reading three sensors with three pings each, plus delays, makes each control update slow. Reading fewer pings, or only the sensors that are needed, would speed up the loop.
- Add a flood-fill algorithm so the bot can map the maze and take the shortest path.

## Credits

Built during the workshop conducted by IEEE-VIT. The base code and project idea come from the workshop; testing, tuning and modifications are my own.

## Author

Shubh Disawal

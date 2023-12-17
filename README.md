
# Two wheel differential robot

The project goal was to create a simple mobile robot with a use of 3D printing, which uses basic components commonly used in robotics.

## Components

All devices are connected with wires. Mechanical parts are 3D printed in PLA and connected with M3 screws.
- Microcontroller STM32F411RE Nucleo board
- Motor driver L298N
- 2x The cheapest DC motors + wheels
- Distance sensor HC-SR04p
- BT module HC-05
- 2x Battery cells Li-ion Samsung INR18650-26J 2600mAh + basket
- Ball Caster 1/2'' Pololu 953

## Features

- Wireless control via bluetooth
- Drive forward, backward, left, right
- Distance measurement
- Drive and maintain set distance from the obstacle before the robot


## Programming

##### Whole program was written with a use of HAL library. Each functionality is in seperate files in my_lib folder:
- Button.c/h
- complex_parser.c/h 
- HCSR04p.c/h 
- L298N.c/h 
- Robot.c/h 
- utils.c/h

Used peripheriales:
- GPIO 
    - Motors revolution direction
    - Nucleo blue button to enable/disable robot operation
- Timers 
    - PWM to control motors speed and to trigger distance sensor measurement
    - Input capure to measure pulse width from distance sensor echo pin and convert it to distance value
- USART
    - Bluetooth SPP communiation with HC-05 module to send commands from terminal
## Software

- STM32CubeIDE with built-in CubeMX ver 1.13.2 - programming and configuration
- Realterm - terminal to send commands via BT
- Fusion 360 - 3D modeling
## Commands

```http
  Enable robot
```

| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `ENABLE=X`      | `string` | Enable robot operation, where **X** is a 1 (enable) or 0 (disable) |


```http
  Move robot
```

| Parameter | Type     | Description                |
| :-------- | :------- | :------------------------- |
| `MOVE=F` | `string` | Move forward |
| `MOVE=B` | `string` | Move backward |
| `MOVE=L` | `string` | Move left |
| `MOVE=R` | `string` | Move right |

```http
  Hold distance mode
```

| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `HOLD=XX.X`      | `string` | Maintain distance from obstacle where **XX.X** is a numerical distance value |


## Documentation

[Motor drawing](https://cdn.sparkfun.com/datasheets/Robotics/DG01D.pdf)

[HC-05](https://components101.com/sites/default/files/component_datasheet/HC-05%20Datasheet.pdf)

[HC-SR04](https://web.eece.maine.edu/~zhu/book/lab/HC-SR04%20User%20Manual.pdf)

[L298N](https://cnc1.lv/PDF%20FILES/L298N%20Motor%20Driver%20manual.pdf)




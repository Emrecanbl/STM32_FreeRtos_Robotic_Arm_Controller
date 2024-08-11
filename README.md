STM32 FreeRTOS Robotic Arm Controller

This project is an implementation of a 3-axis robotic arm with a gripper, controlled by an STM32 microcontroller using FreeRTOS. The primary objective is to apply FreeRTOS features to manage real-time tasks in an embedded system, thereby gaining practical experience with real-time operating systems.

Project Purpose
The project is designed to explore the use of FreeRTOS in controlling a robotic arm. By utilizing FreeRTOS features such as counting semaphores, mutexes, queues, and hooks (for stack overflow and malloc failure), this project demonstrates effective management of complex, time-sensitive tasks. The robotic arm is controlled via a Virtual COM Port (VCP) using UART, allowing for synchronized and coordinated movement of all axes.
 
![Sample](https://github.com/Emrecanbl/STM32_FreeRtos_Robotic_Arm_Controller/blob/main/rsz_11723388000535.jpg?raw=true)

Key Features

#FreeRTOS Task Management: The system leverages FreeRTOS to handle multiple tasks concurrently, ensuring that the robotic arm's movements are smooth and synchronized.

#Counting Semaphores: Implemented to manage resource availability and synchronize task execution, particularly when accessing the UART interface.

#Mutexes: Used to safeguard shared resources, preventing data corruption and ensuring thread safety.

#Queues: Facilitates efficient inter-task communication, allowing tasks to exchange data with minimal delay.

#Hooks for Reliability: Implements stack overflow and malloc failure hooks to catch and handle errors during runtime, increasing system robustness.

#Synchronized Motion Control: Ensures that all motors driving the robotic arm move in unison, providing precise control over the arm's position.

#3-Axis Control with Gripper: The robotic arm features three degrees of freedom, controlled by SG90 servo motors, and a gripper for object manipulati

![Sample](https://github.com/Emrecanbl/STM32_FreeRtos_Robotic_Arm_Controller/blob/main/8zxgha.gif?raw=true)

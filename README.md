# AUTOSAR Port Driver 
AutoSAR Port Driver for TivaC - README

Version: 1.0
AutoSAR Version: 4.0.3
Author: [Dina Salah]
Date: [08/05/2024]

---

## Introduction

This README provides an overview of the AutoSAR Port Driver implementation for TivaC microcontrollers. This driver is developed in compliance with AutoSAR version 4.0.3 standards.

---

## Contents

1. Overview
2. Features
3. Usage
4. Configuration
5. Dependencies
6. License

---

## 1. Overview

The AutoSAR Port Driver for TivaC provides an abstraction layer for handling digital input/output (I/O) ports on TivaC microcontrollers. It allows the user to configure and control the behavior of GPIO pins, including setting pin directions, reading input values, and writing output values.

---

## 2. Features

- Abstraction of GPIO pin functionalities
- Configurable pin directions (Input/Output)
- Support for reading input values
- Support for writing output values
- Configurable pull-up/pull-down resistors (if supported by hardware)
- Error handling mechanisms
- Low-level hardware abstraction for port access

---

## 3. Usage

To use the AutoSAR Port Driver for TivaC in your project:

1. Include the necessary header files in your project.
2. Initialize the port driver module.
3. Configure the desired pins with their respective directions (Input/Output).
4. Utilize the provided APIs to read input values, write output values, or configure pull-up/pull-down resistors as needed.

Example code snippet:

```c
#include "Port.h"

int main() {
    Port_Init(&Port_Configuration); // Initialize Port Driver
    Port_SetPinDirection(PORT_CHANNEL_A, PIN_0, PORT_PIN_OUT); // Configure pin as output
    Port_WritePin(PORT_CHANNEL_A, PIN_0, STD_HIGH); // Set pin output to high

    while (1) {
        // Your application code here
    }

    return 0;
}

4. Configuration
The port driver can be configured through the provided configuration file (Port_Cfg.h). This file allows the user to customize various aspects of the driver, including pin configurations, error handling, and hardware-specific settings.

5. Dependencies
TivaWare Peripheral Driver Library (if applicable)
AutoSAR version 4.0.3 compliant compiler and toolchain
6. License




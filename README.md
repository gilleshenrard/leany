<!--
SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>

SPDX-License-Identifier: MIT
-->

# Leany - An open-Source Inclinometer

Leany is a fully open-source, production-ready inclinometer based on an ARM Cortex-M3 32-bits microcontroller.
It provides high-precision tilt measurements while adhering to strict embedded software and hardware design standards.

## About the Project
An inclinometer is a tool that measures how much something is tilted, specifically in two directions: pitch and roll.

- Pitch measures the tilt forward or backward. Imagine tilting your head down to look at your feet—that’s pitch.
- Roll measures the tilt side to side. It’s like tilting your head left or right to look over your shoulder—that’s roll.

So, if you're using an inclinometer, it can tell you exactly how much something is tilted forward, backward, or side to side. For example, if you're setting up equipment, you can use the inclinometer to make sure it’s positioned correctly, with no unwanted tilt in either direction. It’s useful in many situations, from construction to vehicles, ensuring that everything is set at the right angle for safety and precision.

Leany is designed for engineers, makers, and professionals who need a reliable, open, and customizable inclinometer. It offers precise tilt measurements and multiple operating modes while maintaining a strong focus on code quality, hardware reliability, and ease of integration.

### Key Features

- **Measurements**: Supports pitch and roll rotation axes with a precision of up to **0.1°**.
- **Hold Function**: Freezes screen updates to keep a measurement visible.
- **Slope Mode**: Measures angles relative to gravity (**absolute measurements**).
- **Angle Mode**: Measures angles relative to a user-defined zero position (**relative measurements**).

## Getting Started

Before contributing, please run the following :
```
git config --local include.path ../.gitconfig
```

To learn more about each part of the project, check the respective directories:

- **[Firmware (`firmware/`)](firmware/)** – Contains the firmware source code and instructions on how to build and flash it.
- **[PCB Design (`pcb/`)](pcb/)** – Contains the schematics and board design, along with instructions on generating production files.
- **[Documentation templates (`templates/`)](templates/)** – Contains example files to illustrate the structure documentation files should have.

## Contributions & Community

Leany is managed with the help of a **[Github Project](https://github.com/users/gilleshenrard/projects/3)**

Leany is fully open-source, and contributions are welcome! If you find a bug, have a suggestion, or want to improve the project, feel free to open an issue or submit a pull request.

Discussions are also encouraged—whether it's feature ideas, improvements, or troubleshooting. Join the community and help make Leany even better!

## License

This project is licensed under the **MIT License**, allowing for flexible use and modification.

---

Feel free to suggest any changes or enhancements to this README! 🚀

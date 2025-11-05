<!--
SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>

SPDX-License-Identifier: MIT
-->

# Leany Firmware

Firmware for **Leany**, an inclinometer project based on the **STM32F103 (ARM Cortex-M3)** microcontroller.

**Version:** 0.1

## Building the Firmware

1. Install the required tools by running the appropriate prerequisites installation script in `resources/`.
2. Run the following command, with either **Debug** or **Release** as a configuration name :
```
cmake --preset Release
cmake --build --preset Release
```
3. When the build task is complete, the firmware binary file (*.elf) can be found under build/<configuration_name>/

## Programming the device
### USB (programming only)
1. Plug the device to a computer with a USB type C cable
2. Press and maintain the BOOT button down
3. Press the RESET button
4. Release the BOOT button
5. Upload the Leany.elf file to the device with the STM32CubeProgrammer software in USB mode

### Hardware programmer (programming and debugging)
1. Wire the programmer to the 4-pin male SWD connector on the PCB
2. Connect the programmer to a computer via USB
3. Flash and debug the firmware with an IDE of your choosing

## USB Command-Line Communication

The device supports bidirectional command-line communication over **USB Type-C**, appearing as a **virtual COM port**.  
When connected, look for a port name containing **`CH340`**, which indicates the onboard USB–serial bridge.

### Features

- **Command Protocol:**  
  A custom protocol inspired by **SCPI** is implemented to allow both **read** and **write** operations.  
  Note: The current implementation does *not yet* fully comply with the SCPI standard.

- **Logging System:**  
  The firmware outputs log messages prefixed with an exclamation mark (`!`).  
  Multiple **logging levels** are available to adjust verbosity as needed.

### Example Session

```text
:imu:ki?
0.5
:imu:ki 1.2
!Mahony kI updated
:imu:ki?
1.2
```

## CI/CD workflows
The firmware is instected by a [Github Action](https://github.com/gilleshenrard/leany/actions/workflows/firmware_build_lint.yml) upon push and pull request to ensure the software compiles and the linters are all ran.

## Project Structure

```
firmware/
├── CMakeLists.txt
├── CMakePresets.json
├── Leany.ioc
├── hardware/
├── dispatcher/
├── ui/
└── resources/
    ├── .vscode/
    |   ├── extensions.json
    |   ├── launch.json
    |   ├── settings.json
    |   └── tasks.json
    ├── Doxyfile
    ├── install_prerequisites_ubuntu.sh
    ├── install_prerequisites_windows.bat
    └── check_filenames.sh
```

### `CMakeLists.json` and `CMakePresets.json`
This project is built with CMake to ensure maximum portability across different environments and OSes.

A toolchain file and CMake preset configurations are provided to make building the firmware as easy as possible.

### `Leany.ioc`
The **STM32CubeMX** project file used to configure the STM32F103 microcontroller. It generates all third-party libraries, including LL (Low Level) and CMSIS.

### `resources/`
Contains additional resources:
- **`Doxyfile`**: Configuration for Doxygen documentation generation.
- **`install_prerequisites_ubuntu.sh`**: Script to install everything needed to build the firmware on Ubuntu.
- **`install_prerequisites_windows.bat`**: Script to install everything needed to build the firmware on Windows.
- **`check_filenames.sh`**: Script to make sure the filenames formatting is respected.

### `resources/.vscode/`
Contains project-specific VSCode configuration and useful tasks. Should someone want to use it, they simply need to copy .vscode/ to the root of firmware/ :
- **`extensions.json`**: List of recommended VSCode extensions to use with this project. They will be suggested by VSCode if not already installed.
- **`launch.json`**: Configuration for OpenOCD to use an STLink to debug the application
- **`settings.json`**: Project-specific settings. They will override the user's pre-existing settings, and will ensure everyone uses the same configuration.
- **`tasks.json`**: Some useful custom tasks for VSCode on Windows and Ubuntu.

### `dispatcher/`
Contains the implementation of the events and messages queues dispatcher. This is where the business logic is.

### `hardware/`
Contains the implementation of the hardware modules (e.g. GPIO buttons, MEMS sensor, ...)

### `UI/`
Contains the implementation of the user interface

## Formatting, Code Quality, Linting, Documentation and licensing

- Google-style formatting is used and enforced with **clang-format** and **clang-tidy**
- Strict code quality is maintained with as many **GCC warnings** as possible enabled, all treated as errors.
- Linters configured: **clang-tidy**, **cppcheck**, **flawfinder**, and **lizard**.
- **Doxygen** is used for documentation, with strict control to ensure comprehensive code documentation.
- Licensing and copyrighting compliance is checked with **REUSE tool**

## Contributing

Contributions to improve the firmware code and documentation are welcome. If you have any suggestions or improvements, please create a pull request or open an issue on the repository.

## License

Leany is licensed under the MIT license. See the LICENSE/ directory for more information.

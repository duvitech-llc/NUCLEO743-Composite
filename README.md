# NUCLEO-H743 USB Composite Device (HID Mouse + VCP)

![STM32 NUCLEO-H743ZI](https://www.st.com/bin/ecommerce/api/image.PF266989.en.feature-description-include-personalized-no-cpn-large.jpg)

This project demonstrates a **USB Composite Device** on the **STM32 NUCLEO-H743ZI** board, combining:
- **HID Mouse** (for simulating mouse movements)
- **Virtual COM Port (VCP)** (for serial communication)

## Features
- ✅ **HID Mouse** - Sends simulated mouse movements to the host
- ✅ **VCP (Virtual COM Port)** - Allows serial communication (UART over USB)
- ✅ **USB Composite Device** - Combines both interfaces into a single USB descriptor
- ✅ **STM32CubeMX Configuration** - Ready-to-use `.ioc` file for easy customization

## Hardware Requirements
- NUCLEO-H743ZI development board
- USB Micro-B cable (for connecting to a PC)

## Software Requirements
- STM32CubeMX (for configuration & code generation)
- STM32CubeIDE / Keil / IAR (for compilation & flashing)
- Terminal emulator (e.g., PuTTY, Tera Term, or `screen` for VCP testing)

## Setup & Flashing

### 1. Generate Code with STM32CubeMX
1. Open the `.ioc` file in STM32CubeMX
2. Verify USB settings:
   - **USB_OTG_FS** in **Device Mode**
   - **HID + CDC (VCP)** enabled in Middleware
3. Generate code (select your preferred IDE)

### 2. Build & Flash
1. Open the project in your IDE (STM32CubeIDE/Keil/IAR)
2. Build the project (`Ctrl+B` or `Build`)
3. Flash to the NUCLEO-H743 (`Run` or `Flash`)

### 3. Connect to PC
1. Plug the board into your PC via USB
2. The device should enumerate as:
   - **HID Mouse** (check in Device Manager)
   - **Virtual COM Port** (e.g., `COMx` on Windows, `/dev/ttyACMx` on Linux)

## Testing

### HID Mouse
The firmware automatically sends mouse movements ot the right. Observe the cursor moving on your screen.

### VCP Communication
1. Open a terminal emulator (e.g., PuTTY, Tera Term, `screen`)
2. Set the correct COM port and baud rate (`115200`)
3. The device outputs Hello when user button is pushed.

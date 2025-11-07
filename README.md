📘 Section 0 – Main Firmware (Main Board)

This folder contains the main control firmware for the LT4 system’s main board.
Key features include:

🔥 Temperature sensing from the onboard sensor.

⚡ Triac control for AC load switching.

🕛 Zero-cross detection for synchronized control with the AC power line.

🧩 Task management and I2C communication between modules.

This firmware is designed to run stably on the LT4 main board, handling all core logic and communication between peripheral modules.

👉 Note:
For hardware testing purposes, this code implements only the basic functions of each feature.
However, a more complete and flexible library has been pre-written to support advanced requirements and extended features.

📘 Section 1 – Test Code

This folder contains individual test programs for each feature from Section 0.
These are used to verify and debug hardware functions, such as:

Temperature reading

Zero-cross detection

Triac control

You can flash these test programs directly to an Arduino Uno or any other compatible board using the same framework.
Just make sure to adjust the I2C pins to match your hardware configuration.
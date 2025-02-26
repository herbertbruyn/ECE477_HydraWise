# ECE477_HydraWise

Welcome to HydraWise! HydraWise is a **wearable hydration monitoring device** designed to help athletes and individuals track their electrolyte levels and hydration status in real-time. Using a conductivity-based sweat analysis system, this device measures water levels and heart rate to provide actionable hydration insights via an iOS app. 

## Features
- Real-time sweat conductivity measurement using two electrodes
- Heart rate monitoring with the MAX30101 PPG sensor
- Wireless Bluetooth Low Energy (BLE) connectivity to transmit data to an iOS app
- User-friendly dashboard for hydration tracking
- Historical data storage for trend analysis and insights

## System Components
### Hardware
- ESP32-WROOM (Microcontroller with BLE capabilities)
- MAX30101 (PPG Sensor for heart rate monitoring)
- LFS1107 (Conductivity Sensor for sweat measurement)
- Rechargeable Lithium-Ion Battery

### Software
- Firmware: Developed in C/C++ using the ESP-IDF framework
- iOS App: Built with Xcode using Swift and SwiftUI
- Bluetooth Communication: Bluetooth Low Energy (BLE) for data transfer
- Data Processing: Signal filtering and hydration analysis using Python and Swift

### Data Flow
- Sweat conductivity data is captured by the LFS1107
- Heart rate data is collected using the MAX30101 PPG sensor
- ESP32 microcontroller processes the raw signals
- Processed data is sent via BLE to the iOS app
- The iOS app analyzes and displays hydration insights in real-time
- Users can track trends and receive alerts for dehydration risk

## Usage
1. Wear the HydraWise device on your wrist or upper arm
2. Connect to the iOS app via Bluetooth
3. Start an activity (e.g., running, cycling) to generate sweat
4. The device will measure sweat conductivity and track heart rate
5. View real-time hydration insights on the HydraWise iOS app
6. Receive alerts for hydration needs, battery levels, and heart rate monitoring.

## Contributors
- Areej Mirani - Team Lead
- James Neff - Software Lead
- Matthew Dailey - Hardware Lead
- Herbert Alexander DeBruyn - Systems Lead

## License
This project is licensed under the MIT License.
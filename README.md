# 🛩️ Flight Monitoring System for Radio-Controlled Aircraft

## 📋 Overview

This project implements a **comprehensive flight monitoring and telemetry system** for radio-controlled (RC) aircraft. It provides real-time data acquisition, SD card logging, and wireless transmission of critical flight parameters, including altitude, temperature, and sensor readings.

## ✨ Key Features

- **📡 Real-Time Telemetry** : Continuous monitoring of aircraft flight parameters
- **💾 Data Logging** : SD card storage for post-flight analysis
- **📊 Multi-Sensor Support** : Integration with various environmental and orientation sensors
- **🔌 Wireless Communication** : Remote data transmission via RF modules
- **⚙️ Arduino-Based** : Portable and customizable implementation using Arduino platforms

## 📁 Project Structure

```
Flight-Monitoring-for-radiocontrolled-aircraft/
├── 📄 README.md                    # This file
├── 📂 src.1/                       # Main source code
│   └── Arduino sketches and files
├── 📂 Sensors tests/               # Sensor testing and calibration
│   └── Validation scripts
└── 📂 docs/                        # Complete documentation
    └── 📊 REPORT (ESSENTIAL READING)
```

## 📊 REPORT - Essential Document

⚠️ **A detailed technical report is available in the `/docs` folder**

This comprehensive report contains:
- **Complete system analysis** and architecture overview
- **Technical documentation** of all sensors and modules used
- **Detailed test results** and performance validation
- **Step-by-step implementation guide** for deployment
- **Hardware specifications** and wiring diagrams
- **Communication protocols** and data formats
- **Recommendations** for future improvements and scaling
- **Troubleshooting guide** for common issues

### **IMPORTANT: Consult the report for a comprehensive understanding of the project**

The report is your primary resource for:
- Understanding the complete system design
- Implementing the system correctly
- Troubleshooting issues
- Optimizing performance

---

## 🚀 Quick Start Guide

### Prerequisites

- **Arduino IDE** (version 1.8.0 or higher)
- **Arduino Microcontroller** (compatible with your setup)
- **Required Libraries** (see `/docs` for complete list)
- **RF Communication Module** (RF24, LoRa, or compatible)
- **SD Card Module** and microSD card (up to 32GB recommended)
- **Sensors**:
  - Altimeter/Barometric pressure sensor
  - Temperature sensor
  - IMU (Inertial Measurement Unit) for orientation
  - Additional sensors as needed

### Installation Steps

#### 1. Clone the Repository

```bash
git clone https://github.com/HyDrakuul/Flight-Monitoring-for-radiocontrolled-aircraft.git
cd Flight-Monitoring-for-radiocontrolled-aircraft
```

#### 2. Review Documentation

Start by reading:
1. **The main report** in `/docs` - Start here!
2. **Sensor specifications** in `/docs`
3. **Hardware wiring guide** in `/docs`

#### 3. Prepare Your Hardware

- Gather all required components (see report)
- Follow wiring diagrams from `/docs`
- Verify all connections before powering on

#### 4. Load and Configure the Code

- Open Arduino IDE
- Navigate to `/src.1`
- Open the main sketch file
- Configure parameters for your hardware:
  ```cpp
  // Example configuration
  #define RF_CHIP_SELECT_PIN 10
  #define RF_INTERRUPT_PIN 2
  #define SD_CHIP_SELECT_PIN 4
  ```
- Upload to your Arduino board

#### 5. Test Sensors

Before flight, validate each component:

```bash
# Test altitude sensor
/Sensors tests/test_altitude.ino

# Test temperature sensor
/Sensors tests/test_temperature.ino

# Test orientation/IMU
/Sensors tests/test_orientation.ino

# Test SD card logging
/Sensors tests/test_sd_logging.ino

# Test RF communication
/Sensors tests/test_rf_module.ino
```

---

## 🔧 Configuration Guide

### Hardware Configuration

Before deployment, ensure:

1. **RF Module Setup**
   - Configure correct SPI pins (see schematic in `/docs`)
   - Set appropriate RF frequency
   - Test transmission range in open area

2. **SD Card Module**
   - Insert microSD card (formatted as FAT32)
   - Verify chip select pin configuration
   - Test write speed and capacity

3. **Sensor Calibration**
   - Calibrate pressure sensor at known altitude
   - Verify temperature sensor accuracy
   - Level and calibrate IMU gyroscope

4. **Power Supply**
   - Verify voltage levels for all modules
   - Use appropriate voltage regulators
   - Add decoupling capacitors where needed

### Software Configuration

Edit the configuration section in `/src.1/main.ino`:

```cpp
// RF Module Configuration
#define RF_FREQUENCY 2400  // MHz
#define RF_CHANNEL 76
#define RF_POWER_LEVEL RF24_PA_MAX

// Sensor Configuration
#define ALTITUDE_SEA_LEVEL_PRESSURE 1013.25  // hPa
#define TEMPERATURE_OFFSET 0.0  // °C
#define LOG_INTERVAL 100  // milliseconds

// SD Card Configuration
#define SD_LOG_FILENAME "flight.csv"
#define SD_MAX_FILE_SIZE 10485760  // 10 MB
```

---

## 📚 Complete Documentation

The `/docs` folder contains comprehensive documentation:

| Document | Purpose |
|----------|---------|
| **System Report** | Complete technical analysis and design |
| **Hardware Guide** | Component list, schematics, wiring diagrams |
| **Sensor Specs** | Detailed specifications for all sensors |
| **API Reference** | Function documentation and usage |
| **Protocols** | Data format and communication protocols |
| **Calibration Guide** | Sensor calibration procedures |

---


### Running Tests

1. Select one test file
2. Open in Arduino IDE
3. Configure pins to match your setup
4. Upload and open Serial Monitor
5. Follow on-screen instructions
6. Record results


## 🎯 Use Cases

- **✈️ Competition Monitoring**: Real-time tracking of drone performance during races
- **📈 Performance Analysis**: Collect detailed flight data for post-flight optimization
- **🔍 Aerodynamic Studies**: Research and validate aerodynamic models
- **🎓 Educational Projects**: Learn about embedded systems and telemetry
- **📡 Wireless Experiments**: Test long-range RF communication
- **🔬 Research**: Collect scientific data for environmental or atmospheric studies

---

## 📊 Data Output Format

### SD Card Log (CSV)

```
timestamp,altitude,temperature,humidity,pitch,roll,yaw,rf_signal
1000,125.34,23.5,45,0.12,-0.05,0.03,-75
1100,125.67,23.6,45,0.10,-0.07,0.05,-76
1200,125.95,23.7,46,0.08,-0.09,0.07,-74
```

## 🛠️ Future Improvements

Planned enhancements and optimization opportunities:

- **Enhanced UI**: Develop mobile app for real-time monitoring
- **Data Encryption**: Implement secure wireless communication
- **Machine Learning**: Anomaly detection for flight safety
- **Sensor Fusion**: Integrate multiple sensor data for accuracy
- **Cloud Sync**: Upload flight logs to cloud storage
- **Battery Monitoring**: Add voltage and current tracking
- **Redundancy**: Implement failsafe mechanisms

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

1. **Report Bugs**
   - Document the issue clearly
   - Include hardware setup details
   - Provide serial monitor output

2. **Suggest Improvements**
   - Describe your enhancement
   - Explain the benefits
   - Include code examples if applicable

3. **Share Modifications**
   - Fork the repository
   - Make your changes
   - Submit a pull request with documentation

4. **Improve Documentation**
   - Fix typos and unclear sections
   - Add examples and clarifications
   - Translate documentation


## 📖 Additional Resources

- **Arduino Official Documentation**: https://www.arduino.cc/
- **RF Module Datasheets**: See `/docs`
- **Sensor Specifications**: See `/docs`
- **Community Forums**: Arduino forums and GitHub discussions

---

## 💡 Important Notes

### Before Any Deployment

⚠️ **CRITICAL**: Read the **technical report in `/docs`** for:
- Complete system understanding
- Proper hardware assembly
- Correct software configuration
- Safety considerations
- Best practices and recommendations

### Data Privacy

- Ensure SD cards are securely stored
- Protect RF transmissions if transmitting sensitive data
- Comply with local data protection regulations

---

**Last Updated**: March 2026  
**Project Repository**: https://github.com/HyDrakuul/Flight-Monitoring-for-radiocontrolled-aircraft




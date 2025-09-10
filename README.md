# Project Guardian: The Future of Road Safety 

An Automatic Crash Notification (ACN) system for proactive emergency response.

---

## The Problem

Every year, thousands of lives are lost in car accidents because emergency services are not alerted in time. Traditional methods rely on a driver who may be unconscious or unable to call for help, which can be the difference between life and death.

## The Solution

Project Guardian is a self-contained IoT device that automatically detects a car crash and sends an instant alert with precise location data to a designated contact or emergency service.

### Key Features
* **Real-time Crash Detection:** Our device uses a multi-layered sensor fusion algorithm to accurately detect high-impact collisions.
* **Autonomous GPS Location:** It automatically acquires the exact latitude and longitude of the crash site.
* **Instant SMS Alert:** The system sends an SMS with all critical information to ensure a rapid emergency response.

---

## How It Works: System Architecture 

Our system is a self-contained unit built on a simple yet effective architecture.

![Project Guardian System Diagram](https://i.imgur.com/your-diagram-image.png)

1.  **Sensor Input:** The MPU6050 (accelerometer/gyroscope) constantly measures G-force.
2.  **Crash Algorithm:** The ESP32 microcontroller processes the sensor data. If a G-force threshold is exceeded for a sustained period, it triggers the alert sequence.
3.  **Location Acquisition:** The GPS module gets the precise location coordinates.
4.  **Alert Transmission:** A GSM module transmits the alert message to the pre-configured number.

---

## Virtual Prototype & Demo 

Since this is a hackathon project, we have created a virtual prototype to demonstrate the core functionality.

[Watch the Demo Video Here](https://www.youtube.com/watch?v=your-demo-video-id)

**How to run the simulation yourself:**

1.  Go to the [Wokwi Virtual Prototype](https://wokwi.com/projects/your-project-id) link.
2.  Click "Start Simulation."
3.  Interact with the virtual accelerometer to simulate a crash and watch the serial monitor output.

---

## Technical Details

### Components Used
* **Microcontroller:** Arduino Uno (simulated)
* **Sensors:** MPU6050 (accelerometer/gyroscope)
* **Connectivity:** Simulated GPS & GSM Module
* **Visual Output:** NeoPixel RGB LED (to indicate status)

### Code
The source code for the project is available in the [`src/`](/src/) directory. The main logic is in the `project-guardian-acn.ino` file.

---

## Future Roadmap 

* **Vehicle Integration:** Integrate with the OBD-II port to retrieve additional vehicle data (e.g., speed, fuel level).
* **Mobile Application:** Develop a companion app for user configuration and real-time status monitoring.
* **Cloud-Based Dashboard:** A centralized dashboard for emergency services to monitor multiple alerts and dispatch teams.

---

## Team

* **Your Name:** [Your GitHub Profile Link]
* **Team Member Name:** [Team Member GitHub Profile Link]

---

### **License**

This project is open-sourced under the MIT License.# project-AcciResQ
An IoT device for automatic crash notification.

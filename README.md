# 🐭 Micromouse Robot – Flood Fill Algorithm (ROBOFEST 2025)

An autonomous **Micromouse robot** developed for **SLIIT ROBOFEST 2025 – University Category**, capable of exploring a 16×16 maze, mapping walls in real time, reaching the goal using the **Flood Fill algorithm**, and returning safely to the start position.

This project integrates **embedded systems, sensor fusion, control theory, and algorithmic path planning** into a compact and efficient robotic platform.

---

## 🚀 Project Overview

The Micromouse starts at cell **(0,0)**, explores an unknown maze autonomously, detects walls using infrared sensors, builds an internal map, and navigates toward the center goal cells using a **dynamic flood-fill approach**.  
After reaching the center, it switches state and **returns to the start cell** using a return flood map.

The robot is built around an **ESP32** and uses **encoder-based motion control** and **gyro-assisted turns** for accurate navigation.

---

## ✨ Key Features

- **Fully Autonomous Navigation**
  - No prior knowledge of the maze
  - Real-time wall detection and mapping

- **Flood Fill Path Planning**
  - Dynamic flood map updates for a 16×16 maze
  - Goal defined as the center 4 cells
  - Optimized movement based on minimum flood values

- **State-Based Operation**
  - `EXPLORING` → Reach maze center  
  - `RETURN_TO_START` → Navigate back safely  
  - `FINISHED` → Stop execution

- **Precision Motion Control**
  - N20 motors with encoders for accurate cell traversal
  - PD control for straight-line motion
  - Gyroscope-based angle correction for turns

- **Robust Sensor Fusion**
  - 5× Sharp IR distance sensors
  - Front and side wall detection
  - Emergency front-stop safety logic

- **Efficient Power Management**
  - Dual Li-Po battery setup
  - Buck converter for stable voltage regulation

---

## ⚙️ Hardware Components

- **ESP32 Development Board**
- **ESP32 WROOM-DA Module**
- **Sharp GP2Y0A51SK0F IR Sensors ×5**
- **N20 Motors (6V, 500 RPM) with Encoders**
- **34 mm N20 Motor Wheels**
- **MPU6050 Gyroscope & Accelerometer**
- **TB6612FNG 1.2A Motor Driver**
- **MP1584 Mini Buck Converter**
- **3.7V 1200mAh Li-Po Batteries ×2**
- Switches, wiring, and custom chassis

---

## 🧠 Algorithm Details

### Flood Fill (16×16 Maze)

- The maze is represented using:
  - `cellsArray[16][16]` → Wall encoding per cell
  - `floodArray[16][16]` → Distance to goal
  - `returnFloodArray[16][16]` → Distance back to start

- Flood values are updated dynamically using a **queue-based consistency check**.
- Movement decisions prioritize:
  1. Forward
  2. Left
  3. Right
  4. Backward  
  (based on orientation and minimum flood value)

### Orientation Encoding

```text
FORWARD  = 0
RIGHT    = 1
BACKWARD = 2
LEFT     = 3
````

---

## 🔌 Pin Configuration

Motor Driver (TB6612FNG):

* `AIN1` → GPIO 19
* `AIN2` → GPIO 18
* `PWMA` → GPIO 5
* `BIN1` → GPIO 17
* `BIN2` → GPIO 16
* `PWMB` → GPIO 4
* `STBY` → GPIO 23

Encoders:

* Left Encoder → GPIO 14
* Right Encoder → GPIO 26

IR Sensors:

* 90° Left  → GPIO 32
* Front Left → GPIO 33
* Front Right → GPIO 34
* 90° Right → GPIO 35

---

## 🛠️ Software & Libraries

* **Arduino Framework**
* **ESP32 Core**
* **Adafruit MPU6050**
* **Adafruit Unified Sensor**
* Standard C++ STL (`queue`, `math.h`)

---

## ▶️ How to Run

1. Open `micromouse_floodfill.cpp` in **Arduino IDE**
2. Select:

   * Board: **ESP32 Dev Module**
   * Baud Rate: **115200**
3. Install required libraries:

   * Adafruit MPU6050
   * Adafruit Sensor
4. Upload the code to the ESP32
5. Place the robot at maze start cell `(0,0)`
6. Power on and observe autonomous exploration

---

## 📌 Notes

* Sensor calibration is critical for reliable wall detection
* Thresholds may need tuning based on lighting and maze material
* `TURN_SIGN` can be adjusted if turning direction is reversed
* Emergency stop triggers if a front obstacle is detected too close

---

## 👥 Team Acknowledgment

This project was made possible through the combined efforts, dedication, and problem-solving mindset of an amazing team:

* **Ayani Atapattu**
* **Tharini Jayarathna**
* **Chanya Shehani**
* **Sawani Weerathunga**

Every test run, bug fix, and improvement was a shared learning experience 🚀

---

## 📷 Competition

**SLIIT ROBOFEST 2025 – University Category**
Micromouse Autonomous Maze-Solving Robot

---

## 📜 License

This project is intended for **educational and research purposes**.
Feel free to explore, learn, and build upon it with proper credit.

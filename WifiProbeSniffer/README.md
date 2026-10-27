# ESP32 Wi-Fi Probe Sniffer

## 1. Overview
An ESP32 project that listens to nearby Wi-Fi traffic for 802.11 probe request frames. It uses the payload from captured packets to extract useful information, including timestamp, Wi-Fi channel, RSSI, the device's average RSSI, estimated distance (cm), MAC address, and SSID. It saves this information to SPIFFS as a CSV file, which can be retrieved later by another device running `retrieval.py`.
- **Disclaimer:** For educational and research use only.

## 2. Hardware Design
- Uses an ESP32 microcontroller from ELEGOO.
- The onboard blue LED flashes each time a relevant packet is captured.
- Powered via USB-C and stores logs in internal SPIFFS memory.

## 3. Software Architecture
- **Packet Sniffer Core:** When powered on, the ESP32 goes into promiscuous mode to listen for packets, hopping through channels 1–13. When a packet is received, it checks that it is the correct type of packet. If it is, it extracts the information from the payload.
- **Data Logger:** Once the data has been extracted from a packet, the program queues it in a buffer. Periodically, the buffer is flushed and the data is written to SPIFFS.
- **Retrieval Script:** A Python script that sends the `GET_CSV` command and saves the returned CSV.

## 4. Data Format
- CSV schema: `timestamp_ms, channel, rssi (dBm), avg_rssi (dBm), distance_cm, mac, ssid`
- Estimated distance is derived from RSSI using the formula `distance_cm = 100 * 10^((RSSI_AT_1M - avg_rssi) / (10 * n))`, where `avg_rssi` is the device's average signal strength, `RSSI_AT_1M` is the expected signal strength at 1 meter, and `n` is the path-loss exponent, which represents how quickly a radio signal weakens as it travels through space.

## 5. Retrieval Workflow
- Make sure you have pyserial installed (`pip install pyserial`).
- Connect the ESP32 to your laptop.
- Set the serial port in `retrieval.py` (e.g. `COM3` on Windows).
- Run `retrieval.py` to get the CSV.

## 6. Challenges and Design Choices
- Initially, I encountered a large number of corrupted and irrelevant packets, which led to implementing a frame-type filter.
- I originally planned for a Bluetooth-connected companion app to receive live data, but storage constraints on the ESP32 made SPIFFS logging a more practical solution.

## 7. Ethics & Legal Use
- This project is for **educational demonstration only**.
- Captures only **broadcast probe frames**, not personal communications.
- Do not use to track or deanonymize individuals.
- Always verify local regulations before operating in public areas. This project is intended for controlled lab or demo environments.
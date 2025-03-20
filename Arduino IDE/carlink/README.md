# carlink

### UPLOADING TO ESP32
- Board: "ESP32 Dev Module"
- Port: /dev/cu.usbserial-ABC23ETC
- Upload Speed: < 921600 
    - 921600 may cause error similar to: "A fatal error occurred: Unable to verify flash chip connection (Packet content transfer stopped (received 7 bytes))"
    - 460800 should work fine
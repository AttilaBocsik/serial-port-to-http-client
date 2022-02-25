# HTTP Device Client

## Description:
Receiving a string stream from a UART2 serial port.
Incoming data from UART 2 serial port is even forwarded to UART1 serial port.
The data is sent in application/x-www-form-urlencoded format via http to the API.


## Hardware:
- RS232-TTL converter module
- NodeMCU32S
- LED(RED, GREEN, YELLOW) 3mm, 1.8...2.4V, 20mA
- Resistor 3 x 330 ohm

### Schematic:
    RS232-TTL converter module              NodeMCU ESP32
                      VCC        <==>        5V 
                      GND        <==>        GND
                      TX         <==>        GPIO16  RX2 
                      RX         <==>        GPIO17  TX2
    Receive UART device
                      VCC        <==>        5V 
                      GND        <==>        GND
                      TX         <==>        GPIOP9  RX1 
                      RX         <==>        GPIOP10 TX1

               LED(RED_LED)(+)   <==>        GPIO5
               LED(GREEN_LED)(+) <==>        GPIO4
              LED(YELLOW_LED)(+) <==>        GPIO18

## Software:
- FreeRTOS(ESP-IDF) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html

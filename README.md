# ambilight_tv
Prerequisites:

```pip install -r requirements.txt```

Running the Python Service:

```python ambilight_main.py```

Deploying to ESP8266:

* Open the Arduino IDE and select NodeMCU ESP8266 as the board.
* Install and include the Adafruit_NeoPixel library.
* Connect your NodeMCU to the LED strip mounted behind the TV.
* Load ambilight_v1_arduino_wifi.cpp, then compile and upload to the board.
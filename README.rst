CircuitPython library for the QMI8658 IMU as found in the Waveshare RP2040-LCD-1.28 <https://www.waveshare.com/wiki/RP2040-LCD-1.28>.

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Installation
============

Copy the jepstone_qmi8658.py file to your CircuitPython device's lib\ subdirectory (such as ``D:\lib`` on Windows or ``/Volumes/CIRCUITPY`` on macOS). If needed, install the necessary requirements on your CircuitPython device with :code:`circup install adafruit_bus_device adafruit_register`.

Usage Example
=============

.. code-block: python3

    import busio
    from board import *
    from jepstone_qmi8658 import QMI8658

    SDL = GP7  # Change if needed
    SDA = GP6

    with busio.I2C(SDL, SDA) as i2c:
        device = QMI8658(i2c)
        print(device.temperature)
        print(device.acceleration)
        print(device.gyro)


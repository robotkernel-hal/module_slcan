# Module SerialCAN

This module provides acecess to SerialCAN devices.

## Configuration

The '''module_slcan''' can be loaded with the following configuration:

 - name: my_pcan_master
   so_file: libmodule_pcan.so
   config:
     tty_name: /dev/ttyUSB1
     baudrate: 250000
     slave_modules: [ module1, module2, ... ]
   power_up: op                                                                
   depends: timer


# Module SerialCAN

This module provides acecess to SerialCAN devices.

## Configuration

The '''module_slcan''' can be loaded with the following configuration:

```yaml
# Configuration file for module_slcan
#
# vim: ft=yaml

#########################################################
# logging settings

# Standard robotkernel module local loglevel.
#loglevel: verbose

#########################################################
# slcan settings

# Name of TTY device node
tty_name: /dev/ttyUSB1

# CAN baudrate to set
baudrate: 250000

# Trigger to do cyclic send of available packets.
#trigger: { dev_name: timer.main.trigger }

# CAN Slave device stream, received packets will be passed
# to and send packets are read from.
slave_streams: [ iboss_0.serial.stream ]

#########################################################
# slcan receive thread settings

# Thread priority, receive thread will be spawned with
# this priority and SCHED_FIFO
prio: 60

# Thread affinity, specifies CPU/Core number to schedule
# receive thread on.
affinity: [ 3, ]

# Explicit specify thread name
#thread_name: my_thread_name
```

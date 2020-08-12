# bit_foot
## low cost high speed foot pressure sensor
This board measures 4 differential voltage signals from load cells. It connects to a RS485 or TTL Bus compatible with Dynamixel motors from Robotis.

We managed to achieve a sensor update rate of 697Hz. The board itself can be read faster than 1kHz from the Dynamixel bus.

Strain gauges connection:
P1: Back Right
P2: Back Left
P3: Front Right
P4: Front Left


![bitfoot](bitfoot.png)

This board is based on the [ForceFoot](https://github.com/Rhoban/ForceFoot) by Rhoban

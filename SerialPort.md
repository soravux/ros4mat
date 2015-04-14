Besides its multiple sensors support, ROS4MAT is able to remotely control a serial port. This page explains how to use this feature and which limitations apply.

# Generalities #

ROS4MAT allows to control a remote serial port. However, some limitations apply :
  * Only 1 KB (1024 bytes) can be sent or received at the time (however, you may send or receive more data by using several consecutive calls).
  * As with all other sensors, there is absolutely no maximum delay guarantee (while it is usually fast, it can largly vary depending on the network used)


# Serial command #

The generic ros4mat command to send or receive from a given serial port is the following :

```
data = ros4mat('serie', PORT, baud_rate, parity, stop_bits, DATA, send_length, receive_length, timeout_sec, timeout_usec, close_conn);
```

Where :
  * **PORT** is the port name (a string, like '/dev/ttyS0')
  * **baud\_rate** the requested transfer rate, in bauds (integer, like 57600)
  * **parity** sets the parity bit on or off (boolean, 1 to send a parity bit)
  * **stop\_bits** indicates the number of stop bits wanted (can be 1 or 2)
  * **DATA** is the data to send (it can be empty if you just want to receive from this port). It should be an array of bytes (see the next section).
  * **send\_length** tells how many bytes should actually be sent (at least the length of **DATA**)
  * **receive\_length** tells how many bytes should be received from the port.
  * **timeout\_sec** and **timeout\_usec** can be used to set a timeout on the data reception. If set to 0, ros4mat will wait indefinetly for **receive\_length** bytes. If it is a positive integer value, then ros4mat will not wait more than timeout\_sec + timeout\_usec/1000000. After that delay, the data received so far will be returned, regardless to number of bytes actually received. Note that no error is emitted when the timeout is triggered.
  * **close\_conn** is a boolean which tells ros4mat to close (or not) the serial port after the communication. If set to **0**, then the connection will remain open (and the following communications will be quite faster). If set to **1**, the connection will be closed.

Note that the combination **receive\_length=0 and send\_length=0** is a special one, which empty the receive and send buffers on the robot. It can be useful to get rid of the garbage data produced by some devices in the initialization process.

# How to format your data #

The data should be sent as an array of bytes. For standard ASCII characters, you may just use standard matlab string (e.g. 'test'). For non-displayable characters (like 0x80), it is recommanded to use sprintf to decode the hexadecimal value (e.g. sprintf('\x80')). For the null value, simply write 0. All of those methods can be combined in a standard array, for instance :

```
[sprintf('\x89') 0 245 'Test']
```

# Practical case : control a Roomba #

```
roombaSerPortName = '/dev/ttyS0';
roombaBaudRate = 57600;
roombaDelaySec = 1;
roombaDelayUSec = 100;


disp('Establishing connection to Roomba...');

% Start 128
ros4mat('serie', roombaSerPortName, roombaBaudRate, 0, 1, sprintf('\x80'), 1, 0, roombaDelaySec, roombaDelayUSec, 0);
pause(0.02)  % Needed for a proper initialization of the Roomba

% Full mode 132
ros4mat('serie', roombaSerPortName, roombaBaudRate, 0, 1, sprintf('\x84'), 1, 0, roombaDelaySec, roombaDelayUSec, 0);
pause(0.02)

% Power LED only 139 0 0 128
ros4mat('serie', roombaSerPortName, roombaBaudRate, 0, 1, [sprintf('\x8B') 0 0 sprintf('\x80')], 4, 0, roombaDelaySec, roombaDelayUSec, 0);
pause(0.02)

% Set song 140 1 1 48 20
ros4mat('serie', roombaSerPortName, roombaBaudRate, 0, 1, [sprintf('\x8C') 1 1 sprintf('\x30\x14')], 5, 0, roombaDelaySec, roombaDelayUSec, 0);

% Sing it 141 1
ros4mat('serie', roombaSerPortName, roombaBaudRate, 0, 1, [sprintf('\x8D') 1], 2, 0, roombaDelaySec, roombaDelayUSec, 0);
```
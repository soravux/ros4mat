This tutorial assumes that you successfully installed ROS4MAT on both server and client side (see the [relevant documentation](Installation.md) on this wiki).

# Basic Principles #

ROS4MAT is based upon some design principles :
  * Send the data only when the user wants it
  * No data is not a failure
  * Keeping data the user does not explicitly want is useless
  * Time synchronization is important
  * Do not keep states on user data or commands

These mere principles have lead the API choices, and should be kept in mind by the user.

# What ROS4MAT is not #

While ROS4MAT can be a very useful framework for many applications, it is not suitable for all kinds of applications. In particular :
  * It does not provide **any** security. The data is transmitted without encryption, and the commands received are executed without any identity verification.
  * It does not guarantee a maximal transmission delay. Due to the _pull_ approach used, the data can be retrieved a long time after its actual capture. Timestamps correctness is preserved, though.

# First Example #

Let's begin with a fairly simple example, the capture of a analog signal from an ADC (in our case, the [DI-149](http://www.dataq.com/products/startkit/di149.htm)) :

```
ros4mat('connect', 'IP', 1);
```
We first establish a connection to the server. The IP should be replace by the actual one of the robot. The last flag tells ros4mat to either use zlib compression (1) or not (0, default). Activating compression will reduce the amount of data transferred at the expense of CPU usage.

```
ros4mat('subscribe', 'adc', 100, 1, 100);
```
Once we are successfully connected to the robot, we may _subscribe_ to a sensor. This command tells the server part to launch the corresponding node and to start collecting data. Let's take a look at the parameters :

  1. The first parameter is the sensor name. The available sensors and their corresponding name are listed in a specific page.
  1. The second is the capture rate, in hertz (in this case, 100 Hz). If this rate exceeds the sensor maximum rate, it will be set to this maximum value.
  1. The third one is sensor-specific : in this case, it tells ROS4MAT to sample only the first ADC channel.
  1. The fourth parameter is the ring buffer size on the server side. Since the data is only sent when the user explicitly ask for it, it accumulates on the server side. To avoid any buffer overflow, a ring buffer is used. This parameter allows the user to tune the size of this ring buffer. It is optional, and defaults to 1 second of data (100 samples with a 100 Hz acquisition rate, 1000 if we use a 1 kHz rate, etc.)

Please check the [Sensors reference](SensorsReference.md) for further sensor subscription information.

We are now subscribed to a sensor. Depending on the nature of this sensor, it may need an initialization time. After that, we can begin to retrieve the data captured :

```
[data, timestamps] = ros4mat('adc');
```
This command will retrieve all the available data on the server, and return all samples with their associated timestamps. The data is a pure Matlab type, and can be used as any other Matlab matrix :
```
plot(data(1,:)); % Plot the channel 1 value
```

The number of samples returned will never be greater than the ring buffer size defined in the previous step. However, _there is no guarantee that it will be this length:_ if the ring buffer is not filled, then the number of samples returned will be smaller. If two retrieving commands are called back to back, the second may even return an empty matrix, meaning that there is no other available data at the moment.

When we do not need a sensor anymore, we can optionally notify ROS4MAT :

```
ros4mat('unsubscribe', 'adc');
```
This will empty the buffers and allow the server the stop the corresponding nodes if there are no more user subscribed to a given sensor, thus saving resources. However, all sensors will be automatically unsubscribed when the client disconnect:

```
ros4mat('close');
```
This call properly closes the connection. No subsequent commands can be issued, unless a new connection is made.

# Other examples #

Examples are provided in the [example section](https://code.google.com/p/ros4mat/source/browse/#hg%2Fexamples) of the repository and should help you start with your projects.
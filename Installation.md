

# Requirements #

## Server ##

The ROS4MAT server is any ROS-enabled data emitter, typically a robot or a data acquisition machine. A ROS4MAT server needs the following software:

  * ROS Electric, Fuerte or Groovy (required)
  * [uvc\_camera](http://www.ros.org/wiki/uvc_camera) package (optional, needed for camera input)
  * [Hokuyo\_node](http://www.ros.org/wiki/hokuyo_node) (optional, needed for Hokuyo rangefinder devices)
  * [Openni\_kinect](http://www.ros.org/wiki/openni_kinect) (optional, needed for Kinect input)
  * [gps\_common](http://www.ros.org/wiki/gps_common) package (optional, needed for GPS devices)

All the aforementioned packages may often be installed with one meta-package (for instance _ros-groovy-desktop-full_ for ROS Groovy on Ubuntu).

## Client ##

The ROS4MAT client is the data receiver, typically a user workstation or laptop.

  * Matlab 2009b or newer
  * [A Matlab compatible C compiler](http://www.mathworks.com/support/compilers/R2013a/index.html)

Note that the The Windows version comes with an embedded LCC compiler (the user still have the possibility to use another compiler). Currently, ROS4MAT wrapper have been succesfully tested with GCC 4.5+, Visual Studio 2010 and 2012 and LCC, on both 32 and 64 bits architectures.

# Server Setup #

In the ROS4MAT terminology, the server is the part on the robot sending data and receiving commands.

The ROS4MAT server part relies on ROS. There is no hard dependencies on a given ROS packages, but some packages are mandatory to use some sensors (see the requirements section).

The processing power required by ROS4MAT is fairly low, so the server can run on a low-end computer. For instance, it can be a netbook where a Linux distribution is running from an USB pendrive.

## Step 1 : Compilation ##

As the server is actually a ROS node, it can easily be compiled with the _rosmake_ command in a shell prompt :

```
rosmake ros4mat
```

## Step 2 : Execution ##

A [launch file](http://www.ros.org/wiki/roslaunch) is [provided](https://code.google.com/p/ros4mat/source/browse/#hg%2Fros4mat) to start all the relevant nodes to execute ROS4MAT. To run it, execute the following command in a shell prompt :

```
roslaunch ros4mat ros4mat.launch
```

The ROS4MAT server is now functional.

# Client Setup #

In the ROS4MAT terminology, the client side is the part with which the user interacts. It relies on a working Matlab installation, but has no other dependencies.

## Step 1 : Setup Mex compilation ##

First, make sure that your Matlab mex compilation setup is correct. You can modify your compiler and its settings by issuing the following command in a Matlab prompt:

```
mex -setup
```

## Step 2 : Compilation ##

Make sure your current working directory in Matlab is the [matlab folder](https://code.google.com/p/ros4mat/source/browse/#hg%2Fmatlab) of the ROS4MAT project.

You can then compile the _ros4mat.c_ mex file by executing the following command in a Matlab prompt:

### Linux ###

```
mex ros4mat.c
```

### Windows ###

On Windows, the socket library must be linked in order to compile properly, like so :

```
mex ros4mat.c wsock32.lib
```

## Step 3 : Adding ROS4MAT to the path ##

Please refer to your version of [Matlab documentation on paths](http://www.mathworks.com/help/matlab/ref/path.html) to add the ROS4MAT compiled mex library to your Matlab path in order to use ros4mat elsewhere than its directory.

# Functionality test #

Once the installation steps are completed on both the server and the client and the ROS4MAT server is up and running, you may use ROS4MAT :

```
ros4mat('connect', 'IP')
```

(replace _IP_ by the actual IP of the server)

If no error messages are issued, your installation is working! You may start to [use ROS4MAT](FirstSteps.md). Otherwise, check for connectivity problems (try to ping the robot from the client station) or security restrictions (like strict firewall rules).
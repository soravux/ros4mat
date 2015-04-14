# Subscription parameters #

The ros4mat subscription call use at most six arguments :
```
ros4mat('subscribe', 'ROS4MAT Name', freq_capture, first_param, buffer_size, freq_polling, second_param, ...);
```

| **Sensor** | **ROS4MAT Name** | **First param value** | **Second param value** | **Third param value** | **Fourth param value** | **Fifth param value** | **Sixth param value** |
|:-----------|:-----------------|:----------------------|:-----------------------|:----------------------|:-----------------------|:----------------------|:----------------------|
| Analog to digital converter (ADC) | adc | Channels to sample<sup>*</sup> | - | - | - | - | - |
| IMU (accelerometer/gyro/magnetometer) | imu | - | - | - | - | - | - |
| GPS | gps | - | - | - | - | - | - |
| Camera (webcam) | camera | Resolution, in string, for instance '640x480' | Camera ID X<sup>†</sup> | JPEG compression quality<sup>‡</sup> | Camera exposure<sup>◊</sup> | - | - |
| Stereo camera | camera\_stereo | Resolution, in string, for instance '640x480' | Camera 1 ID<sup>†</sup> | Camera 2 ID<sup>†</sup> | JPEG compression quality<sup>‡</sup> | Left camera exposure<sup>◊</sup> | Right camera exposure<sup>◊</sup> |
| Kinect | kinect | Resolution, in string, for instance '640x480' | Kinect ID (unsupported) | JPEG compression quality of RGB<sup>‡</sup> | - | - | - |
| Hokuyo range-finder | hokuyo | - | - | - | - | - | - |

`*` This integer value is a [bit mask](http://en.wikipedia.org/wiki/Mask_(computing)) representing the desired channels. Each bit represents one channel, for instance 17 = 16 + 1 = (2<sup>4</sup>) + (2<sup>0</sup>) represents the first and fifth channels (index 0 and 4). It may be easier to figure in binary where 17 in decimal is 10001 in binary, thus representing directly the desired channels.

`†` The Camera ID is their reference number X in /dev/videoX.

`‡` A value of zero (0) means images are sent using no compression (raw lossless bitmap). JPEG Quality is set using a value between 1 and 100, where 1 is the lowest quality and 100 the highest.

`◊` Exposure is an UVC parameter that takes a value between 1 and 10000 where 10000 is the darkest and 1 is the brightest.

# Default values #

| **Parameter** | **Default Value** |
|:--------------|:------------------|
| buffer\_size | equals to freq\_capture (1 second) |
| freq\_polling | freq\_capture / 4 + 10 |
| Camera resolution | 320x240 |
| Compression | Deactivated |
| Exposure | 300 |

# Returned values #

The data can be retrieved (after a subscription) by calling ros4mat with the sensor name:

```
[data_adc, timestamps] = ros4mat('adc');
```

Here are the values returned for each sensor :

| **Sensor** | **ROS4MAT Name** | Return format |
|:-----------|:-----------------|:--------------|
| Analog to digital converter (ADC) | adc | [data, timestamps] where data is a m\*n matrix where _m_ is the number of samples and _n_ the numbers of channels |
| IMU (accelerometer/gyro/magnetometer) | imu | [data, timestamps] where data is a m\*9 matrix where _m_ is the number of samples. The first three columns are for the accelerometer, columns 4-6 contains the gyroscope data and columns 7-9 the magnetometer data. |
| GPS | gps | [data, timestamps] where data is a m\*7 matrix. Column 1 is the GPS state (0 = no fix, 1 = 2D fix, 2 = 3D fix), column 2 the latitude, col. 3 the longitude, col. 4 the altitude, col. 5 and 6 the module and angle of the horizontal speed and col. 7 the vertical speed |
| Camera (webcam) | camera | [images, timestamps] where _images_ is a 4D array, the last dimension being the image index |
| Stereo camera | camera\_stereo | [images\_left, images\_right, timestamps] where the cameras output are synchronized. The format is the same as for the single camera. |
| Kinect | kinect | [images, depth\_map, timestamps] where _images_ is as above and _depth\_map_ is a 4D array containing the depth of every image point in mm. |
| Hokuyo | hokuyo | [data, timestamps, infos] where data is a m\*n matrix, _m_ being the number of samples and _n_ the number of points acquired by the hokuyo. The info array contains some informations about the scan, in order : minimum angle, maximum angle, angle increment, minimum range, maximum range and time increment. |
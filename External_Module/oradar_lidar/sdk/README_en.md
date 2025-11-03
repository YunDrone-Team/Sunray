
# MS200 SDK Basic Introduction
The MS200 SDK is a software development kit designed specifically for the Oradar MS200 LIDAR product. With the MS200 SDK, users can quickly connect to the Oradar MS200 LIDAR and receive LIDAR point cloud data.

# Operational requirements
- Linux system: Ubuntu 14.04 LTS, Ubuntu 16.04 LTS, Ubuntu 18.04 LTS
- Windows 7/10
- C++ 11 compiler
- CMake, version 3.5 or higher

# Compilation and installation methods
First, extract the sdk package, and the extracted file name is "sdk".

Use following commands in Linux:

```
cd sdk
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=out ..
make
sudo make install
```

Use following commands in Windows:

(Here is a windows 10 system, the compiler is QT MinGW for example. You need to import the compiler installation path into the system environment variable)

Hold down the shift key and right mouse button to open the powershell
```
cd sdk
mkdir build
cd build
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=out ..
mingw32-make -j8
mingw32-make install
```
Generate the out folder under the current path, which contains library files, header files, and executable files.


# API Description of SDK Main Functions
|Function name | Function introduction|
|---------|---------------|
|Connect| Check and open the LiDAR serial port, and create the LiDAR serial port read thread|
|Disconnect| Close the read thread of the LiDAR serial port, and close the serial port|
|GrabOneScan           |  Get Lidar latest packet of point cloud data, non-blocking. Point cloud data contains angle, distance and intensity information for all points|
|GrabOneScanBlocking   |  Get Lidar latest packet of point cloud data, blocking. Point cloud data contains angle, distance and intensity information of all points|
|GrabFullScan          |  Get Lidar latest circle of point cloud data, non-blocking. Point cloud data contains angle, distance and intensity information of all points|
|GrabFullScanBlocking  |  Get Lidar latest circle of point cloud data, blocking. Point cloud data contains angle, distance and intensity information of all points|
|GetRotationSpeed      |  Get Lidar latest motor speed|
|SetRotationSpeed      |  Set Lidar motor speed|
|GetTimestamp          |  Get Lidar timestamp of the latest package|
|GetFirmwareVersion    |  Get Lidar firmware version number of the top and bottom board|
|GetDeviceSN           |  Get LiDAR Device SN number|
|Activate              |  LiDAR enters ranging status from standby status|
|Deactive              |  LiDAR enters standby status from ranging status|


# Sample Usage Instructions
Linux:

Connect the MS200 LIDAR device to Ubuntu system via USB to serial cable, open the terminal under Ubuntu system, use "ls /dev/ttyUSB*" command to see if the serial device is connected, if the serial device is detected, use "sudo chmod 777 /dev/ttyUSB*" command to give the highest privilege.
Then execute SDK Sample, enter the following command:

```
cd sdk/build
./blocking_test 
```
OR
```
cd sdk/build
./non-blocking_test
```

OR

```
cd sdk/build
./blocking_c_api_test
```

Note: If the command "ls /dev/ttyUSB*" looks at the device and * is not 0, you need to replace the device name in the Samsple test code from "/dev/ttyUSB0" to the corresponding device name(The port_name variable in the code is modified).

Windows:

Connect the MS200 LIDAR device to the computer via the USB, plug into the Windows PC. Through the device manager to view the serial port name, such as "com10", you need to modify the sample code in the port_name variable to com10, and recompile test code.
Then, click `blocking_test.exe` or `non-blocking_test.exe` or `blocking_c_api_test.exe` to check operation results.
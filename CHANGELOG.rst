^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for the Beckhoff ROS2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (Forthcoming)
-------------------
* Ported to C++ (rclcpp), with the same configuration file, topic and service
* ADS client made for low latency, without dependencies: requests are pipelined, every variable
  gets a symbol handle at startup and several variables are read or written with ADS sum
  commands. A read or a write is a single request (3 before), and a Batch of reads and writes is
  executed in one round trip
* Buffer, View and Field: access to the data of the variables without conversions nor memory
  allocations; asynchronous requests; latest values of the monitored variables kept from their
  notifications (latest, waitForUpdate, onUpdate); cyclic notifications
* The connection is re-established when it is lost, and the handles and notifications are created
  again when the PLC program changes
* The sizes of the variables are checked against the PLC at startup
* Release build by default, benchmark tool
* The configuration file is given with the config_file parameter / launch argument
* Fixed the crash at startup when variables are monitored
* Fixed the values published for elementary types and read for arrays of elementary types
* write_variable can write structures and arrays, and sets its result
* Values that do not fit the PLC types are rejected
* Added more elementary types and the optional structure packing
* Added unit tests and tests against a simulated PLC

1.0.0 (2022-10-18)
------------------
* Fixed a bug where lists of custom DUTs were not handled correctly
* Added a method to read all variables being monitored
* added short packge description and usage to README

0.9.0 (2021-09-19)
------------------
* Initial Release

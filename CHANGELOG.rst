^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for the Beckhoff ROS2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (Forthcoming)
-------------------
* Ported to C++ (rclcpp and Beckhoff's ADS library), with the same configuration file, topic and service
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

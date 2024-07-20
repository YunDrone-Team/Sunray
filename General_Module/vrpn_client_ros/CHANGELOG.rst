^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vrpn_client_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2017-11-30)
------------------
* Fixup find_package
* Contributors: Paul Bovbel

0.2.1 (2017-11-29)
------------------
* Fix warnings; update maintainer email; switch to package format 2
* Contributors: Paul Bovbel

0.2.0 (2017-11-15)
------------------
* Merge pull request `#8 <https://github.com/ros-drivers/vrpn_client_ros/issues/8>`_ from cjue/indigo-devel
  fix: motive auto generated names are not valid ROS identifiers
* fix: don't ignore second char when first one is illegal
* handlerules for first and subsequent chars in ROS names
* fix problem with auto generated names from motive: "Rigidy Body n" is
  not a valid ROS identifier
* Contributors: Christian Juelg, Paul Bovbel

0.1.1 (2016-10-27)
------------------
* Remove -Werror, upstream VRPN has warnings
* Contributors: Paul Bovbel

0.1.0 (2016-10-26)
------------------
* Add separate publishers for each sensor id
* Add Possibility to Append Sensor Id to TF Client Frame
* Update 'supported devices' link in README
* Contributors: Karljohan Lundin Palmerius, Paul Bovbel

0.0.2 (2015-07-07)
------------------
* Fix tf header timestamps
* Update sample.launch
* Contributors: Paul Bovbel

0.0.1 (2015-06-22)
------------------
* Rename to vrpn_client_ros
* Simplify tracker update logic; add blacklist for VRPN Control
* Break out separate nodes for client and tracker, and clean up build
* Initial prototype
* Contributors: Paul Bovbel

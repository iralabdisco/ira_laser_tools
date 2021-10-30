^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ira_laser_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.0.7 (2021-10-30)
------------------
* Use the same header in published scan as the published cloud 
* Fix a concurrency issue
* Contributors: JackFrost67, MikHut, Auri

1.0.6 (2021-08-12)
------------------
* add check for PCL version
* add new mantainer email
* fix launch before laser scan is available
* add a try catch for first non valid tf
* Retrying compare topics between ROS master and Token
* Contributors: JackFrost67

1.0.4 (2020-02-14)
------------------
* fix worning
  "<command-line>:0:0: warning: missing whitespace after the macro name"
* change email maintainer
* Contributors: Pietro Colombo

1.0.3 (2020-02-13)
------------------
* fix `#14 <https://github.com/iralabdisco/ira_laser_tools/issues/14>`_
  now we use math library for pi
* bugfix
* Merge branch 'kinetic'
* fix issue fix`#14 <https://github.com/iralabdisco/ira_laser_tools/issues/14>`_
* Create LICENSE
* Create LICENSE
* Merge pull request `#11 <https://github.com/iralabdisco/ira_laser_tools/issues/11>`_ from mikaelarguedas/patch-1
  add libvtk-qt dependency to fix debian stretch builds
* add libvtk-qt dependency to fix debian stretch builds
* Contributors: Mikael Arguedas, Pietro Colombo, pietrocolombo

1.0.2 (2018-08-28)
------------------
* add libvtk-qt dependency to fix debian stretch builds
  and link to paper in README.md
* Contributors: Pietro Colombo

1.0.1 (2018-07-18)
------------------

1.0.0 (2018-07-11 10:30:16 +0200)
---------------------------------
* add url for wiki in pakage.xml
* adding license
* introduce a param in param file
  expand the range limit of laser
  and minor bug fix
* Merge pull request `#1 <https://github.com/iralabdisco/ira_laser_tools/issues/1>`_ from leonziegler/installprocedure
  Added possibility to install artifacts.
  Merged. Thanks :-)
* Merge branch 'master' of projects.ira.disco.unimib.it:/repository/git/ira_laser_tools
  Conflicts:
  CMakeLists.txt
* this commit fixes `#265 <https://github.com/iralabdisco/ira_laser_tools/issues/265>`_ @5m
* Fixed the Eigen3 required in the CMakeLists.txt
* updating readme
* removing eigen dependency
* Update laserscan_virtualizer.launch
  description of base_frame param. updated
* Added possibility to install artifacts.
* changes to README.md
* global refactoring
* Merge branch 'master' of https://github.com/iralabdisco/ira_laser_tools
* going to version 1.0; useful tutorial on .launch file for laservirtualizer; the overall virtualscanner procedure is now simpler
* First README.md version
* adding readme file
* Fixed the include bug
* First commit
* Contributors: Augusto Luis Ballardini, Axel Furlan, Fabio Nava, Iralab Universita Milano Bicocca, Leon Ziegler

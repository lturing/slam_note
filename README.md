
* [ "Large velocity, reset IMU-preintegration!" #220 ](https://github.com/TixiaoShan/LIO-SAM/issues/220)   
> My IMU data is from INS570D and lidar data is from RS-LIDAR 16, the default coordinate of IMU is "x-forward, y-right, z-down" and the default coordinate of RS-LIDAR is "x-right, y-forward, z-up", i have changed both of them to "x-forward, y-left, z-up", but the result is still bad.

* [ large velocity or bias, reset IMU-preintegration! #29 ](https://github.com/TixiaoShan/LVI-SAM/issues/29) 

* [how to use the SaveAtlas() and LoadAtlas() for relocalization? #128
](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/128)
> there is a bug in MapPoint::PreSave, detele KeyFrame when iter,you should fix it ,and then fix Tracking::Relocalization bug,and write a new function InitWithMap,and you can use the atlas map

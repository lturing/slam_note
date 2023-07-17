
* [ "Large velocity, reset IMU-preintegration!" #220 ](https://github.com/TixiaoShan/LIO-SAM/issues/220)   
> My IMU data is from INS570D and lidar data is from RS-LIDAR 16, the default coordinate of IMU is "x-forward, y-right, z-down" and the default coordinate of RS-LIDAR is "x-right, y-forward, z-up", i have changed both of them to "x-forward, y-left, z-up", but the result is still bad.

* [ large velocity or bias, reset IMU-preintegration! #29 ](https://github.com/TixiaoShan/LVI-SAM/issues/29) 

* [how to use the SaveAtlas() and LoadAtlas() for relocalization? #128
](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/128)
> there is a bug in MapPoint::PreSave, detele KeyFrame when iter,you should fix it ,and then fix Tracking::Relocalization bug,and write a new function InitWithMap,and you can use the atlas map

* [Load/Save Map #443](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/443)

* [Load map but with no point showed in the viewer windows,just KeyFrames #737](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/737)

* [https://github.com/UZ-SLAMLab/ORB_SLAM3/pull/310/files](https://github.com/UZ-SLAMLab/ORB_SLAM3/pull/310/files)

* [https://github.com/gaoxiang12/ORB-YGZ-SLAM/issues/3](https://github.com/gaoxiang12/ORB-YGZ-SLAM/issues/3)

---- 

* imu initialzed
> you should move in a curved trajectory to make the SLAM initialize well. http://doc.openxrlab.org.cn/openxrlab_document/ARDemo/ARdemo.html


-------------
* keyframe selection
> When a new frame comes, we check the parallax of its keypoint matches with respect to the last keyframe. If the parallax exceeds a threshold, we tag the new frame as a keyframe. If the number of matches is below a lower bound, or there have not been any keyframes for the recent T frames, we also mark the frame as a keyframe.

-----------

* [tum imu dataset](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
  

## slam
- [Det-SLAM: A semantic visual SLAM for highly dynamic scenes using Detectron2](https://arxiv.org/pdf/2210.00278.pdf)
- [DL-SLOT: Dynamic Lidar SLAM and Object Tracking Based On Graph Optimization](https://arxiv.org/pdf/2202.11431.pdf)
- [PVI-DSO: Leveraging Planar Regularities for Direct Sparse Visual-Inertial Odometry](https://arxiv.org/pdf/2204.02635.pdf)
- [RDS-SLAM: Real-Time Dynamic SLAM Using Semantic Segmentation Methods](https://sci-hub.st/https://ieeexplore.ieee.org/document/9318990)
- [Semantic SLAM With More Accurate Point Cloud Map in Dynamic Environments](https://sci-hub.st/https://ieeexplore.ieee.org/document/9119407)
- [Dynamic SLAM: The Need For Speed](https://sci-hub.st/https://ieeexplore.ieee.org/document/9196895)
- [SiLK: Simple Learned Keypoints](https://arxiv.org/pdf/2304.06194.pdf)
- [VIO 初始化探究：一种旋转平移解耦的高效鲁棒初始化方法](https://mp.weixin.qq.com/s/tQP6CoROi6pqpr9xLyoMVg)
- [自动驾驶闭环最常用的算法—卡尔曼滤波](https://mp.weixin.qq.com/s/rw5VjtoEdO-7xBDfQ6Qtpw)
- [Double Window Optimisation for Constant Time Visual SLAM](https://sci-hub.st/https://ieeexplore.ieee.org/document/6126517)
- [Automatic Image selection in Photogrammetric Multi-view Stereo methods](https://discovery.ucl.ac.uk/id/eprint/1369912/1/paper1034_Hosseininaveh_VAST2012SecondEdition_30_10_2012_AAM.pdf)
- **[ios recorder](http://doc.openxrlab.org.cn/openxrlab_document/ARDemo/ARdemo.html)**

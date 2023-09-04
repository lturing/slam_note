
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

-------
* [在ORB-SLAM2的基础上，结合三维目标检测，实现目标跟踪，动态特征点剔除的object-SLAM方法](https://www.bilibili.com/video/BV1gP411m7Hf)


-----------
* [Direct-ORB-SLAM: Direct Monocular ORB-SLAM](https://iopscience.iop.org/article/10.1088/1742-6596/1345/3/032016/pdf)
> if a frame is not a keyframe, the features extracted in it are only used to estimate the pose of the frame and just a few tenths of features take part in this step. This wastes a lot of time to compute the useless descriptors.

-----------
* [SOF-SLAM: A semantic visual SLAM for Dynamic Environments](https://www.sci-hub.yt/10.1109/ACCESS.2019.2952161)
> features divided into static, dynamic and potentially dynamic, using static features by image segmentation to compute R an t(H matrix), and the epipolar geometry to decide potentially dynamic whether is static or not.
-------------------

* [A Robust RGB-D SLAM System with Points and Lines for Low Texture Indoor Environments](https://sci.bban.top/pdf/10.1109/JSEN.2019.2927405.pdf#view=FitH)
> We observe that the computation of keypoint descriptors in indirect methods is time-consuming and the descriptors are not reused except in the case of keyframes. This wastes significant computational sources. If we can establish reliable keypoint correspondences without extracting descriptors between adjacent frames (or equivalently in Tracking), it will greatly reduce the computational cost without loss of precision. The first stage is for robust keypoint matching where we predict the initial keypoint correspondences by a uniform acceleration motion (UAM) model and then use a pyramid-based sparse optical flow algorithm to establish coarse keypoint correspondences. The second stage is for inlier refinement where we exploit grid-based motion statistics to filter out outliers and then utilize the epipolar constraint to further refine the correspondences.

---------------

## mesh or planar
- [Incremental Visual-Inertial 3D Mesh Generation with Structural Regularities](https://arxiv.org/abs/1903.01067) **[code](https://github.com/MIT-SPARK/Kimera-VIO)**
- [PlanarSLAM](https://github.com/yanyan-li/PlanarSLAM)
- [Efficient_Surface_Detection](https://github.com/AngusKung/Efficient_Surface_Detection)

------------------------------------


## slam
- [Det-SLAM: A semantic visual SLAM for highly dynamic scenes using Detectron2](https://arxiv.org/pdf/2210.00278.pdf)
- [DL-SLOT: Dynamic Lidar SLAM and Object Tracking Based On Graph Optimization](https://arxiv.org/pdf/2202.11431.pdf)
- [DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments](https://arxiv.org/abs/1809.08379)
- [PVI-DSO: Leveraging Planar Regularities for Direct Sparse Visual-Inertial Odometry](https://arxiv.org/pdf/2204.02635.pdf)
- [RDS-SLAM: Real-Time Dynamic SLAM Using Semantic Segmentation Methods](https://sci-hub.st/https://ieeexplore.ieee.org/document/9318990)
- [Semantic SLAM With More Accurate Point Cloud Map in Dynamic Environments](https://sci-hub.st/https://ieeexplore.ieee.org/document/9119407)
- [Dynamic SLAM: The Need For Speed](https://sci-hub.st/https://ieeexplore.ieee.org/document/9196895)
- [SiLK: Simple Learned Keypoints](https://arxiv.org/pdf/2304.06194.pdf)
- [VIO 初始化探究：一种旋转平移解耦的高效鲁棒初始化方法](https://mp.weixin.qq.com/s/tQP6CoROi6pqpr9xLyoMVg)
- [自动驾驶闭环最常用的算法—卡尔曼滤波](https://mp.weixin.qq.com/s/rw5VjtoEdO-7xBDfQ6Qtpw)
- [Double Window Optimisation for Constant Time Visual SLAM](https://sci-hub.st/https://ieeexplore.ieee.org/document/6126517)
- [Automatic Image selection in Photogrammetric Multi-view Stereo methods](https://discovery.ucl.ac.uk/id/eprint/1369912/1/paper1034_Hosseininaveh_VAST2012SecondEdition_30_10_2012_AAM.pdf)
- [动态环境下的slam问题如何解决？](https://www.zhihu.com/question/47817909/answer/2462128186)
- [动态视觉SLAM的亿点点思考](https://mp.weixin.qq.com/s/3mVVTBudTBdLkvHUiuBxbw)
- **[ios recorder](http://doc.openxrlab.org.cn/openxrlab_document/ARDemo/ARdemo.html)**
- [Introducing augmented reality for e-commerce](https://blog.griddynamics.com/introducing-augmented-reality-for-e-commerce/)
- [How ARKit and ARCore recognize vertical planes](https://blog.griddynamics.com/how-arkit-and-arcore-recognize-vertical-planes/)

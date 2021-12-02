
[code](https://github.com/raulmur/ORB_SLAM2/blob/master/src/Optimizer.cc)              
#### BundleAdjustment

```c
void BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMPs,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
作用：优化pose和landmark
参数：
vpKFs：关键帧集(keyframe, pose)
vpMPs：观测点集(mappoint，landmark)
nIteration：优化迭代次数
pbStopFlag：每次迭代前检查是否强制退出，pbStopFlag是引用型，受到其它线程控制
nLoopKF：检测到回环的当前关键帧的mnId
bRobust：是否使用核函数计算loss，使优化更稳定
说明：
- 将vpKFs中的合法keyframe作为顶点添加到优化图中
- 将vpMPs中的合法mapPoint作为顶点添加到优化图中
    - 遍历mapPoint的观测(keyframes)
        - 过滤掉keyframe.mnId大于vpKFs中最大keyframe.mnId
        - keyframe和mapPoint作为边添加到优化图中
    - 若不存在边，则从优化图中删除mapPoint(该观测不会优化)
- 按照迭代次数，迭代优化图，若pbStopFlag为True，则每次迭代前会检查pbStopFlag，若为是否为True，则终止优化
    - sets a variable checked at every iteration to force a user stop. The iteration exits when the variable is true;
- 遍历更新vpKFs中keyframe
    - 若nLoopKF为0(首帧关键帧)，将优化后的pose赋值给keyframe.Tcw
    - 若nLoopKF不为0
        - 将优化后的pose赋值给keyframe.mTcwGBA
        - keyframe.mnBAGlobalForKF = nLoopKF
- 遍历更新vpMPs中的mapPoint
    - 若nLoop为0
        - 将优化后的landmark赋值给mapPoint.mWorldPos
        - 更新mapPoint中的最大距离、最小距离等
    - 若nLoop不为0
        - 将优化后的landmark赋值给mapPoint.mPosGBA
        - mapPoint.mnBAGlobalForKF = nLoopKF
```

#### GlobalBundleAdjustment

```c
void Optimizer::GlobalBundleAdjustment(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
作用：利用函数BundleAdjustment，优化全局地图上的pose和landmark
参数：
pMap：地图
nIteration：BA迭代次数
pbStopFlag：每次迭代前检查是否强制退出，pbStopFlag是引用型，受到其它线程控制
nLoopPK：检测到回环的当前关键帧的mnId
bRobust：是否使用核函数计算loss，使优化更稳定

vpKFs = pMap.GetAllKeyFrames();
vpMPs = pMap.GetAllMapPoints();
BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
```

#### LocalBundleAdjustment

```c
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
作用：以当前pKF为瞄点，优化pKF以及与pKF有共视图关系的关键帧，优化这些关键帧观察到的路标点
参数：
pKF：当前关键帧
pbStopFlag：每次迭代前检查是否强制退出，pbStopFlag是引用型，受到其它线程控制
pMap：地图，优化后上锁，更新pose和landmark，防止冲突
说明：
- 待优化的关键帧，pkF以及与pKF有共视图关系的关键帧，做标记(防止重复)，即对获得的关键帧.mnBALocalForKF = pKF->mnId
- 待优化的路标点，即待优化的关键帧上看到的路标点，做标记(防止重复)，即对获得的路标点.mnBALocalForKF=pKF->mnId
- 优化中固定的关键帧，除了待优化关键帧外，能观察到待优化的路标点的关键帧，做标记(防止重复)，即对获得的关键帧.mnBAFixedForKF=pKF->mnId
- 将待优化的关键帧、待优化的路标点、优化中固定的关键帧作为顶点添加到优化图中，并将关键帧与路标点形成的边添加到优化图中
- 先后执行两次优化
- 第一次优化
    - 对添加的所有顶点以及边进行优化
- 第二次优化
    - 遍历所有的边，将优化后的路标点投影到像素坐标系统中，将误差超过指定阈值的边固定，不优化
    - 取消使用核函数，因为优化后，误差不会很大
    - 对添加的所有顶点以及边进行优化
    - 遍历所有的边，再次将优化后的路标点(内点)投影到像素坐标系统中，将误差超过指定阈值的边做处理
        - 将边上的关键帧删除边上的路标点
        - 将边上的路标点删除边上的关键帧
- 更新关键帧的pose
- 更新路标点
    - 更新3d位置
    - 计算更新最大距离、最小距离
```

#### PoseOptimization

```c
int Optimizer::PoseOptimization(Frame *pFrame)
作用：只优化位姿
参数：
pFrame：待优化的帧
说明：
- 将当前pFrame作为顶点添加到优化图中
- 将当前pFrame上的路标点作为顶点添加到优化中
- 依次将当前pFrame与其上到路标点作为边，添加到优化图中，并固定路标点
- 分几次优化，后几次优化去掉核函数
- 每次优化前，用pFrame的位姿重新赋值给优化图中顶点
- 优化中不断找出异常边，并在后续优化中固定
- 更新pFrame的位姿
```

#### OptimizeEssentialGraph

```c
void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
作用：优化所有的关键帧的sim3，顶点是所有的关键帧，边是部分边（检测到回环时调用）
参数：
pMap：地图
pLoopKF：与当前关键帧pCurKF构成回环的关键帧
pCurKF：当前关键帧
NonCorrectedSim3：关键帧的位姿sim(3)
CorrectedSim3：根据当前关键帧计算出的位姿sim(3)
LoopConnections：去掉有共视图关系的关键帧后的有共同观测的关键帧
                kf.GetConnectedKeyFrames() - kf.GetVectorCovisibleKeyFrames() - pCurKF.GetVectorCovisibleKeyFrames() 
                for kf in pLoopKF.GetVectorCovisibleKeyFrames()
bFixScale：是否进行尺度优化，判断出现回环后，两帧计算Sim3变换（因为有尺度漂移），也就是从历史帧直接推算当前位姿
           单目SLAM一般都会发生尺度s漂移，而双目可以忽略
说明：
- 添加顶点
  - 对于地图上的每个关键帧kf
      - 若在CorrectedSim3中，直接读取sim3
    - 若不存在，直接用R、t转化为sim3
    - 作为顶点添加到优化图中，将sim3作为该顶点的待优化变量
    - 若kf为pLoopKF，则固定sim3
- 添加边(具有共同观测点的关键帧、关键帧的parent帧、关键帧的回环关键帧、关键帧的共视图关键帧)
  - 对于LoopConnections中的每个关键帧kf1
      - 遍历kf的所有关键帧kf2
        - 将kf1和kf2作为边添加到优化图中
        - 计算相对sim3(kf1 -> kf2)作为边的目标
  - 对于地图上的每个关键帧kf1
    - 若NonCorrectedSim3中存在kf1，则从NonCorrectedSim3中读取sim3
    - 若不存在，则从CorrectedSim3中读取sim3，或者通过R、t
    - 若kf1存在parent关键帧
        - 将kf1以及parent作为边添加到优化图中
        - 计算sim3(kf1 -> parent)作为边的目标
    - 对于kf1的每个回环帧kf2(sLoopEdges)
        - 若kf2.mnId < kf1.mnId
            - 若NonCorrectedSim3中存在kf2，则从NonCorrectedSim3中读取sim3
            - 若不存在，则从CorrectedSim3中读取sim3，或者通过R、t
            - 将kf1和kf2作为边添加到优化图中
            - 计算sim3(kf1->kf2)作为边的目标
    - 对于kf1的共视图里的每个关键帧kf3(top k个)
        - 若 kf3 && kf3 != kf1.parent && kf1.hasChild(kf3) && sLoopEdges.notContain(kf3) && kf3.mnId < kf1.mnId
            - 若NonCorrectedSim3中存在kf3，则从NonCorrectedSim3中读取sim3
            - 若不存在，则从CorrectedSim3中读取sim3，或者通过R、t
            - 将kf1和kf3作为边添加到优化图中
            - 计算sim3(kf1->kf3)作为边的目标
- 优化sim3
- 更新地图上的每个关键帧的pose
- 更新地图上的每个观测点mapPoint
    - 若mapPoint.mnCorrectedByKF==pCurKF->mnId
        - 则通过pCurKF重新计算mapPoint的3d坐标，首先用前优化的pose(R、t)投影到相机坐标，再利用优化后的sim3投影到世界坐标
    - 否则，通过mapPoint的参照帧重新计算mapPoint的3d坐标


```

#### OptimizeSim3

```c
int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
作用：利用epnp算法优化sim3(pKF2 -> pKF1)
参数：
pKF1：关键帧1
pKF2：关键帧2
vpMatches1：关键帧1和关键帧2的观测点匹配对，索引i对于关键帧1的第i个观测点，value对应关键帧2的第value个观测点
g2oS12：相对sim3(pKF2->pKF1)
th2：核函数的阈值
bFixScale：是否进行尺度优化，判断出现回环后，两帧计算Sim3变换（因为有尺度漂移），也就是从历史帧直接推算当前位姿
           单目SLAM一般都会发生尺度s漂移，而双目可以忽略
说明：
- 添加顶点到优化图中，待优化参数为g2oS12，记该顶点为v1
- 对于vpMatches1上的所有索引i
    - 获得pKF1上第i个地图点mp1，以及pKF2上地图点mp2(vpMatches1[i])
    - 判断mp1和mp2是否有效，无效continue
    - 将mp1投影到pKF1上的相机坐标下p1，作为顶点添加到优化图中，并将p1作为固定变量(不优化)
    - 将p1与顶点v1作为边添加到优化图中，获取pKF1上第i个像素点，作为边的目标(measurement)
    - 将mp2投影到pKF2上的相机坐标下p2，作为顶点添加到优化图中，并将p2作为固定变量(不优化)
    - 将p2与顶点v1作为边添加到优化图中，获得pKF2上mp2对应的像素点，作为边的目标(measurement)
- 第一次优化
    - 从优化图中删除误差大于指定阈值的边
- 第二次优化
    - 更新g2oS12
    - 对误差超过指定阈值的边上i，vpMatches1[i] = NULL

```

#### 参考文献

- [SLAM回环及优化](https://www.guyuehome.com/34904)

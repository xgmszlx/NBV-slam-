# Mutual information-based hierarchical NBV decision for active semantic visual SLAM under dynamic environments

**Authors:** Zhenyuan Yang, Ash Wan Yaw Sang, M. A. Viraj J. Muthugala, Mohan Rajesh Elara

**Journal:** Scientific Reports, 2026, 16:5847

**DOI:** https://doi.org/10.1038/s41598-026-36259-x

> 自动从 PDF 转换为 Markdown。为便于代码复现，已保留论文主要正文、图题、表格文本，并补充了图像链接与两个算法的可读伪代码。复杂数学公式和表格由 PDF 自动抽取，建议最终引用前再与原 PDF 校对。

## Abstract

Active Simultaneous Localization and Mapping (A-SLAM) technology enables a robot to autonomously plan its movements to build a comprehensive and accurate map of its surroundings. However, most existing SLAM algorithms are not robust in dynamic environments, as moving objects can negatively impact mapping and localization accuracy, making it difficult for the robot to keep tracking and fully understand its environment. While some semantic SLAM methods can identify and exclude dynamic objects, in active SLAM, excluding features without proper path planning carries significant risks of losing track. In this work, we propose a real-time mutual information-based active SLAM approach designed to enhance robustness in dynamic environments. The proposed method not only excludes dynamic objects from the mapping process but also integrates two Next-Best-View (NBV) decision modules to improve path planning and maintain robustness. This feature allows for improved mapping efficiency and robustness to avoid losing tracking in dynamic environments. Experiments conducted in two simulated environments and one real-world scenario demonstrate that the proposed active SLAM algorithm maintains its robustness and efficiency in dynamic environments, and is deployable in real applications. Simultaneous Localization and Mapping (SLAM) has long been a critical area of focus in robotics, particularly in autonomous navigation and path planning. Visual-based SLAM is more widely used because of its low cost and higher sensitivity in the textures and color in the environments1 compared to laser-based SLAM. Traditional visual SLAM methods such as ORB-SLAM22, LSD-SLAM3, and VINS4 all have good performance across typical scenes. Active SLAM means a robot autonomously creating a map of its environment, localizing itself, and controlling its own movements5. Unlike passive SLAM, active SLAM does not rely on manual intervention or predefined waypoints, making it more adaptable to unpredictable and unfamiliar real-world environments. In real-world scenarios, dynamic elements such as moving people are common and must be considered. These dynamic objects can significantly impact the accuracy of a robot’s localization. This challenge is further intensified by the inherently limited Field Of View (FOV) of cameras in visual SLAM, which prevents the robot from capturing a sufficiently rich set of static features in a single observation and makes the system more vulnerable to occlusion by moving objects. To address this challenge, semantic SLAM techniques, such as DS-SLAM6 and Dyna-SLAM7, have been developed for dynamic environments. These methods use semantic segmentation to exclude dynamic objects by excluding all the dynamic features. In passive visual SLAM, tracking can be corrected through manual control by moving the robot to areas with rich features. In active SLAM, however, the robot must plan its own movements. While many active visual SLAM methods focus on efficient or complete mapping8,9 , robustness and stability are equally important to avoid tracking loss, especially in dynamic environments where dynamic features are excluded. Traditional path planning algorithms often overlook moving obstacles and their relative motion with the robot. They typically maximize information gain on an occupancy map without modeling the predicted spatiotemporal occupancy of moving objects or their relative motion with the robot. As a result, paths that pass close to a pedestrian can continually reduce visible static features after semantic masking, leading to the exclusion of a large portion of the features in the current frame, increasing the risk of feature tracking failure. Similarly, if the robot continues to move in the same direction as the object, fewer features will be consistently captured, resulting in reduced mapping and localization accuracy. Therefore, proper motion planning is essential for active semantic SLAM in dynamic environments to ensure robust performance. Engineering Product Development, Singapore University of Technology and Design, Singapore 487372, Singapore. *email: viraj_jagathpriya@sutd.edu.sg

This paper proposes a novel hierarchical active semantic visual SLAM system based on the information theory10 to increase robustness under dynamic environments. The system has a module to find the global Next-Best-View (NBV) for the robot. Further, the system can generate a Feature Probability Map (FPM) based on the current image input and choose the local NBV. For global and local NBV decisions, they are both based on the Shannon mutual information11. The main contributions of the proposed paper are as follows:

- A global NBV module is proposed to apply mutual information on an occupancy grid, designed to deal with
the limited FOV of a camera.

- A pixel-level dynamic object prediction mechanism that models tracked object motion with anisotropic
Gaussian distributions and fuses it with feature distributions to generate an FPM.

- A local NBV module that applies the FPM to select viewpoints with maximal expected feature observability
while minimizing disturbance from dynamic obstacles.

- Comprehensive validation in both simulation and real-world environments, demonstrating that the proposed
semantic active SLAM framework improves robustness and localization reliability in dynamic environments.

## Related work

### Active SLAM and robot exploration

Active SLAM emphasizes viewpoint selection by actively exploring unknown areas to improve mapping quality and localization robustness. Generally, exploration methodologies can be classified into learning-based and classical models, with classical models further divided into deterministic and stochastic approaches. Among deterministic approaches, the frontier-based method proposed by Yamauchi et al.12 is the most commonly used due to its simplicity and efficiency, but it lacks a measure of information gain in the environments. To address this, more advanced information-theoretic exploration methods were developed, which will be discussed detailedly in “Mutual information-based active SLAM”. Other deterministic strategies include Voronoi-based coverage planners1314, which applies a topological Voronoi graph instead of a heavy occupancy grid, making it easier to store, update, and plan long-range navigation. Stochastic models are advantageous when the exploration space is large or highly uncertain. Xu et al.15 developed an exploration planner combining incremental sampling with Probabilistic Roadmaps16, balancing efficiency and adaptability. Wang et al.17 introduced Semantic Road Maps, which detect frontiers and integrate both information gain and travel cost to improve efficiency in multi-room exploration. Bio-inspired neural approaches such as Glasius-based Neural Network18 generate smooth, adaptive trajectories by unifying obstacle avoidance, goal attraction, and exploration within a single neural field, offering robustness to noise and dynamic changes19,20. Recent work also has highlighted the importance of considering environment changes when exploration. Zhao et al.21 propose a predictive framework that combines recurrent temporal models with spatial representations to capture environment evolution over time. This paper opens new insights for active SLAM in dynamic environments, where exploration decisions can anticipate scene evolution rather than rely solely on instantaneous observations.

### Mutual information-based active SLAM

Mutual information has been widely used in SLAM-based exploration to optimize viewpoint selection by quantifying the information gain. It was developed and applied in maximizing map entropy reduction ?, or minimizing map variance ,. Recent research has explored various applications of mutual information in autonomous exploration and decision-making. For geometry information such as occupancy map, Michael et al.22 proposed a method to reduce communication overhead in multi-agent systems using Gaussian processes for spatial field estimation to let agents optimize the information-gathering process. Zhang et al.23 proposed a method for the efficient computation of Shannon mutual information to evaluate potential information gain from different sensing actions, thereby improving mapping efficiency. Except for geometry mutual information, semantic information can also be applied. Zhang et al.24 introduce an active metric-semantic SLAM approach that combines semantic mutual information with the connectivity metrics of the underlying pose graph to select a strategy during exploration. Asgharivaskasi et al.25 proposed a Bayesian multi-class mapping algorithm using an octree structure to compute Shannon mutual information efficiently, enabling autonomous robots to perform semantic exploration in unknown environments. Recent advances in temporal environment modeling ? suggest that explicitly predicting environment evolution can be critical when planning under uncertainty. These observations indicate a gap between classical information-theoretic exploration methods and the requirements of dynamic real-world environments.

### Semantic active SLAM

In active SLAM, semantic information has recently been increasingly applied to provide robots with a richer understanding of the environment during exploration, going beyond purely geometric methods. Fredriksson26 introduced a semantic topometric exploration strategy that segments the grid map into structural regions and exploits both metric and semantic frontier properties to achieve faster and more efficient exploration. Tao et al.27 proposed an active metric-semantic SLAM framework for aerial robots that balances exploration efficiency with localization uncertainty reduction. In addition to exploration, exploitation also plays a critical role in ensuring localization accuracy and robustness. Wasserman et al.28 presented exploitation-guided exploration, a modular navigation framework that integrates exploration and exploitation modules to improve semantic navigation accuracy and efficiency. Tian et al.29 developed a semantic-centered ground–air collaborative mapping and navigation framework that fuses UAV and UGV maps into a consistent global representation, improving both mapping precision and exploration efficiency. While these approaches are effective in static environments, handling dynamic environments remains a significant challenge. In passive SLAM, semantic information has been widely used to improve robustness

under dynamic conditions6,7,30. Recent semantic SLAM systems have increasingly focused on handling dynamic environments. Islam et al.31 integrated YOLO-based object detection with enhanced strategies of loop closure to improve long term robustness in dynamic scenes. Similarly, Islam et al.32 propose an adaptive segmentation framework combined with dynamic object detection, demonstrating improved performance in real-world environments with moving objects. Several methods further couple semantic perception with geometric reasoning. ARD-SLAM33 introduces dynamic object identification alongside improved multi-view geometric constraints to achieve accurate and robust SLAM under scene dynamics. MVS-SLAM34 tightly integrates semantic RGB-D information with enhanced multi-view geometry, allowing dynamic regions to be identified while preserving static structure for mapping. FADM-SLAM35 emphasizes computational efficiency while maintaining robustness in environments with movable objects. This work integrates mutual information-based NBV selection with semantic segmentation and dynamic object tracking. Unlike prior studies that mainly target static environments, our approach explicitly models moving objects as pixel-level Gaussian distributions and adapts both global and local NBV strategies to maintain robust localization and efficient exploration in dynamic environments.

## Mutual information-based NBV selection

### Framework overview

ORB-SLAM22 is a widely used SLAM system with good performance, making it an excellent foundation for this work. In addition, this work focus on active SLAM with pure vision sensor without inertial measurements. When the IMU is disabled, the core visual frontend and backend optimization pipelines of ORB-SLAM3 are largely equivalent to those of ORB-SLAM2, except for additional components such as multi-map management and enhanced tracking recovery. ORB-SLAM2 therefore provides a simpler and more lightweight implementation that is sufficient for evaluating the proposed hierarchical NBV and semantic modules. The proposed system builds on ORB-SLAM2, integrating an active exploration module and enhancing it to adapt to dynamic environments. The overall framework of the proposed SLAM system is illustrated in Fig. 1. The robot’s decision variable is the selection of future viewpoints through motion actions. This work formulates two coupled optimization problems, one is global NBV problem that selects viewpoints to reduce geometric map uncertainty, the other is a local NBV problem that selects short-horizon motions to reduce feature observability uncertainty. These problems are solved hierarchically and are formalized in the following sections. The following subsections introduce the mutual information-based global exploration approach, the semantic segmentation and object tracking network, the process for detecting and excluding dynamic objects, and the function for local dynamic obstacle avoidance.

### Global NBV decision

For global exploration, the objective is to incrementally construct a complete 2D occupancy grid map, denoted as G ⊂ R2 . During mapping, a 3D dense map is built by back-projecting depth images into point clouds, as described in2. The 2D occupancy grid is then generated using OctoMap38 by projecting point clouds within a specified height range onto a planar surface. Each grid cell g ∈ G can take three semantic states: free, occupied, or unknown.

#### Probabilistic and entropy model

The environment is represented by a two-dimensional occupancy grid G = {g1 , g2 , . . . , gN }, where each grid cell gi is modeled as a Bernoulli random variable indicating whether the cell is occupied or free. The belief over each cell pt (gi ) at time t is defined in (1), where z1:t denotes the set of all observations up to time t. Cells labeled

![Fig 1](assets/fig1_framework.png)

Fig. 1. Overview of the proposed system. The system uses ORB-SLAM2 as the base. Object Tracking is done using Yolov836 and BoT-SORT37. The two NBV modules that will be introduced in this work can let the robot do active exploration. The global NBV decision module will help the robot find a global goal based on map information gain. The local NBV decision module will let the robot choose which direction to go based on the information gain of two FPMs.

as free or unknown correspond to p(g | z1:t ) close to 0 or 0.5, respectively. The uncertainty of the occupancy grid is quantified using Shannon entropy. Under the conditional independence assumption, the entropy of the map is defined in (2).

H(G | z1:t ) = −

∑[

g∈G

p(g | z1:t ) = P (g = occ | z1:t )(1)

]

p(g | z1:t ) log p(g | z1:t ) + (1 − p(g | z1:t )) log(1 − p(g | z1:t )) .(2)

#### Global NBV candidates selection

The search for the global NBV begins by extracting all frontiers, defined as the boundaries between known and unknown regions in the occupancy map. To reduce redundancy, the frontiers are clustered and downsampled using the Density-Based Spatial Clustering of Applications with Noise (DBSCAN) algorithm, yielding a set of representative candidate viewpoints. DBSCAN is applied with neighborhood radius ϵdb = 0.5 m and minimum cluster size set to 5. For each candidate point v, the potential information gain is evaluated across eight discrete yaw directions Θ = {0◦ , 45◦ , 90◦ , 135◦ , 180◦ , 225◦ , 270◦ , 315◦ }. For each direction θ ∈ Θ, the set of observable cells Gvis (v, θ) is determined via ray-casting on the occupancy grid, constrained by the camera field of view ϕ = 90◦ and maximum sensing range R = 5.0 m. Only cells within the angular sector [θ − ϕ/2, θ + ϕ/2] and distance R are considered, excluding those occluded by occupied cells. The next step is to compute the information gain of each candidate viewpoint in all directions. The information gain I is defined as the reduction in map uncertainty when a new observation is made. The uncertainty of the map is measured using the entropy H(G), which is given in (2), where z1:t in Z ⊂ R represents the set of all observations up to time t. For a viewpoint v and direction θ, the directional information gain is defined (3), where Z(v, θ) denotes the next observation obtained from viewpoint v facing direction θ, and Gvis (v, θ) is the corresponding set of visible cells. I(G; Z(v, θ) | z1:t ) = H(G | z1:t ) − H(G | Z(v, θ), z1:t ) = H(Gvis (v, θ) | z1:t )

(3)

To incorporate motion effort, a cost factor c is introduced to account for both translational and rotational costs. This is particularly important under a limited camera field of view, where large yaw adjustments can significantly reduce exploration efficiency. The cost is defined in (4), where d(xt , v) denotes the path length from the current pose xt to viewpoint v, and ∆θ(xt , θ) is the yaw change required to align the camera with direction θ. The final utility score is computed in (5) and the global NBV is selected using the greedy rule, as in (6). The result of global NBV selection is illustrated in Fig. 2a. Algorithm 1 summarizes the process for calculating the global NBV. c = λd d(xt , v) + λθ ∆θ(xt , θ),(4) S(v, θ) =

I(v, θ) ,(5) c+ϵ

(v ∗ , θ∗ ) = arg max S(v, θ).(6) (v,θ)

Due to the restricted FOV of a single camera, the robot may not fully observe the region around the selected global NBV (v ∗ , θ∗ ) upon arrival. To address this limitation, a local completion phase

is introduced. The set of local candidate viewpoints is defined in as N (v ∗ , θ∗ ) =

(v, θ) ∥v − v ∗ ∥ ≤ ρ , where ρ is the local

![Fig 2](assets/fig2_global_nbv.png)

Fig. 2. (a) is the result of finding global NBV. The red points are all NBV candidates, the green point is the selected NBV position, and the arrow is the NBV direction. (b) is the process of NBV selection between two global NBVs.

neighborhood radius. For each local candidate (v, θ) ∈ N (v ∗ , θ∗ ), the utility score (5) is evaluated, and viewpoints are iteratively selected using the greedy rule (6). This process continues until all candidates in N (v ∗ , θ∗ ) have been visited, ensuring sufficient coverage of the neighborhood around the global NBV even under a restricted camera FOV. The local completion process is illustrated in Fig. 2b.

```text
Algorithm 1. Find global NBV

Input: Grid map G, Current pose x_t
Parameters: DBSCAN eps_db = 0.5 m, minimum cluster size m_c = 5;
            yaw set Θ = {0°, 45°, ..., 315°}; camera FOV φ = 90°;
            max range R = 5.0 m; cost weights λ_d = 0.8, λ_θ = 1.0;
            small constant ε = 10^-6
Output: best pose z*, best direction dir*

1:  F_all = FindFrontiers(G)
2:  Z_set = DBSCANCluster(F_all, eps_db, m_c)
3:  S* = -∞, z* = ∅, dir* = ∅
4:  for z in Z_set do
5:      for dir in Θ do
6:          G_vis = RayCast(G, z, dir, φ, R)
7:          I = Σ_{g∈G_vis} H(g | z_{1:t})
8:          c = λ_d d(x_t, z) + λ_θ Δθ(x_t, dir)
9:          S = I / (c + ε)
10:         if S > S* then
11:             S* = S, dir* = dir, z* = z
12:         end if
13:     end for
14: end for
15: return z*, dir*
```

Algorithm 1. Find global NBV

### Dynamic object segmentation and tracking

To ensure robust localization in dynamic environments, the proposed SLAM system integrates semantic segmentation, multi-object tracking, and geometric consistency checks for dynamic object detection. Object segmentation is performed using YOLOv8s36, a state-of-the-art real-time detector trained on the COCO2017 dataset. This model can recognize and segment 80 object categories, including humans, vehicles, animals, and other commonly encountered objects. Consequently, the system is not limited to detecting pedestrians, but can also handle other mobile objects such as cars or movable chairs, which is important for general applicability in different environments. The choice of YOLOv8 is motivated by its high detection accuracy and real-time efficiency, which are critical for SLAM. To maintain temporal consistency, BoT-SORT37 is applied as the multi-object tracker. BoT-SORT assigns consistent IDs to objects detected across consecutive frames, enabling the system to follow their trajectories over time. This provides essential information on object motion, which supports distinguishing static structures from moving entities and reduces the risk of introducing dynamic features into the map. While semantic segmentation and tracking identify candidate dynamic objects, geometric verification based on the epipolar constraint is employed to confirm motion. During visual SLAM, features on static objects should satisfy the epipolar geometry: P 1 = [u1 , v1 , 1],

P 2 = [u2 , v2 , 1],(7)

P 2 F P T1 = 0,(8)

where P 1 and P 2 are matched points in consecutive frames and F is the fundamental matrix. Feature correspondences are considered to consistently violate the epipolar constraint if their reprojection residual exceeds a predefined threshold for more than 3 consecutive frames, and these features are considered as dynamic features. By combining semantic cues with epipolar geometry, the system achieves robust dynamic object detection: semantic segmentation provides object-level priors, while epipolar consistency confirms whether those objects are indeed moving. Features associated with confirmed dynamic objects are removed from the SLAM optimization process, preventing drift and preserving map accuracy. In addition, the tracking of moving objects can also help with the NBV decision in local areas, which will be introduced in the next subsection.

### Local NBV decision

The local NBV decision is based on two parts: one part is the feature points distribution in the current frame, and the other part is the movement of the tracked moving objects. The local NBV module aims to decrease the influence of moving objects and make the robot move toward the direction that has a higher probability of

features based on the generated FPM. The FPM is a combination of two probability maps, one coming from the extracted feature points, and one coming from the movement of dynamic objects.

#### Feature points

The probability map from static feature points is generated from the current camera frame. Suppose the current frame will be divided into n grids. For each grid, count the number of feature points. Then normalize the number of feature points in all regions so that the sum is one. The calculation of the probability map generated by feature points M f is defined in (9). N (G(x)) represents the number of feature points in the grid that the pixel in, x = [u, v]T represents the pixel position, K is the total number of feature points. Finally, a Gaussian blur is introduced after constructing the normalized probabilistic map. Based on empirical found, a Gaussian blur applied to the normalized probabilistic map can smooth out local noise and grid boundary effects, providing a more continuous distribution that better reflects feature uncertainty and increases the stability of the FPM. M f (x) =

N (G(x)) ,(9) K

This construction creates a discrete probability distribution over the image plane, representing the likelihood of observing static features at different locations. Then a Gaussian blur is introduced after constructing the normalized probabilistic map M f to reduce local noise and grid boundary artifacts. Based on empirical found, a Gaussian blur applied to the normalized probabilistic map can smooth out local noise and grid boundary effects, providing a more continuous distribution that better reflects feature uncertainty and increases the stability of the FPM.

#### Tracked moving objects

To model the influence of dynamic objects, there are some transitions from tracked moving objects to the FPM. Dynamic objects reduce the likelihood of observing static features in regions they occupy or are expected to occupy. Since features inside dynamic objects are excluded from visual SLAM, the influence of dynamic objects is modeled by estimating the probability that a given pixel location may be affected by dynamic motion. So the problem can be transferred to the probability that dynamic objects may appear. For each tracked dynamic object, pixels inside its segmentation mask are used to construct a probabilistic motion influence field. The probability that a pixel location x is affected by a dynamic object is modeled as a multivariate Gaussian distribution, which is shown in (10), where µ denotes the pixel coordinate of a dynamic pixel inside the segmentation mask, and Σr is a rotated covariance matrix encoding motion uncertainty. p(x) =

(

2π

)

exp − (x − µ)T Σ−1 r (x − µ) (10) |Σr |

√

To account for directional motion, an anisotropic covariance matrix is constructed. For the anisotropic covariance matrix, the calculation is in (11) and (12). θ is the tracked object moving direction angle, R is the rotation matrix, Σ is the covariance matrix before rotation.

[ 2

σ Σ= ∥

]

, σ⊥

[

cos θ R(θ) = sin θ

]

− sin θ cos θ (11)

Σr = R(θ)ΣR(θ)T (12)

The longitudinal and orthogonal variances σ∥ and σ⊥ can be calculated in (13), where v is the image-plane displacement of the tracked object centroid between consecutive frames (pixels per frame), ϵ is a small constant preventing degeneracy, and α, β are tunable parameters controlling uncertainty growth along and perpendicular to the motion direction. Tuning α and β can adjust the level of the influence of speed on the uncertainty both along or perpendicular to the motion direction. σ∥ = α(v + ϵ),

σ⊥ = β(v + ϵ)(13)

When multiple dynamic objects are present in the scene, each tracked object is modeled independently in the image domain. For each object, a Gaussian probability is generated over the pixels within its segmentation mask based on the predicted motion state. The dynamic influence map is obtained by summing the Gaussian probability of all tracked objects, capturing the cumulative reduction in feature observability caused by multiple movingh objects. Since segmentation masks of different objects do not overlap, each pixel is associated with a single dynamic influence value. The aggregated map is then normalized to produce a valid probability distribution in FPM fusion and construction the final FPM.

#### FPM fusion

The above equations calculated the probability that a pixel is dynamic based on one existing moving pixel. To construct the probability map M t , Gaussian kernels are placed at all n pixels inside the dynamic mask and added all together. The result is normalized over the entire image domain Ω to ensure it integrates to one. The final probability map generated from tracked moving objects can be calculated as in (14).

∑n pk (x) k=1 ∑n M t (x) = 1 − ∑ x∈Ω

k=1

pk (x)

.(14)

Here, the subtraction from one represents that regions with higher dynamic probability should contribute less to feature probability. Finally, the FPM from moving objects M t is linearly combined with the FPM from detected features M f and normalized, as in (15). Static feature observability and dynamic object influence are modeled as separable components in a pixel level and fused linearly prior to normalization to construct the final FPM. As a result, the final FPM M is obtained by linearly combining the probability maps both from features and dynamic objects and renormalizing, as shown in (15). M f (x) + M t (x) .(15) (M f (x) + M t (x)) x∈Ω

M (x) = ∑

An example of the FPM construction process is shown in Fig. 3. Note that the probability map generated from dynamic objects represents diffused motion uncertainty rather than precise object contours. With the FPM generated from the current state, estimating the information gain for a candidate’s next view requires predicting the FPM in the next predicted state. The predicted FPM is approximated using a deterministic prediction of the next state. This involves predicting both the feature distribution and the states of tracked moving objects. For the feature distribution, two cases are considered. If the candidate view includes previously unexplored regions, their feature density is approximated by that of the nearest adjacent explored area. If the view overlaps with an already mapped area, the feature count is obtained directly from the sparse map, which consists of reliable tracked feature points for robot localization. An example of the sparse feature map is shown in Fig. 4a, while the corresponding prediction process is illustrated in Fig. 4b. By getting feature density of the predicted view, the predicted FPM M pf (x) from the number of predicted features N (Gp (x)) can be calculated using the same method in (9). For tracked moving objects, the future state is predicted using a constant-velocity Kalman Filter (KF). Specifically, each pixel x inside the tracked object is associated with an independent KF instance, which updates position and velocity estimates and provides a prediction one step ahead under the assumption of constant speed. With the predicted pixel position xp , the predicted FPM from tracked objects M pt (x) can be calculated in (14). With both the predicted feature distribution and predicted object states, the FPM for the next view M p (x) is generated using (15). Suppose the observation for the next view is Z l , the entropy of this predicted FPM H(M p | Z l , z1:t ) can be calculated using (16). To find the local NBV, the information gain needs to be calculated for potential next views and find the maximum information gain. The information gain can be calculated in (17). H(M p | Z l , z1:t ) = −

∑

mp ∈M p

p(mp | Z l , z1:t ) log p(mp | Z l , z1:t ),(16)

I(M ; Z l | z1:t ) = H(M | z1:t ) − H(M p | Z l , z1:t ).(17)

![Fig 3](assets/fig3_fpm_generation.png)

Fig. 3. An example of FPM generation. (a) is the original frame with semantic segmentation, object tracking with KF, and feature detection (green points). (b) shows the probability map generated by feature points. (c) shows the probability map generated by the tracked moving object. (d) is the final FPM.

![Fig 4](assets/fig4_feature_prediction.png)

Fig. 4. (a) shows the sparse feature points inside the map (points in orange color). (b) is the illustration of feature prediction. If the robot rotate up, the previous observed features and remaining current features will be the predicted features; if the robot rotate down to the unknown area, the predicted features will keep the same density.

The robot is modeled as a differential-drive platform with velocity constraints on both linear and angular motion. Candidate poses are thus limited to those reachable within the dynamic window defined by these constraints, implemented using the Dynamic Window Approach (DWA)39. Local NBV planning is triggered at a fixed frequency (10 Hz in our experiments), ensuring that the robot continuously updates its motion plan as both features and tracked objects update. This combination of predicted FPM and dynamic feasibility ensures that the selected local NBV maximizes information gain while remaining executable under the robot’s kinematic limits. Algorithm 2 summarizes the process for calculating the local NBV.

```text
Algorithm 2. Find local NBV

Input: Current frame F, Grid map G, Current pose x_t
Parameters: detection threshold τ_det = 0.8, non-maximum suppression IoU τ_nms = 0.45;
            BoT-SORT IoU τ_iou = 0.8, frame rate A = 10 frames per second;
            FPM grid n = 640 × 480; prediction horizon Δt = 1;
            anisotropic parameters α = 1.5, β = 0.5, ε = 0.01;
            DWA v_max = 0.2 m/s, ω_max = 0.2 rad/s
Output: best path P*

1:  mask = Segmentation(F, τ_det, τ_nms)
2:  v, dir = Track(mask, F, τ_iou, A)
3:  feature = ExtractORB(F)
4:  M_f = GetFeatureProbMap(feature, F, n)
5:  M_t = GetObjectProbMap(mask, v, dir, F, α, β, ε)
6:  M = Normalize(M_f + M_t)
7:  H(M) = -Σ_{x∈Ω} M(x) log M(x)
8:  I* = -∞, view* = ∅
9:  nextviews = SampleNextViews(x_t, v_max, ω_max, T)
10: for view in nextviews do
11:     M_f^p = PredictFeatureProbMap(feature, G, view)
12:     v^p, dir^p = PredictTracks(v, dir, Δt)
13:     M_t^p = GetObjectProbMap(mask, v^p, dir^p, F, α, β, ε)
14:     M_p = Normalize(M_f^p + M_t^p)
15:     H(M_p) = -Σ_{x∈Ω} M_p(x) log M_p(x)
16:     I = H(M) - H(M_p)
17:     if I > I* then
18:         I* = I, view* = view
19:     end if
20: end for
21: P* = DWAPlanner(view*, G)
22: return P*
```

Algorithm 2. Find local NBV

## Experiments and results

For the experiments, the performance of the proposed SLAM system is evaluated both on two simulation environments and a real-world scene.

### Implementation details

Both simulation and real-world experiments are conducted using an RGB-D camera rigidly mounted on the robot platform. Camera intrinsics are obtained through offline calibration and remain fixed throughout all experiments. The calibrated intrinsic matrix coefficients fx , fy , cx , cy are 573.4, 574.8, 320.1 and 322.6 respectively corresponding to a 640 × 480 image resolution. Depth measurements are provided directly by the RGB-D sensor with a maximum reliable range of 5.0 m and are registered to the RGB stream using factory calibration. Semantic segmentation and object detection are performed using YOLOv8s model with pretrained weights trained on the COCO dataset with no extra finetuning. Detection confidence threshold is set to 0.8, and nonmaximum suppression is performed with an IoU threshold of 0.45. Detected semantic classes corresponding to potentially dynamic objects, only ’person’ is included as potential dynamic objects in experiments. Once a person is detected, the algorithm will identify whether it is moving using epipolar constraint as mentioned in 3.3. Instance association across frames is handled by BoT-SORT with a maximum association age of 30 frames and an IoU matching threshold of 0.8. The resulting object tracks provide the motion in the pixel-level estimates used for dynamic uncertainty modeling.

### Experiments on simulation environments

The experiments are conducted on a computer that has an Intel Core i7-8700 CPU @ 3.20GHz × 12 processors and an NVIDIA GeForce RTX 3080 graphic. The operating system is Ubuntu 20.04. The experiments are conducted using the Robot Operating System (ROS)40, and the simulation environments are created in Gazebo41. The first environment has an open space with an area of 10 m × 10 m. The second environment has a more complex layout with a larger area of 19 m × 22 m. Both environments have two persons walking back and forth towards fixed trajectories. The simulation environments are shown in Fig. 5a Env_1 and Env_2. The linear and angular velocities of the robot are set to 0.2 m/s and 0.2 rad/s, grid map resolution is 0.05 m per cell. The robot will stop exploring after the next information gain from the global NBV module is less than a threshold. The threshold is 50 bits for Env_1 and 260 bits for Env_2. The proposed method is divided into two parts: only the global NBV module part and the whole proposed method. They are compared with three state-of-the-art methods: the nearest frontier-based exploration method, RNEX42 and TARE43. Their efficiency is evaluated using travel distance, travel time, and map Entropy Reduction Rate (ERR). Their accuracy is evaluated using ATE, RPE. The active mapping results are shown in Fig. 5, and their travel distance, travel time, ATE, and ERR are shown in Tables 3 and 4. The robot trajectories and groundtruth trajectories are shown in Fig. 6. The proposed approach is evaluated in two parts: using only the global NBV module and using the full hierarchical NBV framework. Both variants are compared with three representative exploration baselines, including a nearest-frontier strategy, RNEX42, and TARE43. Exploration efficiency is evaluated using travel distance, travel time, and the map Entropy Reduction Rate (ERR), which measures the rate of uncertainty reduction in the occupancy map. Localization accuracy is evaluated using standard SLAM metrics, including

![Fig 5](assets/fig5_simulation_mapping.png)

Fig. 5. Active mapping in simulation environments.

![Fig 6](assets/fig6_trajectories.png)

Fig. 6. Comparison of groundtruth trajectories and robot trajectories. (a) Nearest Frontier based, (b) proposed (global only), (c) proposed.

Metric

Travel Distance (m) Travel Time (min) ATE (m) RPE Trans. (m)

Definition

∑T −1 t=1

√ ∑ T T

t=1

√1 ∑ N

∥pest − pgt t t ∥

Global localization accuracy

t

∥∆test − ∆tgt t t ∥

Local translational motion error

t

gt −1 2 θ(∆Rest ) t (∆Rt )

Local rotational motion error

ERR (bits/s) Tracking loss rate (%)

Nfail Ntotal

Near collision counts

Nnc

N

Total path length of the robot Total exploration duration

tT − t1

√1 ∑

RPE Rot. (rad)

Description

∥pt+1 − pt ∥

H(G0 )−H(GT ) Ttravel

× 100

Information gain efficiency Frequency of tracking failures Number of getting close to moving objects

Table 1. Definitions of evaluation metrics used in experiments.

Absolute Trajectory Error (ATE), as well as Relative Pose Error (RPE) in both translational and rotational parts. They reflect the stability and the safety of motion execution in dynamic environments. Representative active mapping results are visualized in Fig. 5. More details of these metrics and the calculation methods are shown in Table 1 and Table 2. Quantitative comparisons of these metrics are shown in Table 3 and Table 4. The estimated robot trajectories and the corresponding ground-truth trajectories provided by the simulator are illustrated in

![Fig 6](assets/fig6_trajectories.png)

Fig. 6. The results of the experiment show that the proposed method outperforms other methods both in efficiency and accuracy. As shown in Fig. 5, particularly in Fig. 5e and f, the method incorporating the local NBV module leaves the fewest residual points corresponding to moving persons (highlighted in red circles) on the map, indicating that dynamic targets have minimal impact on the mapping process. From Table 3 and 4, compared to the nearest frontier-based method, the global NBV module takes less time because it can maximize the entropy reduction, making it complete the mapping process more efficiently. Compared with TARE and RNE, maps generated by TARE exhibit greater distortion and yield the highest ATE and RPE due to interference from dynamic objects. Compared to only the global NBV module, the whole proposed method has the least ATE, RPE and travel time because it avoids the person in advance to make its path smoother and leads itself to the next area with rich features once it detects the moving person, which increases the accuracy. Regarding localization accuracy, Fig. 6 compares the estimated and groundtruth trajectories. The nearestfrontier, TARE, and RNE methods exhibit higher drift when the robot approaches moving obstacles. In contrast, the proposed method demonstrates the smallest trajectory drift because of the local NBV module, which reduces the likelihood of keeping moving persons within the robot’s current FOV and thus improves localization precision.

Symbol

Definition

pt

Robot position at time step t

pest t

Estimated robot position at time t

pgt t

Ground-truth robot position provided by the Gazebo at time t

∆tt

Relative translation between consecutive poses: pt+1 − pt

∆Rt

Relative rotation between consecutive poses

θ(·)

Rotation angle extracted from a relative rotation matrix

T

Total number of poses in the trajectory

N

Number of valid relative pose pairs used for RPE computation

H(G)

Shannon entropy of the occupancy grid map G

G0 , G T

Occupancy grid map at the start and end of exploration

Ttravel

Total exploration time in seconds

Nfail

Number of SLAM tracking failures

Ntotal

Total number of evaluated frames

Nnc

Number of near collision times detected in simulation

Table 2. Notation used in evaluation metrics.

Method

Nearest frontier

TARE

RNE

Proposed (global)

Proposed

Travel distance (m)

31.25

25.31

34.34

29.88

24.21

Travel time (min)

5.24

4.89

4.93

4.91

4.31

ATE (m)

0.30

0.32

0.31

0.27

0.25

RPE Trans. (m)

0.27

0.28

0.29

0.25

0.21

RPE Rot. (rad)

0.15

0.18

0.17

0.19

0.13

ERR (bits/s)

77.35

85.90

85.27

84.43

97.47

Table 3. Comparison results of environment 1. The best results are highlighted.

Method

Nearest frontier

TARE

RNE

Proposed (global)

Proposed

Travel distance (m)

71.79

69.07

71.06

68.04

68.86

Travel time (min)

11.44

11.32

11.79

10.54

10.60

ATE (m)

0.42

0.56

0.46

0.38

0.30

RPE Trans. (m)

0.32

0.44

0.39

0.31

0.26

RPE Rot. (rad)

0.21

0.22

0.21

0.23

0.18

ERR (bits/s)

152.97

153.88

148.71

166.94

165.36

Table 4. Comparison Results of environment 2. The best results are highlighted.

Environment 1

Environment 2

Proposed (global)

Proposed

Proposed (global)

Proposed

Tracking loss rate (%)

Near collision counts

Table 5. Comparison of tracking loss rate and numbers of getting near collision between global NBV and full hierarchical NBV in two environments.

In Env_2, the proposed method also has the least ATE. However, regarding the efficiency, the global NBV module is slightly better than the whole proposed method. This is because in this experiment, when the robot was navigating using the whole proposed method, the robot deviated from its original path based on the local NBV module the first time it encountered the moving person, and later navigated back to the original global goal. This will compromise a little efficiency for a higher localization accuracy because of less dynamic influence. To isolate the contribution of the local NBV and FPM module to system robustness and safety, tracking loss rate and near collision counts are evaluated for the two proposed variants: the only global NBV configuration

![Fig 7](assets/fig7_real_world_experiment.png)

Fig. 7. The real-world experiment. (a) is the Meerkat robot; (b) is the experiment environment, a person is walking back and forth; (c) is the created map with the robot trajectory.

Module

Time (s)

Hardware

SLAM pipeline

0.132

CPU

Semantic segmentation (YOLOv8)

0.032

GPU

Object tracking (BoT-SORT)

0.024

GPU

Global NBV planning

0.087

CPU

Local NBV planning

0.064

CPU

Motion planning

0.056

CPU

Table 6. Real-time performance of each system module on the Meerkat robot.

and the full hierarchical NBV framework. This comparison is conducted within the same SLAM, detection, and tracking pipeline, the only difference between the two variants is whether the local NBV module is activated to refine viewpoints in the neighborhood of a global goal using the FPM. Tracking loss is directly decided by the SLAM backend when localization fails during the exploration. Near collision counts are counted when the robot remains within a distance of 2 meters from moving obstacles for more than 3 seconds, which represent a safety factor during exploration. The results are shown in Table 5. The experiments show that the full hierarchical NBV framework leads to a reduction in both tracking loss rate and near collision counts across the two environments. In Env 1, the tracking loss rate decreases from 30% under the only global variant to 10% with the full hierarchical NBV, while near collision counts are reduced from 5 to 1. Env 2 shows the same trend with tracking loss rate reduced from 20% to 10 and near collision counts reduced from 7 to 2. These results indicate that the local NBV and FPM module provides big robustness benefits beyond global frontier selection alone. By performing local viewpoint completion under limited FOV constraints, the robot is able to maintain more stable feature observations and reduce interactions with moving obstacles.

### Experiments on a real-world scene

#### Experiment environment

The real-world experiment was conducted using the robot Meerkat44. It is equipped with a Jetson AGX Orin, which has an NVIDIA Ampere architecture GPU and an Arm Cortex-A78AE CPU. The camera is Realsense D435i. The robot and experiment environment are shown in Fig. 7a and b, the map created by the robot is shown in Fig. 7c. The real-world experiment validates the usability of the proposed method in real applications.

### Computational complexity and real-time analysis

The proposed framework has four main components: global NBV planning, local NBV planning based on the FPM, semantic segmentation and object tracking, and SLAM back-end optimization. The global NBV module operates on a 2D occupancy grid with Ng cells and Nv frontier candidates. For each candidate viewpoint, information gain is evaluated over a fixed set of discrete yaw angles, resulting in a computational complexity of O(Nv · Ngvis ), where Ngvis ≪ Ng denotes the number of visible cells obtained via ray casting. Since the global NBV module is triggered at a low frequency, its impact on online performance is negligible. The local NBV module works in the image domain. Let Np denote the number of pixels in the image and Nd the number

of dynamic object pixels. FPM construction scales linearly with the number of detected features and image grids, which is O(Np ). Dynamic object uncertainty modeling using anisotropic Gaussian diffusion also scales as O(Nd ). For each candidate local view, the predicted FPM and corresponding entropy are computed, generating a total complexity of O(Nc · Np ), where Nc is the number of locally reachable candidate views. Overall, the proposed framework has linear complexity related to map size and image resolution. The performance of real application is shown in Table 6. Table 6 shows the average runtime of each system module measured during real-world experiments on the Meerkat robot. Semantic segmentation and object tracking continuously update dynamic object information on the GPU, while the CPU updates the SLAM algorithm, the whole global and local NBV modules, and robot motion planning. These results demonstrate that the proposed hierarchical NBV framework can be deployed on practical robotic platforms in real time.

## Conclusion

This paper introduces a novel active SLAM system based on Shannon mutual information, designed to achieve robust localization and efficient mapping in dynamic environments. The proposed system uses semantic segmentation to exclude moving objects, determines the global NBV using an occupancy map, tracks moving objects, and identifies the local NBV based on the current feature distribution and the status of moving objects. Experimental results from two simulations demonstrate the system’s ability to maintain robustness and provide accurate localization in dynamic environments. Additionally, real-world experiments show that the system is suitable for real-time robotic applications. While this method offers new insights into active SLAM in dynamic settings, some limitations remain. The robot may still lose track when dynamic objects suddenly occupy the majority of the camera view. Future work will focus on two areas: enabling the robot to recover from sudden tracking loss and integrating a new path planning module to enhance mapping efficiency.

## Data availability

All data supporting the findings of this study are available within the paper and its Supplementary Information. Received: 6 November 2025; Accepted: 12 January 2026

## Key equations normalized for implementation

```math
p(g \mid z_{1:t}) = P(g = occ \mid z_{1:t})
```

```math
H(G \mid z_{1:t}) = -\sum_{g \in G}\left[p(g \mid z_{1:t})\log p(g \mid z_{1:t}) + (1-p(g \mid z_{1:t}))\log(1-p(g \mid z_{1:t}))\right]
```

```math
I(G; Z(v,\theta) \mid z_{1:t}) = H(G \mid z_{1:t}) - H(G \mid Z(v,\theta), z_{1:t}) = H(G_{vis}(v,\theta) \mid z_{1:t})
```

```math
c = \lambda_d d(x_t, v) + \lambda_\theta \Delta\theta(x_t,\theta)
```

```math
S(v,\theta) = \frac{I(v,\theta)}{c + \epsilon}
```

```math
(v^*,\theta^*) = \arg\max_{(v,\theta)} S(v,\theta)
```

```math
M_f(x) = \frac{N(G(x))}{K}
```

```math
p(x) = \frac{1}{2\pi\sqrt{|\Sigma_r|}}\exp\left(-\frac{1}{2}(x-\mu)^T\Sigma_r^{-1}(x-\mu)\right)
```

```math
\Sigma = \begin{bmatrix}\sigma_\parallel^2 & 0 \\ 0 & \sigma_\perp^2\end{bmatrix},\quad
R(\theta)=\begin{bmatrix}\cos\theta & -\sin\theta \\ \sin\theta & \cos\theta\end{bmatrix}
```

```math
\Sigma_r = R(\theta)\Sigma R(\theta)^T
```

```math
\sigma_\parallel = \alpha(v+\epsilon),\quad \sigma_\perp = \beta(v+\epsilon)
```

```math
M_t(x)=1-\frac{\sum_{k=1}^{n}p_k(x)}{\sum_{x\in\Omega}\sum_{k=1}^{n}p_k(x)}
```

```math
M(x)=\frac{M_f(x)+M_t(x)}{\sum_{x\in\Omega}(M_f(x)+M_t(x))}
```

```math
H(M^p \mid Z_l,z_{1:t})=-\sum_{m^p\in M^p}p(m^p\mid Z_l,z_{1:t})\log p(m^p\mid Z_l,z_{1:t})
```

```math
I(M;Z_l\mid z_{1:t})=H(M\mid z_{1:t})-H(M^p\mid Z_l,z_{1:t})
```

## References

1. Pu, H., Luo, J., Wang, G., Huang, T. & Liu, H. Visual slam integration with semantic segmentation and deep learning: A review. IEEE Sens. J. (2023). 2. Mur-Artal, R. & Tardós, J. D. Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras. IEEE Trans. Rob. 33, 1255–1262 (2017). 3. Engel, J., Schöps, T. & Cremers, D. Lsd-slam: Large-scale direct monocular slam. In European Conference on Computer Vision, 834–849 (Springer, 2014). 4. Qin, T., Li, P. & Shen, S. Vins-mono: A robust and versatile monocular visual-inertial state estimator. IEEE Trans. Rob. 34, 1004– 1020 (2018). 5. Placed, J. A. et al. A survey on active simultaneous localization and mapping: State of the art and new frontiers. IEEE Trans. Rob. 39, 1686–1705 (2023). 6. Yu, C., et al. Ds-slam: A semantic visual slam towards dynamic environments. In 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 1168–1174 (IEEE, 2018). 7. Bescos, B., Fácil, J. M., Civera, J. & Neira, J. Dynaslam: Tracking, mapping, and inpainting in dynamic scenes. IEEE Robot. Autom. Lett. 3, 4076–4083 (2018). 8. Bonetto, E., Goldschmid, P., Pabst, M., Black, M. J. & Ahmad, A. irotate: Active visual slam for omnidirectional robots. Robot. Auton. Syst. 154, 104102 (2022). 9. Chen, Y., Huang, S. & Fitch, R. Active slam for mobile robots with area coverage and obstacle avoidance. IEEE/ASME Trans. Mechatron. 25, 1182–1192 (2020). 10. Thomas, M. & Joy, A. T. Elements of Information Theory (Wiley-Interscience, XXX, 2006). 11. Shannon, C. E. A mathematical theory of communication. Bell Syst. Tech. J. 27, 379–423 (1948). 12. Yamauchi, B. A frontier-based approach for autonomous exploration. In Proceedings 1997 IEEE International Symposium on Computational Intelligence in Robotics and Automation CIRA’97.’ Towards New Computational Principles for Robotics and Automation’, 146–151 (IEEE, 1997). 13. Huang, K.-C., Lian, F.-L., Chen, C.-T., Wu, C.-H. & Chen, C.-C. A novel solution with rapid voronoi-based coverage path planning in irregular environment for robotic mowing systems. Int. J. Intell. Robot. Appl. 5, 558–575 (2021). 14. Hu, J., Niu, H., Carrasco, J., Lennox, B. & Arvin, F. Voronoi-based multi-robot autonomous exploration in unknown environments via deep reinforcement learning. IEEE Trans. Veh. Technol. 69, 14413–14423 (2020). 15. Xu, Z., Deng, D. & Shimada, K. Autonomous UAV exploration of dynamic environments via incremental sampling and probabilistic roadmap. IEEE Robot. Autom. Lett. 6, 2729–2736. https://doi.org/10.1109/LRA.2021.3062008 (2021). 16. Kavraki, L. E., Svestka, P., Latombe, J.-C. & Overmars, M. H. Probabilistic roadmaps for path planning in high-dimensional configuration spaces. IEEE Trans. Robot. Autom. 12, 566–580 (2002). 17. Wang, C., Zhu, D., Li, T., Meng, M.Q.-H. & De Silva, C. W. Efficient autonomous robotic exploration with semantic road map in indoor environments. IEEE Robot. Autom. Lett. 4, 2989–2996 (2019). 18. Glasius, R., Komoda, A. & Gielen, S. C. A biologically inspired neural net for trajectory formation and obstacle avoidance. Biol. Cybern. 74, 511–520 (1996). 19. Wenhao, W., Fangfang, Z., Jianbin, X., Hongnian, Y. & Yanhong, L. An improved multi-robot coverage method in 3d unknown environment based on gbnn. In Chinese Intelligent Automation Conference, 476–483 (Springer, 2023). 20. Wan, A. Y. S., Yi, L., Hayat, A. A., Gen, M. C. & Elara, M. R. Complete area-coverage path planner for surface cleaning in hospital settings using mobile dual-arm robot and gbnn with heuristics. Complex Intell. Syst. 1–19 (2024). 21. Zhao, X., Wang, P., Gao, S., Yasir, M. & Islam, Q. U. Combining LSTM and plus models to predict future urban land use and land cover change: A case in Dongying City, China. Remote Sens. 15, 2370 (2023). 22. Kepler, M. E. & Stilwell, D. J. An approach to reduce communication for multi-agent mapping applications. In 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 4814–4820 (IEEE, 2020).

23. Zhang, Z., Henderson, T., Karaman, S. & Sze, V. FSMI: Fast computation of Shannon mutual information for information-theoretic mapping. Int. J. Robot. Res. 39, 1155–1177 (2020). 24. Zhang, R., Bong, H. M. & Beltrame, G. Active semantic mapping and pose graph spectral analysis for robot exploration. In 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 13787–13794 (IEEE, 2024). 25. Asgharivaskasi, A. & Atanasov, N. Semantic octree mapping and Shannon mutual information computation for robot exploration. IEEE Trans. Rob. 39, 1910–1928 (2023). 26. Fredriksson, S., Saradagi, A. & Nikolakopoulos, G. Robotic exploration through semantic topometric mapping. In 2024 IEEE International Conference on Robotics and Automation (ICRA), 9404–9410 (IEEE, 2024). 27. Tao, Y., Liu, X., Spasojevic, I., Agarwal, S. & Kumar, V. 3d active metric-semantic slam. IEEE Robot. Autom. Lett. 9, 2989–2996 (2024). 28. Wasserman, J., Chowdhary, G., Gupta, A. & Jain, U. Exploitation-guided exploration for semantic embodied navigation. In 2024 IEEE International Conference on Robotics and Automation (ICRA), 2901–2908 (IEEE, 2024). 29. Tian, X. et al. Same: ground-air collaborative semantic active mapping and exploration. In 2024 IEEE International Conference on Unmanned Systems (ICUS), 1923–1930 (IEEE, 2024). 30. Yang, Z., Sachinthana, W., Samarakoon, S. B. P. & Elara, M. R. Semantic slam fusing moving constraint for dynamic objects under indoor environments. In 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 7900–7907 (IEEE, 2024). 31. Islam, Q. U. et al. Advancing autonomous slam systems: Integrating yolo object detection and enhanced loop closure techniques for robust environment mapping. Robot. Auton. Syst. 185, 104871 (2025). 32. Islam, Q. U. et al. Advancing real-world visual slam: Integrating adaptive segmentation with dynamic object detection for enhanced environmental perception. Expert Syst. Appl. 255, 124474 (2024). 33. Islam, Q. U. et al. Ard-slam: Accurate and robust dynamic slam using dynamic object identification and improved multi-view geometrical approaches. Displays 82, 102654 (2024). 34. Islam, Q. U., Ibrahim, H., Chin, P. K., Lim, K. & Abdullah, M. Z. MVS-SLAM: Enhanced multiview geometry for improved semantic RGBD SLAM in dynamic environment.. J. Field Robot. 41, 109–130 (2024). 35. Ul Islam, Q., Ibrahim, H., Chin, P. K., Lim, K. & Abdullah, M. Z. FADM-SLAM: A fast and accurate dynamic intelligent motion slam for autonomous robot exploration involving movable objects. Robot. Intell. Autom. 43, 254–266 (2023). 36. Jocher, G., Chaurasia, A. & Qiu, J. Ultralytics YOLO (2023). 37. Aharon, N., Orfaig, R. & Bobrovsky, B.-Z. Bot-sort: Robust associations multi-pedestrian tracking. arXiv preprint arXiv:2206.14651 (2022). 38. Hornung, A., Wurm, K. M., Bennewitz, M., Stachniss, C. & Burgard, W. OctoMap: An efficient probabilistic 3D mapping framework based on octrees. Autonomous Robots https://doi.org/10.1007/s10514-012-9321-0 (2013). 39. Fox, D., Burgard, W. & Thrun, S. The dynamic window approach to collision avoidance. IEEE Robot. Autom. Mag. 4, 23–33 (1997). 40. Quigley, M. et al. ROS: an open-source robot operating system. In ICRA Workshop on Open Source Software, Vol. 3, 5 (2009). 41. Koenig, N. & Howard, A. Design and use paradigms for gazebo, an open-source multi-robot simulator. In 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (IEEE Cat. No. 04CH37566), Vol. 3, 2149–2154 (IEEE, 2004). 42. Steinbrink, M., Koch, P., Jung, B. & May, S. Rapidly-exploring random graph next-best view exploration for ground vehicles. In 2021 European Conference on Mobile Robots (ECMR), 1–7 (IEEE, 2021). 43. Cao, C., Zhu, H., Choset, H. & Zhang, J. Tare: A hierarchical framework for efficiently exploring complex 3d environments. Robot. Sci. Syst. 5, 2 (2021). 44. Borusu, C. S. C. S. et al. Evaluating the robot inclusivity of buildings based on surface unevenness. Appl. Sci. 14, 7831 (2024).

## Author contributions

Zhenyuan Yang designed the algorithms, Zhenyuan Yang and Ash Wan Yaw Sang conducted the experiments, Zhenyuan Yang, Ash Wan Yaw Sang, M. A. Viraj J. Muthugala, and Mohan Rajesh Elara analysed the results. All authors reviewed the manuscript.

## Funding

This research is supported by the National Robotics Programme under category National Robotics Programme 2.0, LEO 1.0: A New Class of Bed Making Robot, Award No. M25N4N2028, and also supported by A*STAR under its RIE2025 IAF-PP programme, Modular Reconfigurable Mobile Robots (MR)2, Grant No. M24N2a0039.

## Declarations

### Competing interests

The authors declare no competing interests.

## Additional information

Supplementary Information The online version contains supplementary material available at https://do i.org/1 0.1 038/s41598 -026-36259 -x. Correspondence and requests for materials should be addressed to M.A.V.J.M. Reprints and permissions information is available at www.nature.com/reprints. Publisher’s note Springer Nature remains neutral with regard to jurisdictional claims in published maps and institutional affiliations.

Open Access This article is licensed under a Creative Commons Attribution 4.0 International License, which permits use, sharing, adaptation, distribution and reproduction in any medium or format, as long as you give appropriate credit to the original author(s) and the source, provide a link to the Creative Commons licence, and indicate if changes were made. The images or other third party material in this article are included in the article’s Creative Commons licence, unless indicated otherwise in a credit line to the material. If material is not included in the article’s Creative Commons licence and your intended use is not permitted by statutory regulation or exceeds the permitted use, you will need to obtain permission directly from the copyright holder. To view a copy of this licence, visit http://creativecommons.org/licenses/by/4.0/. © The Author(s) 2026

# PoseLib
This library provides a collection of minimal solvers for camera pose estimation. The focus is on calibrated absolute pose estimation problems from different types of correspondences (e.g. point-point, point-line, line-point, line-line).

The goals of this project are
* Fast and robust implementation of the current state-of-the-art solvers.
* Consistent calling interface between different solvers.
* Minimize dependencies, both external (currently only [Eigen](http://eigen.tuxfamily.org/)) and internal. Each solver is (mostly) stand-alone, making it easy to extract only a specific solver to integrate into other frameworks.


## P4Pf-related tinkering

* Added companion-matrix based root finding
* Rotation orthogonalization
* Solution selection:
* * Norm-based (original)
* * SVD-based
* * Reprojection-based
* Changed RNG to Merssene Twister 'cause we do not care for RNG speed

Results (intel xeon platinum 8176):
|                          Solver|       Solutions|           Valid|        1+ valid|        GT found|    0.5 quantile|    0.9 quantile|   0.95 quantile|   0.99 quantile|  0.999 quantile|         Runtime|
|--------------------------------|----------------|----------------|----------------|----------------|----------------|----------------|----------------|----------------|----------------|----------------|
|                             p3p|         1.54132|             100|             100|          99.999|     1.70875e-14|     6.30032e-13|     1.99624e-12|     1.89893e-11|     3.57453e-10|      354.442 ns|
|                            gp3p|          3.8834|             100|             100|             100|     5.04103e-14|     2.96697e-12|     1.26918e-11|     2.97308e-10|     7.56792e-09|      9.25425 us|
|                           gp4ps|               1|             100|             100|             100|     1.38662e-13|     1.03461e-11|     4.74992e-11|     7.39838e-10|     1.03605e-08|      9.54978 us|
|                     gp4ps(Deg.)|          2.4004|             100|             100|             100|     1.46339e-14|      2.2097e-13|     5.88378e-13|     1.14875e-11|     7.66636e-10|       382.72 ns|
|             p4pf-sturm-raw-norm|               1|           99.99|           99.99|           99.62|     2.29757e-12|     8.68679e-10|     7.63245e-09|     2.19184e-07|     4.16503e-06|      3.57444 us|
|               p4pf-eig-raw-norm|               1|             100|             100|           99.71|     2.62526e-12|     6.49413e-10|     4.79255e-09|     1.72913e-07|     3.32927e-06|      10.7443 us|
|            p4pf-sturm-orth-norm|               1|           99.99|           99.99|           99.67|     2.28714e-12|     6.05754e-10|     6.03867e-09|     1.90439e-07|     3.71873e-06|      6.08474 us|
|              p4pf-eig-orth-norm|               1|             100|             100|           99.81|     2.64061e-12|     7.21132e-10|     4.69758e-09|     1.14881e-07|     1.68381e-06|       13.332 us|
|              p4pf-sturm-raw-svd|               1|           99.98|           99.98|            99.6|      2.1747e-12|     6.97412e-10|     6.59875e-09|      2.4951e-07|     3.30453e-06|      5.90281 us|
|                p4pf-eig-raw-svd|               1|             100|             100|            99.8|     2.77114e-12|     7.41165e-10|     4.92646e-09|     1.45777e-07|     2.06556e-06|      13.2298 us|
|             p4pf-sturm-orth-svd|               1|           99.98|           99.98|           99.72|     2.24814e-12|      7.3915e-10|     6.81732e-09|     1.69381e-07|     2.20537e-06|      6.07497 us|
|               p4pf-eig-orth-svd|               1|             100|             100|           99.82|     2.63657e-12|     6.90292e-10|     4.81907e-09|     1.40823e-07|     1.38727e-06|      13.3242 us|
|           p4pf-sturm-raw-reproj|          4.9502|         20.1992|           99.98|           99.62|     2.28083e-12|     7.18691e-10|     6.12073e-09|     2.65792e-07|     4.76313e-06|      3.47324 us|
|             p4pf-eig-raw-reproj|           4.951|            20.2|             100|           99.67|     2.64176e-12|     7.16259e-10|     5.49135e-09|     1.57185e-07|     2.88571e-06|      10.7592 us|
|          p4pf-sturm-orth-reproj|               1|           99.97|           99.97|           99.69|     2.18334e-12|      6.4344e-10|     5.02878e-09|     1.57333e-07|     4.82445e-06|      6.20966 us|
|            p4pf-eig-orth-reproj|               1|             100|             100|            99.8|     2.77471e-12|     7.13368e-10|     5.09666e-09|     1.63266e-07|     2.45603e-06|      13.3859 us|
|                          p2p2pl|           7.072|          99.802|             100|            99.5|     4.95315e-13|     8.03844e-11|     5.48586e-10|     2.25845e-07|     0.000364089|      46.8271 us|
|                            p6lp|           4.615|             100|             100|             100|     7.01582e-14|     5.02877e-12|     2.22841e-11|     4.08681e-10|     5.76465e-09|       9.4837 us|
|                     p5lp_radial|         2.50154|         99.9944|             100|          99.997|     3.55058e-15|     2.79106e-14|     6.15883e-14|     3.77537e-13|     6.77119e-12|      1.63264 us|
|                          p2p1ll|               4|             100|             100|             100|     3.62682e-14|     1.67686e-12|     7.79845e-12|     2.52747e-10|     6.72163e-09|      8.87257 us|
|                          p1p2ll|          4.4816|             100|             100|             100|     4.45656e-14|      3.3681e-12|     1.53352e-11|     3.70966e-10|     1.43759e-08|      9.17733 us|
|                            p3ll|            4.59|             100|             100|             100|     5.43706e-14|     3.35099e-12|     1.62899e-11|     4.52297e-10|     2.13649e-08|      9.39211 us|
|                            up2p|               2|             100|             100|             100|     1.71923e-15|     1.24023e-14|     2.70182e-14|     1.57424e-13|     1.83888e-12|      136.087 ns|
|                           ugp2p|               2|             100|             100|             100|     1.91317e-15|     1.37189e-14|     2.95705e-14|      1.7321e-13|     2.09697e-12|      145.988 ns|
|                          ugp3ps|               1|             100|             100|             100|     4.62176e-15|     2.48931e-14|     4.92622e-14|     2.51661e-13|     2.44594e-12|      433.628 ns|
|                         up1p2pl|          3.3702|             100|             100|             100|     3.41913e-15|     3.17468e-14|     7.25758e-14|     4.57632e-13|     1.50523e-11|      442.231 ns|
|                           up4pl|           4.489|         99.9309|             100|           99.81|     5.42019e-14|     9.39954e-12|      5.2446e-11|     5.44644e-09|     3.69778e-06|      2.03427 us|
|                          ugp4pl|          4.4632|         99.9328|             100|           99.81|     5.59485e-14|      9.9402e-12|     6.78239e-11|     7.28606e-09|     3.74314e-06|      2.06463 us|
|                   RelUpright3pt|          2.5236|             100|             100|           99.99|     1.28181e-14|     3.06889e-13|     8.79089e-13|     8.44166e-12|     8.19086e-10|      399.684 ns|
|                GenRelUpright4pt|            4.61|         99.7701|           99.99|           99.75|     3.34477e-13|     8.23445e-11|     5.00845e-10|     3.03126e-08|     9.77942e-05|      2.06656 us|
|                          Rel8pt|               1|             100|             100|             100|     3.42184e-15|       2.214e-14|     4.66647e-14|     2.04346e-13|     2.41084e-12|      3.33284 us|
|                 Rel8pt(100 pts)|               1|             100|             100|             100|     2.46297e-15|     5.76859e-15|      7.4367e-15|      1.1781e-14|     1.76174e-14|      8.58185 us|
|                      Rel5pt-eig|           4.746|         99.4079|             100|           99.46|     5.76107e-14|     5.24738e-11|     5.97927e-10|      1.5665e-07|     5.74609e-05|      20.7846 us|
|                    Rel5pt-sturm|          4.7382|         99.2402|           99.95|           99.34|     4.46698e-14|     7.84008e-11|     1.17966e-09|     2.79217e-07|      0.00115815|      7.80543 us|
|             RelUprightPlanar2pt|               2|             100|             100|             100|     9.40596e-16|     7.27208e-15|     1.74986e-14|     1.76443e-13|     6.10143e-12|       198.38 ns|
|             RelUprightPlanar3pt|               1|             100|             100|             100|     1.01619e-15|     7.43331e-15|     1.50889e-14|     7.10001e-14|     6.87767e-13|      446.286 ns|



## Naming convention
For the solver names we use a slightly non-standard notation where we denote the solver as

<pre>
p<b><i>X</i></b>p<b><i>Y</i></b>pl<b><i>Z</i></b>lp<b><i>W</i></b>ll
</pre>

where the number of correspondences required is given by
* <b><i>X</i></b>p - 2D point to 3D point,
* <b><i>Y</i></b>pl - 2D point to 3D line,
* <b><i>Z</i></b>lp - 2D line to 3D point,
* <b><i>W</i></b>ll - 2D line to 2D line.

The prefix with `u` is for upright solvers and  `g` for generalized camera solvers. Solvers that estimate focal length have the postfix with `f` and similarly `s` for solvers that estimate scale.

## Calling conventions
All solvers return their solutions as a vector of `CameraPose` structs, which defined as
```
struct CameraPose {
   Eigen::Matrix3d R;
   Eigen::Vector3d t;
   double alpha = 1.0; // either focal length or scale
};
```
where `[R t]` maps from the world coordinate system into the camera coordinate system.


For <b>2D point to 3D point</b> correspondences, the image points are represented as unit-length bearings vectors. The returned camera poses `(R,t)` then satisfies (for some `lambda`)
```
  lambda * x[i] = R * X[i] + t
```
where `x[i]` is the 2D point and `X[i]` is the 3D point.
<b>Note</b> that only the P3P solver filters solutions with negative `lambda`.

Solvers that use point-to-point constraints take one vector with bearing vectors `x` and one vector with the corresponding 3D points `X`, e.g. for the P3P solver the function declaration is

```
int p3p(const std::vector<Eigen::Vector3d> &x,
        const std::vector<Eigen::Vector3d> &X,
        std::vector<CameraPose> *output);
```
Each solver returns the number of real solutions found.

For constraints with <b>2D lines</b>, the lines are represented in homogeneous coordinates. In the case of 2D line to 3D point constraints, the returned camera poses then satisfies
```
  l[i].transpose() * (R * X[i] + t) = 0
```
where `l[i]` is the line and  `X[i]` is the 3D point.

For constraints with <b>3D lines</b>, the lines are represented by a 3D point `X` and a bearing vector `V`. In the case of 2D point to 3D point constraints
```
  lambda * x[i] = R * (X[i] + mu * V[i]) + t
```
for some values of `lambda` and `mu`. Similarly, for line to line constraints we have
```
  l[i].transpose() * (R * (X[i] + mu * V[i]) + t) = 0
```
### Generalized Cameras
For generalized cameras we represent the image rays similarly to the 3D lines above, with an offset `p` and a bearing vector `x`. For example, in the case of point-to-point correspondences we have
```
p[i] + lambda * x[i] = R * X[i] + t
```
In the case of unknown scale we also estimate `alpha` such that
```
alpha * p[i] + lambda * x[i] = R * X[i] + t
```
For example, the generalized pose and scale solver (from four points) has the following signature
```
 int gp4ps(const std::vector<Eigen::Vector3d> &p, const std::vector<Eigen::Vector3d> &x,
              const std::vector<Eigen::Vector3d> &X, std::vector<CameraPose> *output);
```

### Upright Solvers
For the upright solvers it assumed that the rotation is around the y-axis, i.e.
```
R = [a 0 -b; 0 1 0; b 0 a] 
```
To use these solvers it necessary to pre-rotate the input such that this is satisfied.

## Implemented solvers
The following solvers are currently implemented.

### Absolute Pose
| Solver | Point-Point | Point-Line | Line-Point | Line-Line | Upright | Generalized | Approx. runtime | Max. solutions | Comment |
| --- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | --- |
| `p3p` | 3 | 0 | 0| 0|  |  | 250 ns | 4 | Persson and Nordberg, LambdaTwist (ECCV18) |
| `gp3p` | 3 | 0 | 0| 0|  | :heavy_check_mark:  | 1.6 us | 8 | Kukelova et al., E3Q3 (CVPR16) |
| `gp4ps` | 4 | 0 | 0| 0|  | :heavy_check_mark: | 1.8 us | 8 | Unknown scale.<br> Kukelova et al., E3Q3 (CVPR16)<br>Camposeco et al.(ECCV16) |
| `p4pf` | 4 | 0 | 0| 0|  |  | 2.3 us | 8 | Unknown focal length.<br> Kukelova et al., E3Q3 (CVPR16) |
| `p2p2pl` | 2 | 2 | 0| 0|  |  | 30 us | 16 | Josephson et al. (CVPR07) |
| `p6lp` | 0 | 0 | 6|  0| |  | 1.8 us | 8 | Kukelova et al., E3Q3 (CVPR16)  |
| `p5lp_radial` | 0 | 0 | 5|  0| |  | 1 us | 4 | Kukelova et al., (ICCV13)  |
| `p2p1ll` | 2 | 0 | 0 |  1| |  | 1.6 us | 8 | Kukelova et al., E3Q3 (CVPR16), Zhou et al. (ACCV18)  |
| `p1p2ll` | 1 | 0 | 0 |  2| |  | 1.7 us | 8 | Kukelova et al., E3Q3 (CVPR16), Zhou et al. (ACCV18)  |
| `p3ll` | 0 | 0 | 0 |  3| |  | 1.8 us | 8 | Kukelova et al., E3Q3 (CVPR16), Zhou et al. (ACCV18)  |
| `up2p` | 2 | 0 | 0| 0| :heavy_check_mark: |  | 65 ns | 2 | Kukelova et al. (ACCV10) |
| `ugp2p` | 2 | 0 | 0| 0| :heavy_check_mark: | :heavy_check_mark: | 65 ns | 2 | Adapted from Kukelova et al. (ACCV10)   |
| `ugp3ps` | 3 | 0 | 0| 0| :heavy_check_mark: | :heavy_check_mark: | 390 ns | 2 | Unknown scale. Adapted from Kukelova et al. (ACCV10)  |
| `up1p2pl` | 1 | 2 | 0| 0| :heavy_check_mark: |  | 370 ns | 4 |  |
| `up4pl` | 0 | 4 | 0| 0| :heavy_check_mark: |  | 1.4 us | 8 | Sweeney et al. (3DV14) |
| `ugp4pl` | 0 | 4 | 0| 0| :heavy_check_mark: | :heavy_check_mark: | 1.4 us | 8 | Sweeney et al. (3DV14) |


### Relative Pose
| Solver | Point-Point | Upright | Planar | Generalized | Approx. runtime | Max. solutions | Comment |
| --- | :---: | :---: | :---: | :---: | :---: | :---: | --- |
| `relpose_5pt` | 5 | | | | 5.5 us | 10 | Nister (PAMI 2004) |
| `relpose_8pt` | 8+ | | | | 2.2+ us | 1 |  |
| `relpose_upright_3pt` | 3 | :heavy_check_mark: | | | 210 ns | 4 | Sweeney et al. (3DV14)  | 
| `gen_relpose_upright_4pt` | 4 | :heavy_check_mark: | | :heavy_check_mark:  | 1.2 us | 6 | Sweeney et al. (3DV14)  | 
| `relpose_upright_planar_2pt` | 2 | :heavy_check_mark: | :heavy_check_mark: | | 120 ns | 2 | Choi and Kim (IVC 2018)  | 
| `relpose_upright_planar_3pt` | 3 | :heavy_check_mark: | :heavy_check_mark: | | 300 ns | 1 |  Choi and Kim (IVC 2018) | 



## How to compile?

Getting the code:

    > git clone --recursive https://github.com/vlarsson/PoseLib.git
    > cd PoseLib

Example of a local installation:

    > mkdir _build && cd _build
    > cmake -DCMAKE_INSTALL_PREFIX=../_install ..
    > cmake --build . --target install -j 8
      (equivalent to  'make install -j8' in linux)

Installed files:

    > tree ../installed
      .
      ├── bin
      │   └── benchmark
      ├── include
      │   └── PoseLib
      │       ├── gp3p.h
      │       ├──  ...
      │       ├── poselib.h          <==  Library header (includes all the rest)
      │       ├──  ...
      │       └── version.h
      └── lib
          ├── cmake
          │   └── PoseLib
          │       ├── PoseLibConfig.cmake
          │       ├── PoseLibConfigVersion.cmake
          │       ├── PoseLibTargets.cmake
          │       └── PoseLibTargets-release.cmake
          └── libPoseLib.a

Uninstall library:

    > make uninstall


## Benchmark

Conditional compilation of `benchmark` binary is controlled by `WITH_BENCHMARK` option. Default if OFF (without benchmark).

Add `-DWITH_BENCHMARK=ON` to cmake to activate.

    > cmake -DWITH_BENCHMARK=ON ..


## Use library (as dependency) in an external project.

    cmake_minimum_required(VERSION 3.13)
    project(Foo)

    find_package(PoseLib REQUIRED)

    add_executable(foo foo.cpp)
    target_link_libraries(foo PRIVATE PoseLib::PoseLib)


## Citing
If you are using the library for (scientific) publications, please cite the following source:
```
@misc{PoseLib,
  title = {{PoseLib - Minimal Solvers for Camera Pose Estimation}},
  author = {Viktor Larsson},
  URL = {https://github.com/vlarsson/PoseLib},
  year = {2020}
}
```
Please cite also the original publications of the different methods (see table above).


## License
PoseLib is licensed under the BSD 3-Clause license. Please see [License](https://github.com/vlarsson/PoseLib/blob/master/LICENSE) for details.

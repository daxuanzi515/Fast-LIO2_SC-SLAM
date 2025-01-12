### APE
```bash
# original
evo_ape tum DCC02.txt trajectory_tum.txt --plot --save_results results_ape_ori.zip --align
# fast-lio2
evo_ape tum DCC02.txt odom_poses_tum.txt --plot --save_results results_ape_fast_lio2.zip --align
```

#### Original
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	16.240410
      mean	10.318144
    median	10.263940
       min	3.669080
      rmse	10.724899
       sse	120314.535131
       std	2.925638
```
#### fast-lio2
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	18.947324
      mean	9.301205
    median	9.194239
       min	5.253482
      rmse	9.716993
       sse	98763.278883
       std	2.812037
```

### RPE
```bash
# original
evo_rpe tum DCC02.txt trajectory_tum.txt --plot --save_results results_rpe_ori.zip --align
# fast-lio2
evo_rpe tum DCC02.txt odom_poses_tum.txt --plot --save_results results_rpe_fast_lio2.zip --align
```
#### Original
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	64.435630
      mean	4.879659
    median	4.515211
       min	3.657652
      rmse	5.420973
       sse	30709.356020
       std	2.361329

```

#### fast-lio2
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	64.435685
      mean	4.864208
    median	4.513983
       min	3.657652
      rmse	5.386329
       sse	30318.106039
       std	2.313444
```

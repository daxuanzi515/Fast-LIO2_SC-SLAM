### APE
```bash
# original
evo_ape tum KAIST03.txt trajectory_tum.txt --plot --save_results results_ape_ori.zip --align
# fast-lio2
evo_ape tum KAIST03.txt odom_poses_tum.txt --plot --save_results results_ape_fast_lio2.zip --align
```

#### Original
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	13.922782
      mean	4.073462
    median	3.502793
       min	0.342736
      rmse	4.821602
       sse	33802.368354
       std	2.579681
```
#### fast-lio2
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	35.752979
      mean	17.389047
    median	16.274581
       min	1.463005
      rmse	19.398395
       sse	547136.907263
       std	8.597603
```

### RPE
```bash
# original
evo_rpe tum KAIST03.txt trajectory_tum.txt --plot --save_results results_rpe_ori.zip --align
# fast-lio2
evo_rpe tum KAIST03.txt odom_poses_tum.txt --plot --save_results results_rpe_fast_lio2.zip --align
```
#### Original
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	28.226883
      mean	4.839445
    median	4.795775
       min	2.291302
      rmse	4.932679
       sse	35353.407881
       std	0.954513
```

#### fast-lio2
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	28.226903
      mean	4.839443
    median	4.795461
       min	2.290893
      rmse	4.932681
       sse	35353.445429
       std	0.954533
```
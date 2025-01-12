### APE
```bash
# original
evo_ape tum DCC03.txt trajectory_tum.txt --plot --save_results results_ape_ori.zip --align
# fast-lio2
evo_ape tum DCC03.txt odom_poses_tum.txt --plot --save_results results_ape_fast_lio2.zip --align
```

#### Original
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	12.815916
      mean	4.705064
    median	4.006220
       min	1.228029
      rmse	5.199675
       sse	43069.337261
       std	2.213366
```
#### fast-lio2
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	35.244882
      mean	12.483178
    median	10.519327
       min	3.040432
      rmse	14.802341
       sse	349041.096322
       std	7.954845
```

### RPE
```bash
# original
evo_rpe tum DCC03.txt trajectory_tum.txt --plot --save_results results_rpe_ori.zip --align
# fast-lio2
evo_rpe tum DCC03.txt odom_poses_tum.txt --plot --save_results results_rpe_fast_lio2.zip --align
```
#### Original
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	98.243909
      mean	4.922309
    median	4.746313
       min	3.721802
      rmse	5.574593
       sse	49473.126221
       std	2.616670
```

#### fast-lio2
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	98.243685
      mean	4.922310
    median	4.746784
       min	3.722320
      rmse	5.574589
       sse	49473.064222
       std	2.616659
```
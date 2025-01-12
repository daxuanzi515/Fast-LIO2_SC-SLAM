### APE
```bash
# original
evo_ape tum DCC01.txt trajectory_tum.txt --plot --save_results results_ape_ori.zip --align
# fast-lio2
evo_ape tum DCC01.txt odom_poses_tum.txt --plot --save_results results_ape_fast_lio2.zip --align
```

#### Original
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	34.092491
      mean	8.994430
    median	7.295394
       min	2.618141
      rmse	10.829194
       sse	199009.652053
       std	6.030893
```
#### fast-lio2
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	45.349274
      mean	21.104382
    median	20.659201
       min	4.351231
      rmse	23.056536
       sse	902131.720187
       std	9.284875
```

### RPE
```bash
# original
evo_rpe tum DCC01.txt trajectory_tum.txt --plot --save_results results_rpe_ori.zip --align
# fast-lio2
evo_rpe tum DCC01.txt odom_poses_tum.txt --plot --save_results results_rpe_fast_lio2.zip --align
```
#### Original
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	57.697185
      mean	5.323362
    median	4.805852
       min	3.463479
      rmse	6.124221
       sse	63610.316473
       std	3.027854

```

#### fast-lio2
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	57.697827
      mean	5.323364
    median	4.805640
       min	3.463051
      rmse	6.124228
       sse	63610.468461
       std	3.027866

```
## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/interpolate.py
```

### APE
```bash
# lio-sam
evo_ape tum DCC01.txt interpolated_poses.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	84.626999
      mean	43.596333
    median	45.187126
       min	1.877158
      rmse	50.539726
       sse	316728.718710
       std	25.566064
```
### RPE
```bash
# lio-sam
evo_rpe tum DCC01.txt interpolated_poses.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	3.952420
      mean	1.481529
    median	1.122437
       min	1.040823
      rmse	1.603896
       sse	316.415440
       std	0.614454
```

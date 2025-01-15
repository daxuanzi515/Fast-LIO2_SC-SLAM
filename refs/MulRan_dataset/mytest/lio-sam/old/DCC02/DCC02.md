## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/interpolate.py
```


### APE
```bash
# lio-sam
evo_ape tum DCC02.txt interpolated_poses.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	84.805483
      mean	44.085149
    median	42.458562
       min	10.513456
      rmse	49.667190
       sse	345356.169491
       std	22.876395
```
### RPE
```bash
# lio-sam
evo_rpe tum DCC02.txt interpolated_poses.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	2.168335
      mean	1.277395
    median	1.202972
       min	1.027790
      rmse	1.298773
       sse	234.466812
       std	0.234677

```

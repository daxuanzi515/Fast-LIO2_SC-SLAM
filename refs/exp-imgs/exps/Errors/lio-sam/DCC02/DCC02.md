## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
```

### APE
```bash
# lio-sam
evo_ape tum DCC02.txt trajectory_tum.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	20.744551
      mean	9.384334
    median	7.681438
       min	0.460641
      rmse	11.026738
       sse	102864.244094
       std	5.789924

```
### RPE
```bash
# lio-sam
evo_rpe tum DCC02.txt trajectory_tum.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	47.629798
      mean	3.482633
    median	2.733605
       min	1.969667
      rmse	4.194455
       sse	14866.464834
       std	2.337673
```

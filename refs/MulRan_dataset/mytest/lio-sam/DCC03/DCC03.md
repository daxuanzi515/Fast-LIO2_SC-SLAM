## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/interpolate.py
```


### APE
```bash
# lio-sam
evo_ape tum DCC03.txt interpolated_poses.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	153.139271
      mean	69.991159
    median	67.588549
       min	2.550044
      rmse	80.947466
       sse	1087713.709591
       std	40.666079
```
### RPE
```bash
# lio-sam
evo_rpe tum DCC03.txt interpolated_poses.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	9.680834
      mean	1.843152
    median	1.712914
       min	1.021345
      rmse	2.067028
       sse	704.979800
       std	0.935625
```

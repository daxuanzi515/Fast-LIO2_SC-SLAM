## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
```

### APE
```bash
# lio-sam
evo_ape tum DCC03.txt trajectory_tum.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
       max	69.111100
      mean	24.781316
    median	22.002032
       min	4.212821
      rmse	28.376741
       sse	2403639.764931
       std	13.824826
```
### RPE
```bash
# lio-sam
evo_rpe tum DCC03.txt trajectory_tum.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	23.025714
      mean	3.647584
    median	3.237468
       min	1.909435
      rmse	4.165164
       sse	51768.186739
       std	2.010900
```

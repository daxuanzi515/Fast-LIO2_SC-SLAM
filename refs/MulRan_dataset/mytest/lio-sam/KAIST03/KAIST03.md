## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
```

### APE
```bash
# lio-sam
evo_ape tum KAIST03.txt trajectory_tum.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	94.181049
      mean	41.125024
    median	31.960102
       min	11.760686
      rmse	47.720731
       sse	7316862.584252
       std	24.207448

```
### RPE
```bash
# lio-sam
evo_rpe tum KAIST03.txt trajectory_tum.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	26.775890
      mean	3.897367
    median	3.348582
       min	1.909467
      rmse	4.394070
       sse	62016.824714
       std	2.029380
```

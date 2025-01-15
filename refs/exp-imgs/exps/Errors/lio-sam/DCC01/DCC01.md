## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
```

### APE
```bash
# lio-sam
evo_ape tum DCC01.txt trajectory_tum.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	74.389304
      mean	37.224053
    median	37.062619
       min	12.002518
      rmse	41.245585
       sse	4418011.877807
       std	17.764237

```
### RPE
```bash
# lio-sam
evo_rpe tum DCC01.txt trajectory_tum.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	23.121985
      mean	3.157556
    median	2.626344
       min	1.900761
      rmse	3.481090
       sse	31458.302307
       std	1.465548
```

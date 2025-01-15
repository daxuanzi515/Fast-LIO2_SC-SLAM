## prework
```
python3 /home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/lio_sam_convert.py
```

### APE
```bash
# lio-sam
evo_ape tum KAIST02.txt trajectory_tum.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	75.314873
      mean	30.470002
    median	24.302248
       min	13.897310
      rmse	34.104214
       sse	3674224.706459
       std	15.319150
```
### RPE
```bash
# lio-sam
evo_rpe tum KAIST02.txt trajectory_tum.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	27.365806
      mean	3.786302
    median	3.217727
       min	1.908574
      rmse	4.321747
       sse	58983.537548
       std	2.083606
```

### APE
```bash
# lio-sam
evo_ape tum Riverside02.txt interpolated_poses.txt --plot --save_results results_ape_lio_sam.zip --align
```

#### lio-sam
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	66.314365
      mean	34.413647
    median	34.601471
       min	0.646122
      rmse	39.288781
       sse	138924.748193
       std	18.955454
```
### RPE
```bash
# lio-sam
evo_rpe tum Riverside02.txt interpolated_poses.txt --plot --save_results results_rpe_lio_sam.zip --align
```
#### lio-sam
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	4.205201
      mean	1.489558
    median	1.255359
       min	1.027858
      rmse	1.604879
       sse	229.231752
       std	0.597373
```
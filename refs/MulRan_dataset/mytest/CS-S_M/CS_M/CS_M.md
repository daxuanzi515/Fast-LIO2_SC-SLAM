# CS_M: Ceres Multiple threads

### APE
```bash
evo_ape tum KAIST02.txt trajectory_tum.txt --plot --save_results results_ape_csm.zip --align
```

#### CS_M
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	43.510113
      mean	15.777419
    median	15.312251
       min	0.953418
      rmse	17.925657
       sse	1255111.803482
       std	8.508951
```


### RPE
```bash
evo_rpe tum KAIST02.txt trajectory_tum.txt --plot --save_results results_rpe_csm.zip --align
```
#### CS_M
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	47.562041
      mean	2.839643
    median	2.754754
       min	1.364132
      rmse	2.980939
       sse	34699.818719
       std	0.906878
```
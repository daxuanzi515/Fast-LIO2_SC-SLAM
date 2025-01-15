# CS_S: Ceres Single thread


### APE
```bash
evo_ape tum KAIST02.txt trajectory_tum.txt --plot --save_results results_ape_css.zip --align
```

#### CS_S
```bash
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	107.191481
      mean	53.982495
    median	50.651663
       min	15.848998
      rmse	57.356930
       sse	8898956.047081
       std	19.383179
```

### RPE
```bash
evo_rpe tum KAIST02.txt trajectory_tum.txt --plot --save_results results_rpe_css.zip --align
```
#### CS_S
```bash
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(with SE(3) Umeyama alignment)

       max	495.122864
      mean	3.992334
    median	2.769708
       min	1.364133
      rmse	19.673271
       sse	1046549.644540
       std	19.263926
```

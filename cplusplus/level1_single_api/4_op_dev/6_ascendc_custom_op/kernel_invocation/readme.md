### Demostration
```shell
bash run.sh [KERNEL_NAME](add_custom/matmul_custom/topk_custom) [SOC_VERSION](ascend910/ascend310p) [CORE_TYPE](AiCore/VectorCore) [RUN_MODE](cpu/npu)
```
### NOTICE
THEY ARE JUST DEMOS, NO DFX DEFENSE, do not type invalid command!!!
actually all that you can run:
#### On 1980 board
```shell
(cd Add; bash run.sh add_custom ascend910 AiCore cpu)
(cd Add; bash run.sh add_custom ascend910 AiCore npu)

(cd TopK; bash run.sh topk_custom ascend910 AiCore cpu)  # Not reocommanded, too time-consuming
(cd TopK; bash run.sh topk_custom ascend910 AiCore npu)

(cd MatMul; bash run.sh matmul_custom ascend910 AiCore cpu)
(cd MatMul; bash run.sh matmul_custom ascend910 AiCore npu)
```

#### On 1951 board
```shell
(cd Add; bash run.sh add_custom ascend310p AiCore cpu)
(cd Add; bash run.sh add_custom ascend310p AiCore npu)
(cd Add; bash run.sh add_custom ascend310p VectorCore cpu)
(cd Add; bash run.sh add_custom ascend310p VectorCore npu)  # Not yet adapted

(cd TopK; bash run.sh topk_custom ascend310p AiCore cpu)  # Not reocommanded, too time-consuming
(cd TopK; bash run.sh topk_custom ascend310p AiCore npu)
(cd TopK; bash run.sh topk_custom ascend310p VectorCore cpu)  # Not reocommanded, too time-consuming
(cd TopK; bash run.sh topk_custom ascend310p VectorCore npu)  # Not yet adapted

(cd MatMul; bash run.sh matmul_custom ascend310p AiCore cpu)
(cd MatMul; bash run.sh matmul_custom ascend310p AiCore npu)
```

#### profiling
```shell
(cd Add; msprof --application="./add_custom_npu" --output="./out" --aic-metrics="PipeUtilization" --sys-hardware-mem=on --sys-io-profiling=on --sys-interconnection-profiling=on)
(cd Add; msprof --application="./add_custom_npu" --output="./out" --aic-metrics="Memory")
(cd Add; msprof --application="./add_custom_npu" --output="./out" --aic-metrics="MemoryUB")

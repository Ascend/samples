# export ASCEND_OPP_PATH=/usr/local/Ascend/opp
# export PATH=/usr/local/Ascend/atc/ccec_compiler/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:$PATH
# export LD_LIBRARY_PATH=/usr/local/Ascend/acllib/lib64:/usr/local/HiAI/acllib/lib64/stub:/usr/local/Ascend/atc/lib64:/usr/local/Ascend/driver/lib64/driver:/usr/local/Ascend/driver/lib64/common:/usr/local/Ascend/add-ons/:/root/host:/usr/local/lib:/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
# export PYTHONPATH=/usr/local/Ascend/atc/python/topi.egg:/usr/local/Ascend/atc/python/te.egg:/usr/local/Ascend/opp/op_impl/built-in/ai_core/tbe:$PYTHONPATH

atc --model=../../models/snapshots/models.pb --framework=3 --input_shape="w:500,16" --output=../../models/snapshots/models --soc_version=Ascend310 --log=debug --out_nodes="s/logits:0"

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export CUDA_VISIBLE_DEVICES=0

log_name=log17
nohup python3 -u main.py > $log_name 2>&1 &
tail -f $log_name 

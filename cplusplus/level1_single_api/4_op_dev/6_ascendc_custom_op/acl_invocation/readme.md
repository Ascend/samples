# acl samples
cd acl_offline_model/acl_online_model
bash run_torch.sh ${is_dynamic}(0/1) ${replay_mode}(/batch/iterator)

# run static op (depend on chip version)
(cd acl_offline_model; bash run.sh 0)
(cd acl_offline_model; bash run.sh 0 batch)
(cd acl_offline_model; bash run.sh 0 iterator)

(cd acl_online_model; bash run.sh 0)
(cd acl_online_model; bash run.sh 0 batch)
(cd acl_online_model; bash run.sh 0 iterator)

# run dynamic op (depend on chip version)
(cd acl_offline_model; bash run.sh 1)
(cd acl_online_model; bash run.sh 1)
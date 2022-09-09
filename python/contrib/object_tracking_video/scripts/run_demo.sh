# download om model
# Google Drive
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1RU1UBVH5EBbVV4CVAPuNokSzpfx9A3Ug' -O ../model/mot_v2.om
# Huawei Cloud
wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/object_tracking_video/mot_v2.om' -O ../model/mot_v2.om

# download sample video london.mp4
# Google Drive
wget -nc --no-check-certificate 'https://docs.google.com/uc?export=download&id=1ntbudc1JB8HzEw38pwZKPXukrgADiKdS' -O ../data/london.mp4
# Huawei Cloud
wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/object_tracking_video/london.mp4' -O ../data/london.mp4

cd ../src
python3 main.py --conf_thres 0.35 --input_video ../data/london.mp4
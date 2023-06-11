#!/bin/bash
card_id=0   #使用卡0
training_arch_list=0 #训练training_arch/arch0.txt文件中的架构
python train_arch.py --card_id $card_id --training_arch_list $training_arch_list
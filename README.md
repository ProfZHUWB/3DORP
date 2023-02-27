The first version of source code is developed by Mr. You Zhou
It is restructured and improved by Ms. Ying Fu and Prof. Wenbin Zhu
This project is currently maintained by Prof. Wenbin Zhu (i@zhuwb.com)

# 3DORP
The source code for our paper 3D Online Robotic Palletization Problem

## Files

	RoboticPacking/		An eclipse Java project
		data/
			ZhuFuZhou2/	288 instance generated with random seed 2			
		result/			Detailed experiment results reported in Section 5 of paper
		src/			Java source code

## Generate instance

Run com.zhuwb.research.roboticpacking.inst.Generator will generate the 288 instances used in Section 5.1

## Repeat experiments

For Section 5.2 - 5.5
Run com.zhuwb.research.roboticpacking.exp.MainOnline
Solution file will be written in subdirectories under results/


package com.zhuwb.research.roboticpacking.exp;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import com.zhuwb.research.roboticpacking.exp.Main.ExpConfig;
import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.Vacuum.Type;
import com.zhuwb.research.roboticpacking.search.AlgoConfig;
import com.zhuwb.research.roboticpacking.search.LookAheadOnline.SelectMode;
import com.zhuwb.research.rpp2i.boxpp.segmenttree.GridPointBySegmentTree;

public class Main4Online {

	public static void main(String[] args) throws IOException {
		long instSeed = 2;
		int algoSeed = 2;
		
		expAlgoSelectMode(instSeed, algoSeed, new File("result/2021-07-05-selectMode-"+instSeed));
		expAlgoSimCount(instSeed, algoSeed, new File("result/2021-07-05-simCount-"+instSeed));		
		expBoxCountInRange(instSeed, algoSeed, new File("result/2021-07-05-boxCountInRange-"+instSeed));
		expKnowBoxCount(instSeed, algoSeed, new File("result/2021-07-05-knowBoxCount-"+instSeed));
		expFinal(instSeed, algoSeed, new File("result/2021-07-05-final-"+instSeed));
		
		System.out.println("gridCount: "+GridPointBySegmentTree.gridCount+
				"; segmentTreeCount: "+GridPointBySegmentTree.segmentTreeCount+
				"; bruteForceCout: "+GridPointBySegmentTree.bruteForceCount);
	}
	
	
	// 这个实验是在找simulationCount,其它的算法参数随机选
	public static void expAlgoSelectMode(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfigOnline(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (SelectMode selectMode: SelectMode.values()) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = 16;
				conf.algoConf.hweight = 1;
				conf.sysConf.openPalletCount=1;
				conf.algoConf.selectMode = selectMode;
				expList.add(conf);
			}
		}
		
		Main.runAll(expList, outdir);
	}
	
	
	// 这个实验是在找simulationCount,其它的算法参数随机选
	public static void expAlgoSimCount(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfigOnline(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.inst.getBoxCount()>1000) {continue;}
			for (int simCount : new int[]{2,4,8}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = 16;
				conf.algoConf.hweight = 1;
				conf.sysConf.openPalletCount = 1;
				conf.algoConf.selectMode=SelectMode.voting;
				conf.algoConf.simulationCount = simCount;   
				expList.add(conf);
			}
		}
		
		Main.runAll(expList, outdir);
	}
	
	
	
	// 这个实验是在找openPalletCount,其它的算法参数随机选
	// 跳过 instCount = 2000的例子
	public static void expBoxCountInRange(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfigOnline(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (int boxCountInRange : new int[] {1,2,3,4,5}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = 16;
				conf.algoConf.hweight = 1;	// 2; before 06-13
				conf.sysConf.openPalletCount = 1;
				conf.algoConf.selectMode=SelectMode.voting;
				conf.algoConf.simulationCount = 8;
				conf.sysConf.boxCountInRange = boxCountInRange;
				expList.add(conf);
			}
		}
		
		Main.runAll(expList, outdir);
	}
	
	
	// 这个实验是在找knowBoxCount,其它的算法参数随机选
	// 跳过 instCount = 2000的例子
	public static void expKnowBoxCount(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfigOnline(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (int knowBoxCount : new int[] {5,10,20,50}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = 16;
				conf.algoConf.hweight = 1;	// 2; before 06-13
				conf.sysConf.openPalletCount = 1;
				conf.algoConf.selectMode=SelectMode.voting;
				conf.algoConf.simulationCount = 8;
				conf.sysConf.knownBoxCount = knowBoxCount;
				expList.add(conf);
			}
		}
		
		Main.runAll(expList, outdir);
	}
	

//	// 这个实验是在分析knowBoxCount的价值,固定boxCountInRange = 1, openPalletCount = 1
//	// 跳过 instCount = 2000的例子
//	public static void expKnowBoxCountRange1Pallet1(long instSeed, long algoSeed, File outdir) throws IOException{
//		ArrayList<ExpConfig> confList = genExpConfigOnline(instSeed, true, -1, algoSeed);
//		
//		ArrayList<ExpConfig> expList = new ArrayList<>();
//		for (ExpConfig confTemplate: confList) {
//			if (confTemplate.instConf.boxCount > 1000) { continue; }
//			for (int knowBoxCount : new int[] {5,10,20,50}) {  
//				ExpConfig conf = confTemplate.deepCopy();
//				conf.algoConf.depth = 1; 
//				conf.algoConf.effort = 16;
//				conf.algoConf.hweight = 1;
//				conf.algoConf.selectMode=SelectMode.voting;
//				conf.algoConf.simulationCount = 8; 
//				conf.sysConf.openPalletCount = 1;
//				conf.sysConf.boxCountInRange = 1;
//				conf.sysConf.knownBoxCount = knowBoxCount;
//				expList.add(conf);
//			}
//		}
//		
//		Main.runAll(expList, outdir);
//	}
	
	// 固定了所有算法及系统参数，跑一个完整实验
	public static void expFinal(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfigOnline(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = 16;
				conf.algoConf.hweight = 1;
				conf.sysConf.openPalletCount = 1;
				conf.algoConf.selectMode=SelectMode.voting;
				conf.algoConf.simulationCount = 8; 
				conf.sysConf.boxCountInRange = 2;
				conf.sysConf.knownBoxCount = 50;
				expList.add(conf);
		}
		
		Main.runAll(expList, outdir);
	}
	

	
	public static final long CONFIG_GEN_SEED = 3; // 生成各种Config的随机参数
	public static final int selectedSize = 384;   
	
	/**
	 * 随机生成实验配置
	 * @param instSeed	测试数据集
	 * @param shuffle	是否打乱测试例子的顺序
	 * @param instCount	生成的配置数量，-1 表示所以测试集中的例子
	 * @param algoSeed	算法用的种子
	 * @return
	 * @throws IOException
	 */
	public static ArrayList<ExpConfig> genExpConfigOnline(long instSeed, boolean shuffle, int instCount, long algoSeed) throws IOException {
				
		// Algorithm
		int[] depthList = new int[]{0,1,2};  // depth=0,直接用greedy的算法去求解，根本不用
//		int[] nodesKeptList = new int[]{5,10,15,20};
		int[] effortList = new int[] {4, 9, 16, 25};
		int[] simulationCountList = new int[]{2,4,8};  
		SelectMode[] selectModeList = SelectMode.values();
		double[] hweightList = new double[]{0.2,0.5,1.0,2.0};

				
		// System configuration:
		// fixed		
		Type[] vaccumTypeList = {Type.TwoByThree}; // 机械手吸盘的长宽数量比
		double[] thetaList = {1.0};                // 机械手大小的缩放比例
		double[] maxDropHeightRatioList = {0.1};   // 一个盒子可以安全的自由下落盒子在H-轴上10%的长度
		
		double[] maxHGapForSupportList = {0.5};
		double[] minOverlapRatioForSupportList = {0.2};  // Zhou的alpha
						
		// changeable:
		int[] knownBoxCountList = new int[] {5,10,20,50};  		// known 系统知道未来的信息数量, N_k
		int[] openPalletCountList = new int[] {1,3,5};  	// pn 装载时同时打开的托盘数量, P
		int[] boxCountInRangeList = new int[] {1,3,5};  	//oper 机械手可以操作的个数, N_b

		
		ArrayList<InstData> instList = InstanceLoader.load(instSeed);
		if (shuffle) {
			Collections.shuffle(instList,new Random(CONFIG_GEN_SEED));
		}
		if (instCount == -1) {
			instCount = instList.size();
		}

		Random rand = new Random(CONFIG_GEN_SEED);
		ArrayList<ExpConfig> confList = new ArrayList<ExpConfig> ();
		for(int i=0; i<instCount; i++) {
			ExpConfig conf = new ExpConfig(instList.get(i));

			conf.instConf = InstanceLoader.loadConf(instSeed, conf.inst.name);

			conf.algoConf = new AlgoConfig();
			conf.sysConf = new SysConfig(); 
			
			// Algorithm
			conf.algoConf.seed = algoSeed;
			conf.algoConf.depth = depthList[rand.nextInt(depthList.length)];
			conf.algoConf.effort = effortList[rand.nextInt(effortList.length)];
			conf.algoConf.simulationCount = simulationCountList[rand.nextInt(simulationCountList.length)];
			conf.algoConf.selectMode = selectModeList[rand.nextInt(selectModeList.length)];
			conf.algoConf.hweight = hweightList[rand.nextInt(hweightList.length)];
			
			// System System configuration
			// fixed
			conf.sysConf.vaccumType = vaccumTypeList[rand.nextInt(vaccumTypeList.length)];
			conf.sysConf.theta = thetaList[rand.nextInt(thetaList.length)];
			conf.sysConf.maxDropHeightRatio = maxDropHeightRatioList[rand.nextInt(maxDropHeightRatioList.length)];
			
			conf.sysConf.maxHGapForSupport = maxHGapForSupportList[rand.nextInt(maxHGapForSupportList.length)];
			conf.sysConf.minOverlapRatioForSupport = minOverlapRatioForSupportList[rand.nextInt(minOverlapRatioForSupportList.length)];
	
			// changeable:
			conf.sysConf.knownBoxCount = knownBoxCountList[rand.nextInt(knownBoxCountList.length)];
			conf.sysConf.boxCountInRange = boxCountInRangeList[rand.nextInt(boxCountInRangeList.length)];
			conf.sysConf.openPalletCount = openPalletCountList[rand.nextInt(openPalletCountList.length)];
			confList.add(conf);
		}
		return confList;
	}

}



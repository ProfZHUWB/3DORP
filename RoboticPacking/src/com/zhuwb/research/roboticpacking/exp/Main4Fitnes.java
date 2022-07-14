package com.zhuwb.research.roboticpacking.exp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import com.zhuwb.research.roboticpacking.inst.InstConfig;
import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.SystemInfo;
import com.zhuwb.research.roboticpacking.inst.Vacuum.Type;
import com.zhuwb.research.roboticpacking.search.AlgoConfig;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.search.Fitness;
import com.zhuwb.research.roboticpacking.search.Greedy;
import com.zhuwb.research.roboticpacking.search.LookAhead;
import com.zhuwb.research.roboticpacking.search.LookAhead.LeafEvaluator;
import com.zhuwb.research.roboticpacking.search.LookAheadOnline;
import com.zhuwb.research.roboticpacking.search.Solution;
import com.zhuwb.research.roboticpacking.search.Solver;
import com.zhuwb.research.roboticpacking.search.State;
import com.zhuwb.research.roboticpacking.search.State.PalletFiteness;
import com.zhuwb.research.rpp2i.boxpp.segmenttree.GridPointBySegmentTree;

public class Main4Fitnes {

	public static void main(String[] args) throws IOException {
		long instSeed = 2;
		int algoSeed = 2;
		
		
		expAlgoFitness(instSeed, algoSeed, new File("result/2021-09-26-fitness-offline-"+instSeed));		

		System.out.println("gridCount: "+GridPointBySegmentTree.gridCount+
				"; segmentTreeCount: "+GridPointBySegmentTree.segmentTreeCount+
				"; bruteForceCout: "+GridPointBySegmentTree.bruteForceCount);
	}

	
	
	// 这个实验是在测试fitness,其它的算法参数随机选
	public static void expAlgoFitness(long instSeed, long algoSeed, File outdir) throws IOException{		
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			for (Fitness fitness : Fitness.values()) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.fitness = fitness;
				conf.algoConf.depth = 1;
				conf.algoConf.effort = 16;
				conf.algoConf.hweight = 1;
				conf.algoConf.simulationCount = 0;                 // offline版本需要控制simulationCount=0  
				conf.sysConf.knownBoxCount = conf.inst.t.length;   // knownBoxCount 只对 online 起作用
				expList.add(conf);
			}
		}
		runAll(expList, outdir);
	}
	
	
	
	public static class ExpConfig {
		public InstConfig instConf;
		public SysConfig sysConf;
		public AlgoConfig algoConf;
		
		public final InstData inst;
		public ExpConfig(InstData inst) {
			this.inst = inst;
		}
		
		public static String getHeader() {
			return InstConfig.getHeader() +","+ SysConfig.getHeader() + ","+AlgoConfig.getHeader();
		}
		public String toString() {
			return instConf.toString()+","+sysConf.toString()+","+algoConf.toString();
		}
		
		public ExpConfig deepCopy() {
			ExpConfig conf = new ExpConfig(this.inst);
			conf.instConf = instConf.clone();
			conf.sysConf = sysConf.clone();
			conf.algoConf = algoConf.clone();
			return conf;
		}
		
		public void print(PrintStream ps, String linePrefix) {
			ps.println(linePrefix+" instName: "+inst.name);
			ps.println(linePrefix+"instConf:");
			instConf.print(ps, linePrefix+"  ");
			ps.println(linePrefix+"sysConf:");
			sysConf.print(ps, linePrefix+"  ");
			ps.println(linePrefix+"algoConf:");
			algoConf.print(ps, linePrefix+"  ");
		}
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
	public static ArrayList<ExpConfig> genExpConfig(long instSeed, boolean shuffle, int instCount, long algoSeed) throws IOException {
				
		// Algorithm
		int[] depthList = new int[]{0,1,2};  // depth=0,直接用greedy的算法去求解，根本不用
//		int[] nodesKeptList = new int[]{5,10,15,20};
		int[] effortList = new int[] {4, 9, 16, 25};
		int[] simulationCountList = new int[]{1,2,4,8};  
		double[] hweightList = new double[]{0.2,0.5,1.0,2.0};

				
		// System configuration:
		// fixed		
		Type[] vaccumTypeList = {Type.TwoByThree}; // 机械手吸盘的长宽数量比
		double[] thetaList = {1.0};                // 机械手大小的缩放比例
		double[] maxDropHeightRatioList = {0.1};   // 一个盒子可以安全的自由下落盒子在H-轴上10%的长度
		
		double[] maxHGapForSupportList = {0.5};
		double[] minOverlapRatioForSupportList = {0.2};  // Zhou的alpha
						
		// changeable:
		int[] knownBoxCountList = new int[] {5,10,20,50};  	// known 系统知道未来的信息数量, N_k
		int[] openPalletCountList = new int[] {1,2,3};  	// pn 装载时同时打开的托盘数量, P
		int[] boxCountInRangeList = new int[] {1,2,3,4,5};  	//oper 机械手可以操作的个数, N_b

		
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

	
	public static void runAll(ArrayList<ExpConfig> confList, File outdir) throws IOException {
		outdir.mkdirs();

		File summaryFile = new File(outdir, "summary-"+outdir.getName()+".csv");   	
		boolean exists = summaryFile.exists();	
		PrintWriter pw = new PrintWriter(new FileWriter(summaryFile, true));
		if (!exists) {
			pw.println(ExecutionRecord.getHeader()+","+ExpConfig.getHeader()); pw.flush();
		}

		for (int i=0; i<confList.size(); i++) {
			System.out.println("run "+i+" ************");
			ExpConfig conf = confList.get(i);
			File runDir = new File(outdir,"run-"+i+"-"+conf.inst.name);
			ExecutionRecord exeRec = run(conf.inst, conf, runDir);
			if (exeRec != null) {
				pw.println(exeRec.toString()+","+conf.toString()); pw.flush();
			}
		}
		pw.close();
	}
	
	public static class ExecutionRecord {
		public String instName;
		
		public String algo;
		
		public int closedPalletCount;	// 关闭的pallet数量
		public int usedPalletCount;  	// 总共使用的pallet数量

		public double averageUtilClosed;    		// close的pallet的体积体用率
		public double averageUtil;                  // 所有用到的pallet的体积利用率
		
		public long time;
		public long timePerBox;
		
		public double averagePlacementsPerState;
		
		
		public static String getHeader() {
			return "instName,algo,closedPalletCount,usedPalletCount,averageUtilClosed,averageUtil,time(s),timePerBox(s),averagePlacementsPerState";
		}
		
		public String toString() {
			return instName+","+algo + ","+closedPalletCount+","+usedPalletCount+","+
					averageUtilClosed+","+averageUtil+","+time*0.001 + ","+ timePerBox*0.001+", "+averagePlacementsPerState;
		}
		
		public void print(PrintStream ps, String linePrefix) {
			ps.println(linePrefix+"instName: "+instName);
			ps.println(linePrefix+"algo: "+algo);
			ps.println(linePrefix+"closedPalletCount: "+closedPalletCount);
			ps.println(linePrefix+"usedPalletCount: "+usedPalletCount);
			ps.println(linePrefix+"averageUtilClosed: "+averageUtilClosed);
			ps.println(linePrefix+"averageUtil: "+averageUtil);
			ps.println(linePrefix+"time(s): "+time*0.001);
			ps.println(linePrefix+"averageUtil(s): "+timePerBox*0.001);
			ps.println(linePrefix+"averagePlacementsPerState: "+averagePlacementsPerState);
		}
	}

	/**
	 * 按照配置执行一个实验，如果已经执行过，返回 null
	 * @param inst
	 * @param conf
	 * @param dir
	 * @return
	 * @throws IOException
	 */
	public static ExecutionRecord run(InstData inst, ExpConfig conf, File dir) throws IOException {
		conf.print(System.out, "");

		File solFile = new File(dir, "loading-operation.txt");
		if (solFile.exists()) {
			System.out.println("skip: already executed ----------");
			return null;
		}
		
		ExecutionRecord exeRec = new ExecutionRecord();
		exeRec.instName = conf.inst.name;
		
		SysConfig sysConf = conf.sysConf;
		AlgoConfig algoConf = conf.algoConf;
		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
			
		State s = new State(sysInfo, algoConf);
		State.totalPlacementCount = 0;
		State.totalSteps = 0;
		
		// 计时
		long ti1 = System.currentTimeMillis();
		//LookAheadOnline la = new LookAheadOnline(algoConf, new Random(algoConf.seed)); // 调用lookAheadOnline进行求解
		Solver la = null;
		if (conf.algoConf.depth == 0) {
			exeRec.algo = "Greedy";
			System.out.println("Use algo: "+exeRec.algo+", ignore simuCount, knownBoxCount");
			la = new Greedy(conf.algoConf.fitness);
		} else if (conf.algoConf.simulationCount <= 0) {
			exeRec.algo = "LookAhead";
			System.out.println("Use algo: "+exeRec.algo+", ignore knownBoxCount");
			la = new LookAhead(algoConf);
		} else { // simulationCount >= 1
			exeRec.algo = "LookAheadOnline";
			System.out.println("Use algo: "+exeRec.algo);
			la = new LookAheadOnline(algoConf, new Random(algoConf.seed));
		}
		Solution sol = la.solve(s);  // 调用lookAheadOnline进行求解
		
		exeRec.closedPalletCount = sol.closed.length;
		exeRec.usedPalletCount = sol.closed.length + sol.open.length;
		
		exeRec.averageUtilClosed = sol.averageUtilClosed();
		exeRec.averageUtil = sol.averageUtil();
		
		exeRec.time = System.currentTimeMillis()-ti1;
		exeRec.timePerBox = exeRec.time/inst.getBoxCount();
		exeRec.averagePlacementsPerState = State.getAveragePlacementsPerState();
		
		
		if(!dir.exists()) {
			dir.mkdirs();
		}

		try {
			sol.validate();
			sol.draw(dir, false);
			sol.s.printLoadingInstructionsToFile(new File(dir, "loading-operation.txt"));
		} catch (RuntimeException e) {
			sol.draw(dir, true);
			sol.s.printLoadingInstructionsToFile(new File(dir, "error-loading-operation.txt"));
			throw e;
		}
		
		return exeRec; 
	}	
}



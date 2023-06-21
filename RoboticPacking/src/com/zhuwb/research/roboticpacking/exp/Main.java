package com.zhuwb.research.roboticpacking.exp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;
import java.util.TreeMap;
import java.util.TreeSet;

import com.zhuwb.research.roboticpacking.inst.BoxType;
import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;
import com.zhuwb.research.roboticpacking.inst.InstConfig;
import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.KnapsackUB;
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
import com.zhuwb.research.rpp3i.sclp.common.KnapsackFitnessCalc;

public class Main {

	public static void main(String[] args) throws IOException {
		long instSeed = 2;
		int algoSeed = 2;
		
//		GridPointBySegmentTree.debugDir = new File("result/GridPoint-debug");
		
//		expAlgoFitness(instSeed, algoSeed, new File("result/2021-05-27-fitness-offline-"+instSeed));		
//		expAlgoFitness2PushToEnd(instSeed, algoSeed, new File("result/2021-05-27-fitness2PushToEnd-offline-"+instSeed));		
//		expAlgoDepth(instSeed, algoSeed, new File("result/2021-06-11-depth-offline-"+instSeed));
//		expAlgoSearchEffort(instSeed, algoSeed, new File("result/2021-06-11-effort-offline-"+instSeed));
//		expAlgoSearchEffortSecRound(instSeed, algoSeed, new File("result/2021-06-11-effort2-offline-"+instSeed));
//		expAlgoHWeight(instSeed, algoSeed, new File("result/2021-06-11-hweight-offline-"+instSeed));
//		expAlgoGridSearch(instSeed, algoSeed, new File("result/2021-05-27-GridSearch-offline-"+instSeed));
		expOpenPalletCount(instSeed, algoSeed, new File("result/2021-06-22-openPalletCount-"+instSeed));

		System.out.println("gridCount: "+GridPointBySegmentTree.gridCount+
				"; segmentTreeCount: "+GridPointBySegmentTree.segmentTreeCount+
				"; bruteForceCout: "+GridPointBySegmentTree.bruteForceCount);
		
//		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
//		int depth = 1;
//		ArrayList<ExpConfig> expList = new ArrayList<>();
//		for (ExpConfig confTemplate: confList) {
//			ExpConfig conf = confTemplate.deepCopy();
//			conf.algoConf.depth = 1;
//			conf.algoConf.simulationCount = 2; // offline�汾��Ҫ����simulationCount=1  
//			conf.algoConf.effort = 9;
//			conf.sysConf.knownBoxCount = 10;   // offline�汾����֪���еĺ��ӵ���Ϣ
//			expList.add(conf);
//		}
//		runAll(expList, new File("result/test-sim"));
		
//		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
//		
//		ArrayList<ExpConfig> expList = new ArrayList<>();
//		for (ExpConfig confTemplate: confList) {
//			for (int depth : new int[] {0,1,2}) {
//				ExpConfig conf = confTemplate.deepCopy();
//				conf.algoConf.depth = depth;
//				conf.algoConf.simulationCount = 1; // offline�汾��Ҫ����simulationCount=1  
//				conf.sysConf.knownBoxCount = conf.inst.t.length;   // offline�汾����֪���еĺ��ӵ���Ϣ
//				expList.add(conf);
//			}
//		}
//		
//		ExpConfig conf = expList.get(187);
//		File runDir = new File("result/tmp-"+conf.inst.name);
//		ExecutionRecord exeRec = run(conf.inst, conf, runDir);
//		if (exeRec != null) {
//			System.out.println(exeRec.toString()+","+conf.toString()); System.out.flush();
//		}
	}

	
	// ���ʵ��������openPalletCount,�������㷨�������ѡ
	// ���� instCount = 2000������
	public static void expOpenPalletCount(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (int openPalletCount : new int[] {1,3,5}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = 16;
				conf.algoConf.hweight = 1;  						// 2; 2021-06-13 �õ� 2
				conf.algoConf.simulationCount = 0;                 // offline�汾��Ҫ����simulationCount=0  
				conf.sysConf.knownBoxCount = conf.inst.t.length;   // knownBoxCount ֻ�� online ������
				conf.sysConf.openPalletCount = openPalletCount;
				expList.add(conf);
			}
		}
		
		runAll(expList, outdir);
	}
	
	// ���ʵ�����ڲ���fitness,�������㷨�������ѡ
	public static void expAlgoFitness(long instSeed, long algoSeed, File outdir) throws IOException{		
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			for (Fitness fitness : Fitness.values()) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.fitness = fitness;
				conf.algoConf.simulationCount = 0;                 // offline�汾��Ҫ����simulationCount=0  
				conf.sysConf.knownBoxCount = conf.inst.t.length;   // knownBoxCount ֻ�� online ������
				expList.add(conf);
			}
		}
		runAll(expList, outdir);
	}
	
	// ���ʵ�����ڲ���fitness,�������㷨�������ѡ
	public static void expAlgoFitness2PushToEnd(long instSeed, long algoSeed, File outdir) throws IOException{		
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		int[] boxCountInRange = new int[] {1, 3, 5};
		int instCount = 0;
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.inst.getBoxCount() > 200) { continue; }
			
			for (int openPalletCount: new int[] {3,1,5}) {
				for (Fitness fitness : new Fitness[] {Fitness.Zhou, Fitness.ZhouNewType}) {  
					for (LeafEvaluator leafEvaulator : new LeafEvaluator[] {LeafEvaluator.Zhou, LeafEvaluator.UtilOfEmployed}) {  
						for (PalletFiteness palletFitness : PalletFiteness.values()) {
							ExpConfig conf = confTemplate.deepCopy();
							conf.algoConf.gridSearch = GridSearch.PushToEnd;
							conf.algoConf.palletFitness = palletFitness;
							conf.algoConf.fitness = fitness;
							conf.algoConf.leafEvaulator = leafEvaulator;
							conf.algoConf.depth = 1;
							conf.algoConf.effort = 16;
							conf.algoConf.simulationCount = 0;                 // offline�汾��Ҫ����simulationCount=0  
							conf.sysConf.knownBoxCount = conf.inst.t.length;   // knownBoxCount ֻ�� online ������
							conf.sysConf.openPalletCount = openPalletCount;
							conf.sysConf.boxCountInRange = boxCountInRange[instCount%boxCountInRange.length];
							expList.add(conf);
						}
					}
				}
			}
			
			instCount++;
		}
		runAll(expList, outdir);
	}

	
	// ���ʵ��������depth,�������㷨�������ѡ
	public static void expAlgoDepth(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (int depth : new int[] {0,1,2}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = depth;
				conf.algoConf.simulationCount = 0;                 // offline�汾��Ҫ����simulationCount=0  
				conf.sysConf.knownBoxCount = conf.inst.t.length;   // knownBoxCount ֻ�� online ������
				expList.add(conf);
			}
		}
		
		runAll(expList, outdir);
	}
	
	
	// ���ʵ��������hweight,�������㷨�������ѡ
	public static void expAlgoHWeight(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (double hweight : new double[] {0.2,0.5,1.0,2.0}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.hweight = hweight;
				conf.algoConf.simulationCount = 0; 				   // offline�汾��Ҫ����simulationCount=0  
				conf.sysConf.knownBoxCount = conf.inst.t.length;   // knownBoxCount ֻ�� online ������
				expList.add(conf);
			}
		}
		runAll(expList, outdir);
	}
	
	// ���ʵ��������effort,�������㷨�������ѡ
	public static void expAlgoSearchEffort(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (int effort : new int[] {4,9,16,25}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = effort;
				conf.algoConf.simulationCount = 0; 					// offline�汾��Ҫ����simulationCount=1  
				conf.sysConf.knownBoxCount = conf.inst.t.length;   	// knownBoxCount ֻ�� online ������
				expList.add(conf);
			}
		}
		
		runAll(expList, outdir);
	}
	
	
	// ���ʵ��������effort,�������㷨�������ѡ
	public static void expAlgoSearchEffortSecRound(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			for (int effort : new int[] {36,49}) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.effort = effort;
				conf.algoConf.simulationCount = 0; 					// offline�汾��Ҫ����simulationCount=0  
				conf.sysConf.knownBoxCount = conf.inst.t.length;    // knownBoxCount ֻ�� online ������
				expList.add(conf);
			}
		}
		
		runAll(expList, outdir);
	}

	
	
	
	// ���ʵ��������grid search,�������㷨�������ѡ
	public static void expAlgoGridSearch(long instSeed, long algoSeed, File outdir) throws IOException{
		ArrayList<ExpConfig> confList = genExpConfig(instSeed, true, -1, algoSeed);
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			for (GridSearch gridSearch : GridSearch.values()) {  
				ExpConfig conf = confTemplate.deepCopy();
				conf.algoConf.depth = 1; 
				conf.algoConf.gridSearch = gridSearch;
				conf.algoConf.simulationCount = 0; 					// offline�汾��Ҫ����simulationCount=1  
				conf.sysConf.knownBoxCount = conf.inst.t.length;   	// knownBoxCount ֻ�� online ������
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
	
	
	
	public static final long CONFIG_GEN_SEED = 3; // ���ɸ���Config���������
	public static final int selectedSize = 384;   
	
	/**
	 * �������ʵ������
	 * @param instSeed	�������ݼ�
	 * @param shuffle	�Ƿ���Ҳ������ӵ�˳��
	 * @param instCount	���ɵ�����������-1 ��ʾ���Բ��Լ��е�����
	 * @param algoSeed	�㷨�õ�����
	 * @return
	 * @throws IOException
	 */
	public static ArrayList<ExpConfig> genExpConfig(long instSeed, boolean shuffle, int instCount, long algoSeed) throws IOException {
				
		// Algorithm
		int[] depthList = new int[]{0,1,2};  // depth=0,ֱ����greedy���㷨ȥ��⣬��������
//		int[] nodesKeptList = new int[]{5,10,15,20};
		int[] effortList = new int[] {4, 9, 16, 25};
		int[] simulationCountList = new int[]{1,2,4,8};  
		double[] hweightList = new double[]{0.2,0.5,1.0,2.0};

				
		// System configuration:
		// fixed		
		Type[] vaccumTypeList = {Type.TwoByThree}; // ��е�����̵ĳ���������
		double[] thetaList = {1.0};                // ��е�ִ�С�����ű���
		double[] maxDropHeightRatioList = {0.1};   // һ�����ӿ��԰�ȫ���������������H-����10%�ĳ���
		
		double[] maxHGapForSupportList = {0.5};
		double[] minOverlapRatioForSupportList = {0.2};  // Zhou��alpha
						
		// changeable:
		int[] knownBoxCountList = new int[] {5,10,20,50};  	// known ϵͳ֪��δ������Ϣ����, N_k
		int[] openPalletCountList = new int[] {1,3,5};  	// pn װ��ʱͬʱ�򿪵���������, P
		int[] boxCountInRangeList = new int[] {1,3,5};  	//oper ��е�ֿ��Բ����ĸ���, N_b

		
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
	
	
	
	public static void upperBoundAll(ArrayList<ExpConfig> confList, File outdir) throws IOException {
		outdir.mkdirs();

		File summaryFile = new File(outdir, "ub-summary-"+outdir.getName()+".csv");   	
		boolean exists = summaryFile.exists();	
		PrintWriter pw = new PrintWriter(new FileWriter(summaryFile, true));
		if (!exists) {
			pw.println("instName,util_ub"); pw.flush();
		}

		for (int i=0; i<confList.size(); i++) {
			ExpConfig conf = confList.get(i);
			SystemInfo sysInfo = new SystemInfo(conf.inst, conf.sysConf);
			double volumeUB = KnapsackUB.upperBound(sysInfo);
			pw.println(conf.inst.name+","+volumeUB); pw.flush();
		}
		pw.close();
	}
	
	public static class ExecutionRecord {
		public String instName;
		
		public String algo;
		
		public int closedPalletCount;	// �رյ�pallet����
		public int usedPalletCount;  	// �ܹ�ʹ�õ�pallet����

		public double averageUtilClosed;    		// close��pallet�����������
		public double averageUtil;                  // �����õ���pallet�����������
		
		public int maxDropH;			// max {dropH of loaded boxes}
		public double maxDropHRatio;	// max {dropH/dH of loaded boxes}
		
		public long time;
		public long timePerBox;
		
		public double averagePlacementsPerState;
		
		
		public static String getHeader() {
			return "instName,algo,closedPalletCount,usedPalletCount,averageUtilClosed,averageUtil,max{dropH},max{dropH/dH},time(s),timePerBox(s),averagePlacementsPerState";
		}
		
		public String toString() {
			return instName+","+algo + ","+closedPalletCount+","+usedPalletCount+","+
					averageUtilClosed+","+averageUtil+","+maxDropH+","+maxDropHRatio+","
					+time*0.001 + ","+ timePerBox*0.001+", "+averagePlacementsPerState;
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
	 * ��������ִ��һ��ʵ�飬����Ѿ�ִ�й������� null
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
		
		// ��ʱ
		long ti1 = System.currentTimeMillis();
		//LookAheadOnline la = new LookAheadOnline(algoConf, new Random(algoConf.seed)); // ����lookAheadOnline�������
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
		Solution sol = la.solve(s);  // ����lookAheadOnline�������
		
		exeRec.closedPalletCount = sol.closed.length;
		exeRec.usedPalletCount = sol.closed.length + sol.open.length;
		
		exeRec.averageUtilClosed = sol.averageUtilClosed();
		exeRec.averageUtil = sol.averageUtil();
		
		exeRec.maxDropH = sol.maxDropH();
		exeRec.maxDropHRatio = sol.maxDropHRatio();
		
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



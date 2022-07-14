package com.zhuwb.research.roboticpacking.search;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Random;

import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.SystemInfo;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.search.Greedy.BestNext;
import com.zhuwb.research.roboticpacking.search.State.Operation;

public class LookAheadOnline implements Solver {
	private final AlgoConfig conf;
	private final Greedy g;
	private final LookAhead la;

	private final Random rand;   // 用来生成仿真序列的随机种子
	
	public LookAheadOnline(AlgoConfig conf, Random rand) {
		if (conf.depth == 0) {
			throw new IllegalArgumentException("conf.depth = 0, please use Greedy");
		}
		if (conf.simulationCount <= 0) {
			throw new IllegalArgumentException("conf.simulationCount: "+conf.simulationCount+" must be positive.");
		}
		this.la = new LookAhead(conf);
		this.g = la.g;
		this.conf = conf;
		this.rand = rand;
	}

	static class SimulatedSequence {
		int[] arrivalSeq;
		int start;
		int end;
		
		public SimulatedSequence(int[] seq) {
			this.arrivalSeq = seq;
			this.start = 0;
			this.end = seq.length;
		}
		public SimulatedSequence(int[] t, int start, int end) {
			this.arrivalSeq = t;
			this.start = start;
			this.end = end;
		}
	}
	
	// 生成仿真序列
	protected SimulatedSequence[] generateSimulatedSequences(State s){
	
		int nextUnknowBoxIdx = -1;
		if (s.nextBoxIdx<s.boxCount) {
			nextUnknowBoxIdx = s.nextBoxIdx - s.sysInfo.conf.boxCountInRange + s.sysInfo.conf.knownBoxCount;
			if (nextUnknowBoxIdx<s.boxCount) {
				// 有可能需要仿真
				int[] unknowBoxCount = s.cloneRemainingBoxCount();  // 按照盒子种类统计的未知信息盒子的个数
				for (int t=0; t<s.boxCountInRange.length;t++) {
					unknowBoxCount[t]-=s.boxCountInRange[t];
				}
				
				double loadedVolumn = 0;
				for (int i=s.nextBoxIdx;i<s.boxCount && i< nextUnknowBoxIdx;i++) {
					int boxType = s.sysInfo.inst.t[i];
					unknowBoxCount[boxType]--;
					loadedVolumn += s.sysInfo.boxTypes[boxType].getVolume();
				}
				for (Pallet p: s.open) {
					loadedVolumn += p.loadedVolume;
				}
				
				int maxSimBoxCount = s.boxCount - nextUnknowBoxIdx;
				double maxVolumn = s.open.length * s.sysInfo.inst.getPalletVolume() - loadedVolumn;
				
				SimulatedSequence[] result = new SimulatedSequence[conf.simulationCount];
				for (int seqIdx=0;seqIdx<result.length;seqIdx++) {
					result[seqIdx] = generateOneSimulatedSequence(nextUnknowBoxIdx, unknowBoxCount, 
							maxSimBoxCount, maxVolumn, s);					
				}
				return result;
			}
		}
		return new SimulatedSequence[]{ new SimulatedSequence(s.sysInfo.inst.t,s.nextBoxIdx,s.boxCount)};
	}
	
	private SimulatedSequence generateOneSimulatedSequence(int nextUnknowBoxIdx, final int[] unknowBoxCount, 
							int maxSimBoxCount,double maxVolumn, State s) {
		int[] boxCount = Arrays.copyOf(unknowBoxCount, unknowBoxCount.length);
		double volume = 0;
		ArrayList<Integer> genBoxTypes = new ArrayList<Integer>();
		while (genBoxTypes.size()<maxSimBoxCount && volume < maxVolumn) {
			int boxType = randomPick(boxCount);
			boxCount[boxType]-=1;
			genBoxTypes.add(Integer.valueOf(boxType));
			volume += s.sysInfo.boxTypes[boxType].getVolume();
		}
		
		int[] result = new int[genBoxTypes.size() + nextUnknowBoxIdx - s.nextBoxIdx];
		int j=0;
		for(int i=s.nextBoxIdx;i<nextUnknowBoxIdx;i++) {
			result[j++] = s.sysInfo.inst.t[i];
		}
		for (int i=0;i<genBoxTypes.size();i++) {
			result[j++] = genBoxTypes.get(i).intValue();
		}
		
		return new SimulatedSequence(result);
	}
	
	
	// 按照每种盒子的频率，随机选一个boxType
	public int randomPick(int[] boxCount) {
		int[] accBoxCount = new int[boxCount.length];
		accBoxCount[0]=boxCount[0];
		for (int i=1;i<boxCount.length;i++) {
			accBoxCount[i]=accBoxCount[i-1] + boxCount[i];
		}
		int total = accBoxCount[accBoxCount.length-1];  // 盒子总数
		int j = rand.nextInt(total);  //生成上界为total的数字
		for (int i=0; i<accBoxCount.length;i++) {
			if (accBoxCount[i]>=j) {
				return i;
			}
		}
		throw new RuntimeException();
	}
	
//	private BestNext searchBestOpForSimSeq(State s, SimulatedSequence seq, int d) {
//		State simS = s.switchSequence(seq.arrivalSeq, seq.start, seq.end);
//		BestNext bn = null;
//		if (d>0) {
//			bn = la.lookAhead(simS, d);
//		} else {
//			bn = g.greedy(simS);
//			bn.score = la.evaluate(simS);
//		}
//		
//		return bn;
//	}

	// 为当前 state 选最优 placement 时
	// 计算每个场景下，每个 placement 得到得分数
	// 然后选一个在所有场景下得分最高的 placement
	//
	// voting:     每个场景下，最好的 placement 得 1 分，其它 placment 的 0 分
	// totalVoume: 每个场景下，一个 placement 的分数，按照它放置以后，用 LookAhead 找到的最好的方案中，Open Pallet中总盒子体积
	// totalVoume: 每个场景下，一个 placement 的分数，按照它放置以后，用 LookAhead 找到的最好的方案中，Open Pallet中盒子体积最大的 pallet 的体积
	public static enum SelectMode {
		voting, totalVolume, maxVolume		   
	}

	public BestNext onlineLAByVoting(State s, int d) {
		SimulatedSequence[] seqList = generateSimulatedSequences(s);
		if (seqList.length==1) {
			SimulatedSequence seq = seqList[0];
			State simS = s.switchSequence(seq.arrivalSeq, seq.start, seq.end);
			BestNext bn = la.lookAhead(simS, d);
			if (bn.nextState == null) {
				bn.endState = s;
				bn.score = null;
				return bn;
			}
			
			Operation bestOp = bn.nextState.oper;
			State child = s.place(bestOp.opIdx, bestOp.placement); // 装完后的状态
			return new BestNext(child,null,null);
		}
		
		ArrayList<PlacementWitFitness> allPlacementWithFitness =  s.findAll(g.fitness);
		HashMap<Operation, Double> op2score = new HashMap<>();

		// 找到当前状态最好的 nodesKept 个 placement
		int count = Math.min(conf.getNodesKepth(), allPlacementWithFitness.size());
		ArrayList<Operation> opList = new ArrayList<>();
		for (int i=allPlacementWithFitness.size()-1; i>=allPlacementWithFitness.size()-count; i--) {
			PlacementWitFitness pf = allPlacementWithFitness.get(i);
			Operation op = new Operation(pf.opIdx,pf.placement);
			opList.add(op);
			op2score.put(op, Double.valueOf(0.0));
		}
		
		// 当前状态无法摆放新的盒子
		if (count==0) {
			return new BestNext(null, s, null);
		}

		// 尝试每个 simulation 场景
		for(SimulatedSequence seq:seqList) {
			State simS = s.switchSequence(seq.arrivalSeq, seq.start, seq.end);
			
			if (conf.selectMode == SelectMode.voting) {
				//System.out.println("by voting");
				BestNext bn = la.lookAhead(simS, d);
				Operation op = bn.nextState.oper;
				double scoreSum = op2score.get(op).doubleValue();
				op2score.put(op, Double.valueOf(scoreSum+1));
				continue;
			}
			
			// 计算每个可行 placement 在一个simulation场景下的得分
			for (Operation op:opList) {
				State child = simS.place(op.opIdx, op.placement); // 装完后的状态
				BestNext bn = la.lookAhead(child, d-1);
				
				double simScore;
				switch (conf.selectMode) {
				case totalVolume: simScore = bn.score[0]; break;
				case maxVolume: simScore = bn.score[1]; break;
				default: throw new RuntimeException();
				}
				
				// 把 placement op 在一个场景下的得分累积起来
				double scoreSum = op2score.get(op).doubleValue();
				op2score.put(op, Double.valueOf(scoreSum+simScore));
			}
		}
			
		// 找所有场景累积分数最高的 placement
		Operation bestOp = null;
		double bestScore = Double.NEGATIVE_INFINITY;
		for (Entry<Operation, Double> e:op2score.entrySet()) {
			Operation op = e.getKey();
			double score = e.getValue().doubleValue();
			if (bestScore<score) {
				bestOp = op;
				bestScore = score;
			}
		}
		
		State child = s.place(bestOp.opIdx, bestOp.placement); // 装完后的状态
		return new BestNext(child,null,null);
	}

	/**
	 * Greedy d-step look ahead
	 */
	@Override
	public Solution solve(State state) {
		if (state.sysInfo.conf.knownBoxCount < state.sysInfo.conf.boxCountInRange) {
			throw new IllegalArgumentException("knownBoxCount: "+state.sysInfo.conf.knownBoxCount
					+" < boxCountInRange: "+state.sysInfo.conf.boxCountInRange);
		}
		
		State curS = state;
		int step = 0;
		while (step < state.sysInfo.inst.getBoxCount()) {
			System.out.print(".");
			if (step % 100 == 99) {
				System.out.println("step: "+step);
			}
			curS = curS.close();
			
			BestNext bn = onlineLAByVoting(curS, conf.depth);
			if (bn.nextState != null) {
				curS = bn.nextState;
				step += 1;
			} else {
				Solution sol = new Solution(curS);
				File outdir = new File(debugDir, "step-"+step);
				try {
					sol.draw(outdir, true);
				} catch (IOException e) {
					e.printStackTrace();
				}
				throw new RuntimeException("step: "+step+" bn.nextState == null, see more info in "+outdir);
			}
		}
		return new Solution(curS);
	}
	
	public boolean debug = false;	
	public File debugDir = new File("result/LookAheadOnline-debug");
	

	public static void main(String[] args) throws IOException {
		File dir = new File("result/OnlineGLA-21May22");
		
		InstData inst = InstanceLoader.load(2,"SD0-2-200-large-2");
		
		System.out.println(inst.name);
		SysConfig sysConf = new SysConfig();
		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
		System.out.println("box types:");
		System.out.println(Arrays.toString(sysInfo.boxTypes));
		
		
		AlgoConfig algoConf = new AlgoConfig();
		algoConf.depth = 1;
		algoConf.gridSearch = GridSearch.PushToEnd;
		
		Random rand = new Random(algoConf.seed);
		LookAheadOnline la = new LookAheadOnline(algoConf, rand);
		State s = new State(sysInfo, algoConf);
		Solution sol = la.solve(s);
		sol.draw(dir, true);
		sol.s.printLoadingInstructionsToFile(new File(dir, "loading-operation.txt"));
	}	
	
}

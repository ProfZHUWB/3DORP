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

	private final Random rand;   // �������ɷ������е��������
	
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
	
	// ���ɷ�������
	protected SimulatedSequence[] generateSimulatedSequences(State s){
	
		int nextUnknowBoxIdx = -1;
		if (s.nextBoxIdx<s.boxCount) {
			nextUnknowBoxIdx = s.nextBoxIdx - s.sysInfo.conf.boxCountInRange + s.sysInfo.conf.knownBoxCount;
			if (nextUnknowBoxIdx<s.boxCount) {
				// �п�����Ҫ����
				int[] unknowBoxCount = s.cloneRemainingBoxCount();  // ���պ�������ͳ�Ƶ�δ֪��Ϣ���ӵĸ���
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
	
	
	// ����ÿ�ֺ��ӵ�Ƶ�ʣ����ѡһ��boxType
	public int randomPick(int[] boxCount) {
		int[] accBoxCount = new int[boxCount.length];
		accBoxCount[0]=boxCount[0];
		for (int i=1;i<boxCount.length;i++) {
			accBoxCount[i]=accBoxCount[i-1] + boxCount[i];
		}
		int total = accBoxCount[accBoxCount.length-1];  // ��������
		int j = rand.nextInt(total);  //�����Ͻ�Ϊtotal������
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

	// Ϊ��ǰ state ѡ���� placement ʱ
	// ����ÿ�������£�ÿ�� placement �õ��÷���
	// Ȼ��ѡһ�������г����µ÷���ߵ� placement
	//
	// voting:     ÿ�������£���õ� placement �� 1 �֣����� placment �� 0 ��
	// totalVoume: ÿ�������£�һ�� placement �ķ����������������Ժ��� LookAhead �ҵ�����õķ����У�Open Pallet���ܺ������
	// totalVoume: ÿ�������£�һ�� placement �ķ����������������Ժ��� LookAhead �ҵ�����õķ����У�Open Pallet�к���������� pallet �����
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
			State child = s.place(bestOp.opIdx, bestOp.placement); // װ����״̬
			return new BestNext(child,null,null);
		}
		
		ArrayList<PlacementWitFitness> allPlacementWithFitness =  s.findAll(g.fitness);
		HashMap<Operation, Double> op2score = new HashMap<>();

		// �ҵ���ǰ״̬��õ� nodesKept �� placement
		int count = Math.min(conf.getNodesKepth(), allPlacementWithFitness.size());
		ArrayList<Operation> opList = new ArrayList<>();
		for (int i=allPlacementWithFitness.size()-1; i>=allPlacementWithFitness.size()-count; i--) {
			PlacementWitFitness pf = allPlacementWithFitness.get(i);
			Operation op = new Operation(pf.opIdx,pf.placement);
			opList.add(op);
			op2score.put(op, Double.valueOf(0.0));
		}
		
		// ��ǰ״̬�޷��ڷ��µĺ���
		if (count==0) {
			return new BestNext(null, s, null);
		}

		// ����ÿ�� simulation ����
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
			
			// ����ÿ������ placement ��һ��simulation�����µĵ÷�
			for (Operation op:opList) {
				State child = simS.place(op.opIdx, op.placement); // װ����״̬
				BestNext bn = la.lookAhead(child, d-1);
				
				double simScore;
				switch (conf.selectMode) {
				case totalVolume: simScore = bn.score[0]; break;
				case maxVolume: simScore = bn.score[1]; break;
				default: throw new RuntimeException();
				}
				
				// �� placement op ��һ�������µĵ÷��ۻ�����
				double scoreSum = op2score.get(op).doubleValue();
				op2score.put(op, Double.valueOf(scoreSum+simScore));
			}
		}
			
		// �����г����ۻ�������ߵ� placement
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
		
		State child = s.place(bestOp.opIdx, bestOp.placement); // װ����״̬
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

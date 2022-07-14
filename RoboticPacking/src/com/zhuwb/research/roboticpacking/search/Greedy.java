package com.zhuwb.research.roboticpacking.search;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.SystemInfo;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.search.State.PalletFiteness;


public class Greedy implements Solver {
	public final Fitness fitness; // ̰������׼��


	public Greedy(Fitness fitness) {
		super();
		this.fitness = fitness;
	}

	
	public static class BestNext implements Comparable<BestNext>{	
		State nextState;
		State endState;
		double[] score;
		
		public BestNext(State nextState, State endState, double[] score) {
			super();
			this.nextState = nextState;
			this.endState = endState;
			this.score = score;
		}

		@Override
		public int compareTo(BestNext o) {
			for(int i=0;i<this.score.length;i++) {
				if (score[i]<o.score[i]) {return -1;}
				if (score[i]>o.score[i]) {return 1;}
			}
			return 0;
		}
	}

	/**
	 * ����ΪLookaheadOnline���ӳ������һ��simulationʱ��Ϊ������ٶȣ����������µ�pallet�滻�Ѿ����ģ���ʱӦ������ allowOpen = false.
	 * ����Ϊ���߰汾��Solver��Ӧ������ allowOpen = true
	 * @param allowOpen
	 */
	public BestNext greedy(State state, boolean allowOpen) {
		BestNext bn = new BestNext(null, null, null);

		State curS = state;
		int step = 0;
		while(true) {
//			System.out.println("step  "+step+"  state: "+curS);
//			System.out.println("  -- closed: "+curS.closed);
			
			if (step==1) {
				bn.nextState = curS;
			}
			
			// 1. ���Թر����������޷�װ��Χ�ں��ӵ�pallet
			if (allowOpen) { 
				curS = curS.close();
//				System.out.println("step "+step+" state: "+curS);
			}
			
			// 2. ������placements����¼���ĸ�pallet
			PlacementWitFitness bestPlacement = curS.findBestPlacement(fitness);
		
			// װ���ŵ�pallet�µ�����placement
			if (bestPlacement != null) {
//				System.out.println("step: "+step+" bestOpIdx: "+bestOpIdx+" best placement: "+bestPlacement);
//				Pallet newP = curS.open[bestOpIdx];
//				newP.drawPlacements(new File(dir, "P"+step));			
	
				curS = curS.place(bestPlacement.opIdx, bestPlacement.placement); // װ����״̬
				step+=1;
				
//				Pallet newP = curS.open[bestOpIdx];				
//				newP.draw(new File(dir, "P" + step+".nb"), null);
			} else { // �Ѿ��޷����Է�
				break;
			}
		}  // while true װ�Ĺ���
		bn.endState = curS; // ÿһ��װ������nextState
		return bn;
	}
	
	
	
	// arrivalSeq[start...end)����ʾ��е�ַ�Χ֮��İ�˳�򵽴�ĺ��� 
	@Override
	public Solution solve(State state) {
		BestNext bn = greedy(state, true);
		return new Solution(bn.endState);
	}


	public static void main(String[] args) throws IOException {
		File dir = new File("result/Greedy-21May22");
		
		InstData inst = InstanceLoader.load(2,"SD0-2-1000-large-2");
		
		System.out.println(inst.name);
		SysConfig sysConf = new SysConfig();
		sysConf.openPalletCount = 2;
		sysConf.boxCountInRange = 1;
		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
		System.out.println("box types:");
		System.out.println(Arrays.toString(sysInfo.boxTypes));
		
		AlgoConfig algoConf = new AlgoConfig();
		algoConf.gridSearch = GridSearch.UseGrid;
		algoConf.palletFitness = PalletFiteness.Dummpy;
		State s = new State(sysInfo, algoConf);

		Greedy g = new Greedy(Fitness.Regularity);
		Solution sol = g.solve(s);
		sol.draw(dir, true);
		
		sol.s.printLoadingInstructionsToFile(new File(dir, "loading.txt"));
	}
}


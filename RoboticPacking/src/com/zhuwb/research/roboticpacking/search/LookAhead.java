package com.zhuwb.research.roboticpacking.search;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.SystemInfo;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.search.Greedy.BestNext;

public class LookAhead implements Solver {
	protected Greedy g;
	private final AlgoConfig conf;
	
	public static enum LeafEvaluator {
		Zhou {
			@Override
			public double[] evaluate(State state) {
				double totalVolume = 0; 
				double maxVolume = 0;
				for (int opIdx=0; opIdx<state.open.length; opIdx++) {
					Pallet P = state.open[opIdx];
					totalVolume	+= P.loadedVolume;
					if (P.loadedVolume>maxVolume) {
						maxVolume = P.loadedVolume;
					}
				}
				return new double[] {
						totalVolume,
						maxVolume
				};
			}
		},
		UtilOfEmployed {
			/**
			 * Evaluator fitness of leaf node by consider the total volume utilization of all employed open pallet,
			 * then by the volume of the loaded boxes in the most fully loaded open pallet.
			 * @author iwenc
			 */
			@Override
			public double[] evaluate(State state) {
				double totalVolume = 0; 
				double maxVolume = 0;
				int nonEmptyCount = 0;
				for (int opIdx=0; opIdx<state.open.length; opIdx++) {
					Pallet P = state.open[opIdx];
					totalVolume	+= P.loadedVolume;
					if (P.loadedVolume>maxVolume) {
						maxVolume = P.loadedVolume;
					}
					if (P.loadedCount > 0) {
						nonEmptyCount ++;
					}
				}
				double totalUtil = 0;
				if (nonEmptyCount > 0) {
					totalUtil = totalVolume/nonEmptyCount;
				}
				
				return new double[] {
						totalUtil,
						maxVolume
				};			}
		};
		
		public abstract double[] evaluate(State state);
	}
	
	/**
	 * 当把 Lookahead 作为 LookaheadOnline 的子程序，计算一个simulation的结果时，为了加速，simulation的过程中
	 * 不会关闭pallet并打开新pallet，这时allowOpen = false.
	 * 当单独使用 Lookahead 求解离线版本时，应该设置 allowOpen = true.
	 * @param conf
	 * @param allowOpen
	 */
	public LookAhead(AlgoConfig conf) {
		this.g = new Greedy(conf.fitness);
		this.conf = conf;
	}
	

//	
//	// 装完后的评价
//	protected double[] evaluate(State state) {
//		double totalVolume = 0; 
//		double maxVolume = 0;
//		for (int opIdx=0; opIdx<state.open.length; opIdx++) {
//			Pallet P = state.open[opIdx];
//			totalVolume	+= P.loadedVolume;
//			if (P.loadedVolume>maxVolume) {
//				maxVolume = P.loadedVolume;
//			}
//		}
//		return new double[] {
//				totalVolume,
//				maxVolume
//		};
//	}

	private String getPrefix(int d) {
		StringBuffer sb = new StringBuffer();
		for (int i=d; i<=this.conf.depth; i++) {
			sb.append("  ");
		}
		return sb.toString();
	}
	
	

	public BestNext lookAhead(State state, int d) {
		if (d==0) {
			BestNext bn = g.greedy(state, false);
			bn.score = conf.leafEvaulator.evaluate(bn.endState);
			
			if (debug) {System.out.println(getPrefix(d)+"  greedy score: "+Arrays.toString(bn.score)); }
			return bn;
		}

		if (debug) { System.out.println(getPrefix(d)+"depth: "+d+" state: "+state); }
//		if (allowOpen) {
//			state = state.close();
//		}
		
		BestNext best = new BestNext(null,state,null);
		best.score = new double[] {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
		ArrayList<PlacementWitFitness> allPlacementWithFitness =  state.findAll(g.fitness);
		
		if (debug) {
			System.out.println(getPrefix(d)+"  placement count: "+allPlacementWithFitness.size());
			System.out.println(getPrefix(d)+"  nodesKept: "+conf.getNodesKepth());
		}
				
		int count = Math.min(conf.getNodesKepth(), allPlacementWithFitness.size());
		if (count == 0) {
			best.score = conf.leafEvaulator.evaluate(best.endState);
			return best;
		}
		for (int i=allPlacementWithFitness.size()-1; i>=allPlacementWithFitness.size()-count; i--) {
			PlacementWitFitness pf = allPlacementWithFitness.get(i);
			
			if (debug) {
				System.out.println(getPrefix(d)+"  -- placement["+i+"]: "+pf);
			}
			
			State child = state.place(pf.opIdx, pf.placement); // 装完后的状态
			BestNext bn = lookAhead(child, d-1);
			
			if (debug) {
				System.out.println(getPrefix(d)+"  -- bn.nextState: "+bn.nextState);
				System.out.println(getPrefix(d)+"  -- bn.score: "+Arrays.toString(bn.score));
			}

			if (best.compareTo(bn)<0) {
				best.score = bn.score;
				best.endState = bn.endState;
				best.nextState = child;
			}
		}
		
		if (best.nextState == null) {
			System.out.println(getPrefix(d)+"  placement count: "+allPlacementWithFitness.size());
			System.out.println(getPrefix(d)+"  nodesKept: "+conf.getNodesKepth());
			throw new RuntimeException();
		}

		if (debug)	{ System.out.println(getPrefix(d)+"--> best score: "+Arrays.toString(best.score)); }
		
		return best;
	}
	
	@Override
	public Solution solve(State state) {
		State curS = state;
		int step = 0;
		while (step < state.sysInfo.inst.t.length) {
			System.out.print(".");
			if (step % 100 == 99) {
				System.out.println("step: "+step);
			}
			curS = curS.close();
			
//			if (step >= 186) {
//				System.out.println(curS);
//				try {
//					Solution sol = new Solution(curS);
//					sol.draw(new File("result/test-"+step), true);
//				} catch (IOException e) {
//					throw new RuntimeException(e);
//				}
//			}
//			if (step >= 186) {
//				debug = true;
//				System.out.println();
//			}
			
			BestNext bn = lookAhead(curS, conf.depth);
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
	public File debugDir = new File("result/LookAhead-debug");
	
	
	public static void main(String[] args) throws IOException {
//		File dir = new File("result/GLA-21May23");
//		
//		InstData inst = InstanceLoader.load(2,"SD0-2-1000-medium");
//		
//		System.out.println(inst.name);
//		SysConfig sysConf = new SysConfig();
//		sysConf.openPalletCount = 1;
//		sysConf.boxCountInRange = 1;
//		
//		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
//		System.out.println("box types:");
//		System.out.println(Arrays.toString(sysInfo.boxTypes));
//		
//		State s = new State(sysInfo, GridSearch.UseGrid);
//		
//		AlgoConfig algoConf = new AlgoConfig();
//		
//		algoConf.seed = 2;    // 算法需要的随机种子
//		algoConf.fitness = FitnessFunc.Regularity;
//		
//		algoConf.depth = 2;
//		algoConf.hweight=0.5;
//		algoConf.effort = 25;
//		algoConf.simulationCount =1;
//		
//		LookAhead la = new LookAhead(algoConf);
//		Solution sol = la.solve(s);
//		sol.draw(dir);
//		sol.s.printToFile(new File(dir, "loading-operation.txt"));
		
		File dir = new File("result/GLA-21May27");
		
		InstData inst = InstanceLoader.load(2,"SD0-3-2000-small");
		
		System.out.println(inst.name);
		SysConfig sysConf = new SysConfig();
		sysConf.openPalletCount = 1;
		sysConf.boxCountInRange = 1;
		sysConf.knownBoxCount = 2000;
		
		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
		System.out.println("box types:");
		System.out.println(Arrays.toString(sysInfo.boxTypes));
		
		AlgoConfig algoConf = new AlgoConfig();
		
		algoConf.seed = 2;    // 算法需要的随机种子
		algoConf.fitness = Fitness.Regularity;
		
		algoConf.depth = 2;
		algoConf.hweight=0.5;
		algoConf.effort = 4;
		algoConf.simulationCount =1;
		algoConf.gridSearch = GridSearch.PushToEnd;
		algoConf.fitness = Fitness.Zhou;
				
		LookAhead la = new LookAhead(algoConf);
		State s = new State(sysInfo, algoConf);
		Solution sol = la.solve(s);
		sol.draw(dir, true);
		sol.s.printLoadingInstructionsToFile(new File(dir, "loading-operation.txt"));
	}
}

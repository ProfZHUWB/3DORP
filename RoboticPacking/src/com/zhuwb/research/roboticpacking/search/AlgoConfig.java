package com.zhuwb.research.roboticpacking.search;

import java.io.PrintStream;

import com.zhuwb.research.roboticpacking.search.LookAhead.LeafEvaluator;
import com.zhuwb.research.roboticpacking.search.LookAheadOnline.SelectMode;
import com.zhuwb.research.roboticpacking.search.State.PalletFiteness;

public class AlgoConfig implements Cloneable {
	public long seed = 2;    // 算法需要的随机种子

	public PalletFiteness palletFitness = PalletFiteness.UseEmptyIfNecessary; // Dummy: 无视托盘差异
	
	public Fitness fitness = Fitness.Zhou; //Regularity;
	
	
	// 衡量时考虑用到的托盘的利用率，UtilOfEmployed 
	public LeafEvaluator leafEvaulator = LeafEvaluator.UtilOfEmployed; // Zhou: open pallet 装的总盒子体检

	public int depth = 1;      // 0 使用 Greedy
							   // depth >= 1: 
	                           //    simulationCount = 0: 则用 LookAhead 向前看 depth 步
	                           //    simulationCount >= 1: 则为 LookAheadOnline 为每个simulated序列调用 LookAhead 时向前看的步数
	public int effort = 16;	   // effort = nodesKepth^depth; LookAhead 中扩展出来的深度为 depth 的节点数
	//private int nodesKept = 16; // number 每层保留最优子节点个数

	/**
	 * {@link LookAheadOnline} 算法评估当前最优 {@link com.zhuwb.research.roboticpacking.inst.Placement} 时
	 * 进行仿真的次数。
	 * <ul>0: 表示不仿真，即使用 offline 的算法用 {@link Greedy} 或者 {@link LookAhead} </ul>
	 * <ul>>= 1: 表示用 {@link LookAheadOnline} 算法 </ul> 
	 */
	public int simulationCount = 5;
	
	// 为当前 state 选最优 placement 时
	// 计算每个场景下，每个 placement 得到得分数
	// 然后选一个在所有场景下得分最高的 placement
	//
	// voting:     每个场景下，最好的 placement 得 1 分，其它 placment 的 0 分; Zhou
	// totalVoume: 每个场景下，一个 placement 的分数，按照它放置以后，用 LookAhead 找到的最好的方案中，Open Pallet中总盒子体积
	// totalVoume: 每个场景下，一个 placement 的分数，按照它放置以后，用 LookAhead 找到的最好的方案中，Open Pallet中盒子体积最大的 pallet 的体积
	public SelectMode selectMode = SelectMode.voting;
	
	public double hweight = 1.0;     // zhou中的zweight,H-面规整度权重
	
    // PushToEnd：  （周游）L-,W-push会把盒子推到空间最靠近原点的面
	// Grid:  L-push 会到尝试每个网格线（即新盒子l1或l2与已有盒子l2或者0一样的位置）
	//        W-push 会到尝试每个网格线（即新盒子w1或w2与已有盒子w2或者0一样的位置）
	// GridOnFloor:  仅当 space 的底面高度为 0 时尝试 GridSearch
	public GridSearch gridSearch = GridSearch.PushToEnd;
	public static enum GridSearch {
		PushToEnd, GridOnFloor, UseGrid
	}

	
	public int getNodesKepth() {
		if (depth == 0) {
			return 1;
		} else {
			return (int) Math.round(Math.pow(effort, 1.0/depth));
		}
	}
	
	public static String getHeader() {
		return "seed,depth,nodesKept,simulationCount,selectMode,hweight,palletFitness,fitness,leafEvaulator,gridSearch,effort";
	}
	
	public String toString() {
		return seed+","+depth+","+getNodesKepth()+","+simulationCount+","+selectMode+","+hweight+","+palletFitness+","+fitness+","+leafEvaulator+","+gridSearch+","+effort;
	}
	
	public void print(PrintStream ps, String linePrefix) {
		ps.println(linePrefix+"seed: "+seed);
		ps.println(linePrefix+"depth: "+depth);
		ps.println(linePrefix+"nodesKept: "+getNodesKepth());
		ps.println(linePrefix+"simulationCount: "+simulationCount);
		ps.println(linePrefix+"selectMode: "+selectMode);
		ps.println(linePrefix+"hweight: "+hweight);
		ps.println(linePrefix+"palletFitness: "+palletFitness);
		ps.println(linePrefix+"fitness: "+fitness);
		ps.println(linePrefix+"leafEvaulator: "+leafEvaulator);
		ps.println(linePrefix+"gridSearch: "+gridSearch);
		ps.println(linePrefix+"effort: "+effort);
	}
	
	public static AlgoConfig defaultConf = new AlgoConfig();

	@Override
	public AlgoConfig clone() {
		try {
			return (AlgoConfig) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
	}
}
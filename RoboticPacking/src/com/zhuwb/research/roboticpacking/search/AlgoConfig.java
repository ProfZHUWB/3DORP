package com.zhuwb.research.roboticpacking.search;

import java.io.PrintStream;

import com.zhuwb.research.roboticpacking.search.LookAhead.LeafEvaluator;
import com.zhuwb.research.roboticpacking.search.LookAheadOnline.SelectMode;
import com.zhuwb.research.roboticpacking.search.State.PalletFiteness;

public class AlgoConfig implements Cloneable {
	public long seed = 2;    // �㷨��Ҫ���������

	public PalletFiteness palletFitness = PalletFiteness.UseEmptyIfNecessary; // Dummy: �������̲���
	
	public Fitness fitness = Fitness.Zhou; //Regularity;
	
	
	// ����ʱ�����õ������̵������ʣ�UtilOfEmployed 
	public LeafEvaluator leafEvaulator = LeafEvaluator.UtilOfEmployed; // Zhou: open pallet װ���ܺ������

	public int depth = 1;      // 0 ʹ�� Greedy
							   // depth >= 1: 
	                           //    simulationCount = 0: ���� LookAhead ��ǰ�� depth ��
	                           //    simulationCount >= 1: ��Ϊ LookAheadOnline Ϊÿ��simulated���е��� LookAhead ʱ��ǰ���Ĳ���
	public int effort = 16;	   // effort = nodesKepth^depth; LookAhead ����չ���������Ϊ depth �Ľڵ���
	//private int nodesKept = 16; // number ÿ�㱣�������ӽڵ����

	/**
	 * {@link LookAheadOnline} �㷨������ǰ���� {@link com.zhuwb.research.roboticpacking.inst.Placement} ʱ
	 * ���з���Ĵ�����
	 * <ul>0: ��ʾ�����棬��ʹ�� offline ���㷨�� {@link Greedy} ���� {@link LookAhead} </ul>
	 * <ul>>= 1: ��ʾ�� {@link LookAheadOnline} �㷨 </ul> 
	 */
	public int simulationCount = 5;
	
	// Ϊ��ǰ state ѡ���� placement ʱ
	// ����ÿ�������£�ÿ�� placement �õ��÷���
	// Ȼ��ѡһ�������г����µ÷���ߵ� placement
	//
	// voting:     ÿ�������£���õ� placement �� 1 �֣����� placment �� 0 ��; Zhou
	// totalVoume: ÿ�������£�һ�� placement �ķ����������������Ժ��� LookAhead �ҵ�����õķ����У�Open Pallet���ܺ������
	// totalVoume: ÿ�������£�һ�� placement �ķ����������������Ժ��� LookAhead �ҵ�����õķ����У�Open Pallet�к���������� pallet �����
	public SelectMode selectMode = SelectMode.voting;
	
	public double hweight = 1.0;     // zhou�е�zweight,H-�������Ȩ��
	
    // PushToEnd��  �����Σ�L-,W-push��Ѻ����Ƶ��ռ����ԭ�����
	// Grid:  L-push �ᵽ����ÿ�������ߣ����º���l1��l2�����к���l2����0һ����λ�ã�
	//        W-push �ᵽ����ÿ�������ߣ����º���w1��w2�����к���w2����0һ����λ�ã�
	// GridOnFloor:  ���� space �ĵ���߶�Ϊ 0 ʱ���� GridSearch
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
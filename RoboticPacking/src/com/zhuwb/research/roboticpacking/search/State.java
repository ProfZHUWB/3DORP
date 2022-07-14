package com.zhuwb.research.roboticpacking.search;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import com.zhuwb.research.roboticpacking.inst.ConveyorBelt;
import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.Placement;
import com.zhuwb.research.roboticpacking.inst.SystemInfo;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.space.ArrayListInPlaceSpaceManager.SpaceChecker;

public class State implements Cloneable {
	SystemInfo sysInfo;

	// gridSearch:
    // 		PushToEnd：  （周游）L-,W-push会把盒子推到空间最靠近原点的面
	// 		Grid:  L-push 会到尝试每个网格线（即新盒子l1或l2与已有盒子l2或者0一样的位置）
	//         	   W-push 会到尝试每个网格线（即新盒子w1或w2与已有盒子w2或者0一样的位置）
	// 		GridOnFloor:  仅当 space 的底面高度为 0 时尝试 GridSearch
	// PalletFitness
	final AlgoConfig algoConf;
	
	public int[] boxCountInRange; 	// 机械手当前可以操作的盒子，[t] 表示第 t 种盒子有多少个
	public int nextBoxIdx;      //  机械手操作范围之外的下一个盒子在到达序列中的下标
	public int[] arrivalSeq;   // [i]表示第i个到达盒子的种类
	public int boxCount; 		// arrivalSeq中始终有效的盒子数
	
	// 已装载的托盘列列表
	public Pallet closed; 
	private double loadedVolumeClosed = 0;
	private double closedPalletCount = 0;

	private SpaceChecker spaceChecker; // .remainingBoxCount[t] number of boxes to be load in the future

	// 正在装载的托盘列表
	public Pallet[] open;  // [i]表示第i个正在装载的pallet
	public int totalOpened; // 总共open的pallet数量

	
	//.........
	
	
	
	//方法：1. 枚举所有可能的Placement 
	
	/**
	 * 创建初始的State表示没有装载任何盒子的状态
	 * @param sysInfo
	 * @param algoConf
	 */
	public State(SystemInfo sysInfo, AlgoConfig algoConf) {
		this.sysInfo = sysInfo;
		this.algoConf = algoConf;
		this.arrivalSeq = sysInfo.inst.t;
		this.boxCount = sysInfo.inst.t.length;
		this.nextBoxIdx = Math.min(sysInfo.conf.boxCountInRange, this.arrivalSeq.length);
		this.boxCountInRange = new int[sysInfo.inst.boxType.length]; 
		for (int i=0;i<nextBoxIdx;i++) {
			int type = this.arrivalSeq[i]; 
			this.boxCountInRange[type]+=1;
		}
		
		this.closed = null;

		this.spaceChecker = new SpaceChecker(sysInfo.boxTypes, sysInfo.boxCountPerType);

		this.open = new Pallet[sysInfo.conf.openPalletCount];
		for (int i=0;i<this.open.length;i++) {
			this.open[i] = new Pallet(i,sysInfo,spaceChecker, GridSearch.PushToEnd);
		}
		this.totalOpened = this.open.length;

		//....	
	}
	
//	private State() {}
	protected State copy() {
		State s;
		try {
			s = (State) this.clone();
		} catch (CloneNotSupportedException e) {throw new RuntimeException(e); }
		
		s.boxCountInRange = Arrays.copyOf(this.boxCountInRange, this.boxCountInRange.length);
		s.open = Arrays.copyOf(this.open, this.open.length);
		return s;
	}
	
	public static class Operation {
		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + opIdx;
			result = prime * result + ((placement == null) ? 0 : placement.hashCode());
			return result;
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			Operation other = (Operation) obj;
			if (opIdx != other.opIdx)
				return false;
			if (placement == null) {
				if (other.placement != null)
					return false;
			} else if (!placement.equals(other.placement))
				return false;
			return true;
		}

		public int opIdx;
		public Placement placement; // null 表示是关闭 pallet
		
		public Operation(int opIdx) {
			this.opIdx = opIdx;
			this.placement = null;
		}

		public Operation(int opIdx, Placement placement) {
			this.opIdx = opIdx;
			this.placement = placement;
		}
		
		public boolean isClose() {
			return this.placement == null;
		}
		
		public String toString() {
			if (isClose()) {
				return "close "+opIdx;
			} else {
				return "place boxType: "+placement.boxType+" into open pallet at: "+opIdx;
			}
		}
	}
	
	public State prev;
	public Operation oper;
	
	/**
	 * 尝试找到无法装机器手范围中任何一个盒子的pallet，把最满的一个关闭，并开一个新pallet替换它。
	 * 如果没有这样的pallet就返回自己。
	 * @return
	 */
	public State close() {
		int bestIdx = -1;
		double loadedVolume = Double.NEGATIVE_INFINITY;
		for(int i=0;i<open.length;i++) {
			Pallet P = open[i];
			ArrayList<ArrayList<Placement>> placements = P.computeFeasiblePalcements(boxCountInRange);
			if(placements.size()!=0) { continue; }
			
			if (P.loadedVolume > loadedVolume) {
				loadedVolume = P.loadedVolume;
				bestIdx = i;
			}
		}
		
		if (bestIdx < 0) { return this; }

//		System.out.println("close pallet at: "+bestIdx+" with volume: "+loadedVolume);
		
		State s = this.copy();

		// 关闭pallet bestIdx: 即将pallet p放到关闭的pallet列表的最后一个
		s.closed = this.open[bestIdx];				
		s.closed.prev = this.closed; 
		s.loadedVolumeClosed += s.closed.loadedVolume;
		s.closedPalletCount += 1;

		// 开一个新的 pallet 替换已关闭的
		s.open[bestIdx] = new Pallet(s.totalOpened, this.sysInfo, this.spaceChecker, this.algoConf.gridSearch);
		s.totalOpened += 1;
		
		s.prev = this;
		s.oper = new Operation(bestIdx);
		
		return s;
	}
	
	
	/**
	 * 把一个盒子按照一个指定的方式p放入第opIdx个pallet中, 并且用 nextBoxType 替换机器手范围中
	 * 的一个盒子
	 * @param opIdx
	 * @param p
	 * @return
	 */
	public State place(int opIdx, Placement p) {
		State s = this.copy();

		// 更新放置了新盒子的pallet
		Pallet newP = this.open[opIdx].place(p);
		s.open[opIdx] = newP;
		s.spaceChecker = newP.getSpaceChecker();  // 剩余盒子减少一个，更新spaceChecker

		s.boxCountInRange[p.boxType] -= 1;

		// 从机器手范围外传送一个盒子进机器手范围内
		int nextBoxType = -1;
		if (this.nextBoxIdx < this.boxCount) {
			nextBoxType = arrivalSeq[this.nextBoxIdx];
			s.nextBoxIdx += 1;
			s.boxCountInRange[nextBoxType] += 1;
		}
		
		s.prev = this;
		s.oper = new Operation(opIdx, p);
		
		return s;
	}
	
	public State switchSequence(int[] arrivalSeq, int nextBoxIdx, int boxCount) {
		State s = this.copy();
		s.arrivalSeq = arrivalSeq;
		s.nextBoxIdx = nextBoxIdx;
		s.boxCount = boxCount;
		
		s.prev = this;
		return s;
	}
	
	/**
	 * 从 state 的全局看，评估有多个 Open Pallet 可以摆放新盒子时优先使用哪个托盘。目前实现了两个策略：
	 * 
	 * <ol>Dummy: 忽略不同托盘的影响</ol>
	 * <ol>UseEmptyIfNecessary: 非空托盘优先级一样；空的托盘如果不必要，优先级低于非空的。会避免把很少量的盒子单独装到一个托盘。</ol>
	 * 
	 * @author iwenc
	 */
	public static enum PalletFiteness {
		/**
		 * 无视托盘差异。
		 */
		Dummpy {
			@Override
			public double fitness(State s, Pallet p) {
				return 0;
			}
		}, 
		
		/**
		 * 如果没有必要尽量不用空的新托盘, 0 有必要用空托盘；分数越负越没有必要用空托盘。
		 * <ol> 如果没有装的盒子总体积大于已用 Open Pallet 所有可用空间，将来一定会用到空的 Open Pallet。</ol>
		 * <ol> 如果所有盒子装到已用 Open Pallet 中平均利用率已经低于历史平均，则用空托盘的必要性下降。</ol>
		 */
		UseEmptyIfNecessary {
			@Override
			public double fitness(State s, Pallet p) {
				if (!p.isEmpty()) {
					return 0;
				}
				
				if (s.open.length == 1) { return 0; }
				double remainingBoxVolume = s.sysInfo.totalBoxVolume - s.loadedVolumeClosed;
				if (remainingBoxVolume > s.open.length * s.sysInfo.palletVolume) {
					return 0;
				}
				
				double usedVolume = 0;
				int usedCount = 0;
				double minUsedVolume = s.sysInfo.palletVolume;
				for (int i=0; i<s.open.length; i++) {
					if (!s.open[i].isEmpty()) {
						usedVolume += s.open[i].loadedVolume;
						usedCount += 1;
						if (s.open[i].loadedVolume < minUsedVolume) {
							minUsedVolume = s.open[i].loadedVolume;
						}
					}
				}
				
				if (minUsedVolume < 0.5 * s.sysInfo.palletVolume) {// && usedCount >= 2) {
					return -0.5;
				}
				if ((usedCount-1) * s.sysInfo.palletVolume >= usedVolume) {
					return -1;
				}
				
				remainingBoxVolume += usedVolume;
				
				double totalSpaceInUsedPallet = usedCount * s.sysInfo.palletVolume;
				if (remainingBoxVolume >= totalSpaceInUsedPallet) {
					return 0;
				}
				
				double utilIfNotUseEmpty = remainingBoxVolume / totalSpaceInUsedPallet;
				double targetUtil = 0.8;
				int basePalletCount = 1;
				double expectedUtil = (targetUtil * basePalletCount * s.sysInfo.palletVolume + s.loadedVolumeClosed) 
						            / (basePalletCount + s.closedPalletCount) / s.sysInfo.palletVolume;
				if (utilIfNotUseEmpty >= expectedUtil) {
					return 0;
				}
				return utilIfNotUseEmpty - expectedUtil;
			}
		};
		
		
		public abstract double fitness(State s, Pallet p);
	}
	
	
	public static long totalPlacementCount = 0;
	public static long totalSteps = 0;
	public static double getAveragePlacementsPerState() {
		if (totalSteps == 0) {
			return 0;
		}
		return ((double) totalPlacementCount) / totalSteps;
	}
	protected ArrayList<PlacementWitFitness> computeAllPlacementWitFitness(Fitness fitness) {

		ArrayList<PlacementWitFitness> allPlacementWithFitness = new ArrayList<PlacementWitFitness>();
		
		boolean firstEmpty = true;
		for (int opIdx=0; opIdx<this.open.length; opIdx++) {
			Pallet P = this.open[opIdx];
			if (P.isEmpty()) {
				if (firstEmpty) { 		// 空的托盘只考虑第一个（之后的空托盘与第一个等价，跳过）
					firstEmpty = false;
				} else {
					continue;
				}
			}

			double palletFitness = this.algoConf.palletFitness.fitness(this, P);
			
			ArrayList<ArrayList<Placement>> allPlacements = 
					P.computeFeasiblePalcements(this.boxCountInRange);
			
			for(ArrayList<Placement> plist:allPlacements) {
				for(Placement p:plist) {
					PlacementWitFitness pp = new PlacementWitFitness(palletFitness, fitness.fitness(this.algoConf, P, p),opIdx,p);
					allPlacementWithFitness.add(pp);
				}
			}
		}
		
		State.totalPlacementCount += allPlacementWithFitness.size();
		State.totalSteps += 1;		
		return allPlacementWithFitness;
	}
	
	public ArrayList<PlacementWitFitness> findAll(Fitness fitness) {
		ArrayList<PlacementWitFitness> allPlacementWithFitness = computeAllPlacementWitFitness(fitness);
		// 对allPlacementWithFitness根据fitness排序
		Collections.sort(allPlacementWithFitness);
		return allPlacementWithFitness;
	}
	
	public PlacementWitFitness findBestPlacement(Fitness fitness) {
		ArrayList<PlacementWitFitness> allPlacementWithFitness = computeAllPlacementWitFitness(fitness);
		PlacementWitFitness bestPlacement = null;
		for (PlacementWitFitness p:allPlacementWithFitness) {
			if (bestPlacement == null || bestPlacement.compareTo(p) < 0) {
				bestPlacement = p;
			}
		}
		return bestPlacement;
	}
	
	public String toString() {
		StringBuffer sb = new StringBuffer();
		sb.append("boxCountInRange: "+Arrays.toString(this.boxCountInRange));
		sb.append(" remainingCount: "+Arrays.toString(spaceChecker.remainingBoxCount));
		sb.append(" arrival:");
		if (nextBoxIdx < boxCount) {
			for (int i=nextBoxIdx; i<Math.min(boxCount, nextBoxIdx+5); i++) {
				sb.append(" "+arrivalSeq[i]);
			}
		}
		sb.append("...");
		return sb.toString();
	}
	
	public Pallet[] getClosedPallets() {
		Pallet[] closed = null;
		if (this.closed != null) {
			closed = this.closed.findAllPallets();
		} else {
			closed = new Pallet[0];
		}
		return closed;
	}
	
	// 按种类统计尚未装载的盒子数
	public int[] cloneRemainingBoxCount() {
		return Arrays.copyOf(this.spaceChecker.remainingBoxCount, this.spaceChecker.remainingBoxCount.length);
	}
	
	public static class OperationRecord {
		public State state;			// the state where the operation is applied to
		int boxIdxInOperRange;		// index of box in the operation that is to be loaded; -1 close pallet operation
		int boxIdxInArrivalSeq;     // index of box in arrival sequence 
		int boxType;				// boxType loaded
		int palletIdxInOperRange;	// index of pallet in operation range
		int palletId;   			// the id of pallet
		Placement placement;
		public OperationRecord(State state,
				int boxIdxInOperRange, int boxIdxInArrivalSeq, int boxType,
				int palletIdxInOperRange, int palletId, Placement placement) {
			this.state = state;
			this.boxIdxInOperRange = boxIdxInOperRange;
			this.boxIdxInArrivalSeq = boxIdxInArrivalSeq;
			this.boxType = boxType;
			this.palletIdxInOperRange = palletIdxInOperRange;
			this.palletId = palletId;
			this.placement = placement;
		}
	}
	
	public ArrayList<OperationRecord> getLoadingOperations() {
		ArrayList<OperationRecord> recList = new ArrayList<>();
		
		ArrayList<State> stateList = new ArrayList<>();
		State cur = this;
		while (cur != null) {
			stateList.add(cur);
			cur = cur.prev;
		}
		
		InstData inst = this.sysInfo.inst;
		ConveyorBelt boxBelt = new ConveyorBelt(sysInfo.conf.boxCountInRange, inst.t.length);
		ConveyorBelt palletBelt = new ConveyorBelt(sysInfo.conf.openPalletCount, null);
		
		for (int i = stateList.size()-2; i>=0; i--) {
			State next = stateList.get(i);
			int boxIdx = -1;
			int palletIdx = next.oper.opIdx;
			if (!next.oper.isClose()) { // place
				int boxType = next.oper.placement.boxType;
				// 找到第一个与 boxType 匹配的位置
				for (int j=0; j<boxBelt.idxInRangeCount; j++) {
					int bIdx = boxBelt.idxInRange[j];
					if (inst.t[bIdx] == boxType) {
						boxIdx = j;
						break;
					}
				}
				if (boxIdx < 0) {
					System.out.print(" arrivalSeq: {");
					for (int j=0; j<boxBelt.idxInRangeCount; j++) {
						int bid = boxBelt.idxInRange[j];
						if (j>0) { System.out.print(" "); }
						System.out.print(inst.t[bid]);				
					}
					System.out.print("}");
					if (boxIdx >= 0) {
						for (int j=boxBelt.nextIdx; j<Math.min(boxBelt.nextIdx+5, inst.t.length); j++) {
							System.out.print(" "+inst.t[j]);
						}
						if (boxBelt.nextIdx+5 < inst.t.length) {
							System.out.print(" ...");
						}
					}
					System.out.println();
					System.out.println(" box belt: "+boxBelt.toString(boxIdx, null));
					System.out.println(" pallet belt: "+palletBelt.toString(palletIdx, null));
					System.out.println(" "+next.oper);
					
					throw new IllegalArgumentException();
				}
			}
			State pre = stateList.get(i+1);
			if (next.oper.isClose()) { // place
				palletBelt = palletBelt.remove(palletIdx);
				recList.add(new OperationRecord(pre, -1, -1, -1, palletIdx, pre.open[palletIdx].pid, null));
			} else {
				int idxInInst = boxBelt.idxInRange[boxIdx];
				int boxType = inst.t[idxInInst];
				recList.add(new OperationRecord(pre, boxIdx, idxInInst, boxType, palletIdx, pre.open[palletIdx].pid, next.oper.placement));
				boxBelt = boxBelt.remove(boxIdx);
			}
		}

		return recList;
	}
	
	public void printLoadingInstructions(PrintStream ps) {
		int maxRemovedToPrint = 5;
		int maxArrivalToPrint = 5;
		
		ps.println("box types: ");
		InstData inst = this.sysInfo.inst;
		for (int i=0; i<inst.boxType.length; i++) {
			ps.println("  type "+i+": "+Arrays.toString(inst.boxType[i])
				+" ortPerm: "+Arrays.toString(inst.ortPerm[i]));
		}
		ps.println("arrival seq: "+Arrays.toString(inst.t));
		
		ArrayList<OperationRecord> recList = getLoadingOperations();

		ConveyorBelt boxBelt = new ConveyorBelt(sysInfo.conf.boxCountInRange, inst.t.length);
		ConveyorBelt palletBelt = new ConveyorBelt(sysInfo.conf.openPalletCount, null);
		
		for (int step = 0; step < recList.size(); step++) {
			OperationRecord rec = recList.get(step);
			ps.println("step "+step+"; state: "+rec.state);
			ps.print("  arrivalSeq: {");
			for (int j=0; j<boxBelt.idxInRangeCount; j++) {
				int bid = boxBelt.idxInRange[j];
				if (j>0) { ps.print(" "); }
				ps.print(inst.t[bid]);				
			}
			ps.print("}");
			for (int j=boxBelt.nextIdx; j<Math.min(boxBelt.nextIdx+maxArrivalToPrint, inst.t.length); j++) {
				ps.print(" "+inst.t[j]);
			}
			if (boxBelt.nextIdx+5 < inst.t.length) {
				ps.print(" ...");
			}
			ps.println();
			
			ps.println("  box belt: "+boxBelt.toString(rec.boxIdxInOperRange,maxRemovedToPrint));
			ps.println("  pallet belt: "+palletBelt.toString(rec.palletIdxInOperRange,maxRemovedToPrint));
			if (rec.boxIdxInArrivalSeq == -1) {
				ps.println("  close pallet "+rec.palletId+" at "+rec.palletIdxInOperRange);
				palletBelt = palletBelt.remove(rec.palletIdxInOperRange);
			} else {
				ps.println("  place "+rec.boxIdxInArrivalSeq+"-th box (type: "+rec.boxType+") at "+rec.boxIdxInOperRange+" into "+rec.palletIdxInOperRange+"-th pallet at "+rec.palletIdxInOperRange);
				boxBelt = boxBelt.remove(rec.boxIdxInOperRange);
			}
		}
		ps.println("final box belt: "+boxBelt.toString(-1,maxRemovedToPrint));
		ps.println("final pallet belt: "+palletBelt.toString(-1,maxRemovedToPrint));
	}
	
	public void printLoadingInstructionsToFile(File file) throws FileNotFoundException {
		file.getParentFile().mkdirs();
		PrintStream ps = new PrintStream(new FileOutputStream(file));
		printLoadingInstructions(ps);
		ps.close();
	}

	public static void main(String[] args) throws IOException {
//		ArrayList<InstData> instList = InstanceLoader.load(2);
//		InstData inst = InstanceLoader.load(2,"SD0-2-1000-large-2");
	}
	
}

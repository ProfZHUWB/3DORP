package com.zhuwb.research.roboticpacking.search;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import com.zhuwb.LEAD.array.IntArray;
import com.zhuwb.LEAD.dtable.IntegerDynamicTable;
import com.zhuwb.research.roboticpacking.inst.BoxType;
import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;
import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.Mathematica;
import com.zhuwb.research.roboticpacking.inst.Placement;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.SystemInfo;
import com.zhuwb.research.roboticpacking.inst.Vacuum;
import com.zhuwb.research.roboticpacking.inst.Vacuum.Align;
import com.zhuwb.research.roboticpacking.inst.Vacuum.PushAxis;
import com.zhuwb.research.roboticpacking.inst.Vacuum.Type;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.space.ArrayListInPlaceSpaceManager;
import com.zhuwb.research.roboticpacking.space.ArrayListInPlaceSpaceManager.SpaceChecker;
import com.zhuwb.research.roboticpacking.space.Space;
import com.zhuwb.research.roboticpacking.space.Space.BaseRegion;
import com.zhuwb.research.roboticpacking.space.Space.Face;
import com.zhuwb.research.rpp2i.boxpp.Pair;
import com.zhuwb.research.rpp2i.boxpp.PairIterator;
import com.zhuwb.research.rpp2i.boxpp.segmenttree.GridPointBySegmentTree;
import com.zhuwb.research.rpp2i.boxpp.segmenttree.LayoutST;

public class Pallet implements Cloneable {
	public int pid; // pallet id, 第一个 open 的为 0， 顺序编号
	public SystemInfo sysInfo;
	public Space loadingSpace;
	
    // PushToEnd：  （周游）L-,W-push会把盒子推到空间最靠近原点的面
	// Grid:  L-push 会到尝试每个网格线（即新盒子l1或l2与已有盒子l2或者0一样的位置）
	//        W-push 会到尝试每个网格线（即新盒子w1或w2与已有盒子w2或者0一样的位置）
	// GridOnFloor:  仅当 space 的底面高度为 0 时尝试 GridSearch
	public final GridSearch gridSearch; 
	
	public Pallet prev; // 在已关闭的pallet列表中，排在this之前的pallet; none表示this是第一个
	
	// 每一种盒子对应的摆放方式
	// 1. type-->list of placement 对于这种类型，已计算出来所有可行的placement
	// 2. type --> null 对于这种类型，还没有计算可行的placement
	public  ArrayList<Placement>[] feasiblePlacements;  // [t]表示type t盒子对应的可行placements; null表示没有计算过

	// 已经装完的box列表，用LinkList表示
	public  PlacedBox placed;			// 表示所有已经摆放的盒子，也表示已摆放盒子中的最后一个摆放的盒子
	public int loadedCount = 0;		// 已经装入this的盒子个数
	public int[] loadedCountPerType; 
	public double loadedVolume=0;	// 已经装入this的盒子总体积
	
	public PlacedBox[] placedDescL1;    // 已经摆放的盒子按照 L1 坐标降序排列
	public PlacedBox[] placedDescW1;	// 已经摆放的盒子按照 W1 坐标降序排列
	public PlacedBox[] placedDescH1;	// 已经摆放的盒子按照 H1 坐标降序排列
	public PlacedBox[] placedDescL2;    // 已经摆放的盒子按照 L2 坐标降序排列
	public PlacedBox[] placedDescW2;	// 已经摆放的盒子按照 W2 坐标降序排列
	public PlacedBox[] placedDescH2;	// 已经摆放的盒子按照 H2 坐标降序排列
	
	// 如果盒子是紧凑摆放的，那么每个盒子的l1必然是0或者已经摆放的盒子的l2
	// lGrid x wGrid x hGrid 定义的网格点，包括了所有紧凑摆放的位置 
//	public TreeSet<Integer> lGrid;	// 0及已摆放的盒子的l2，
//	public TreeSet<Integer> wGrid;  // 0及已摆放的盒子的w2
//	public TreeSet<Integer> hGrid;  // 0及已摆放的盒子的h2
	int[] lGrid;
	int[] wGrid;
	int[] hGrid;

	// 所有可用的 Maximal Space
	public ArrayListInPlaceSpaceManager spaceManager;
	
	// 创建一个新的pallet
	@SuppressWarnings("unchecked")
	public Pallet(int pid, SystemInfo sysInfo, SpaceChecker spaceChecker, GridSearch gridSearch) {
		this.pid = pid;
		this.sysInfo = sysInfo;
		this.gridSearch = gridSearch;
		this.loadedCountPerType = new int[sysInfo.inst.getBoxTypeCount()];

		this.loadingSpace = new Space(0,0,0,sysInfo.inst.L,sysInfo.inst.W,sysInfo.inst.H);
		this.spaceManager = new ArrayListInPlaceSpaceManager(this.loadingSpace, spaceChecker);
		this.feasiblePlacements = new ArrayList[sysInfo.inst.boxType.length];
		this.placedDescL1 = new PlacedBox[0];
		this.placedDescW1 = new PlacedBox[0];
		this.placedDescH1 = new PlacedBox[0];
		this.placedDescL2 = new PlacedBox[0];
		this.placedDescW2 = new PlacedBox[0];
		this.placedDescH2 = new PlacedBox[0];
//		this.lGrid = new TreeSet<Integer>(); this.lGrid.add(Integer.valueOf(0));
//		this.wGrid = new TreeSet<Integer>(); this.wGrid.add(Integer.valueOf(0));
//		this.hGrid = new TreeSet<Integer>(); this.hGrid.add(Integer.valueOf(0));
		this.lGrid = new int[] {0};
		this.wGrid = new int[] {0};
		this.hGrid = new int[] {0};
	}
	
	public SpaceChecker getSpaceChecker() {
		return this.spaceManager.getSpaceChecker();
	}

	protected Pallet copy() {
		Pallet newP;
		try {
			newP = (Pallet) this.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
		return newP;
	}
	
	/**
	 * 安装指定面的坐标的降序把placed插入到一个列表，返回一个新列表
	 * @param placedList
	 * @param placed
	 * @param face
	 * @return
	 */
	public static PlacedBox[] insertDesc(PlacedBox[] placedList, PlacedBox placed, Face face) {
		PlacedBox[] newPlacedList = new PlacedBox[placedList.length+1];
		int i=placedList.length-1;
		int newCoord = placed.occupied.getFaceCoord(face);
		while (i>=0 && placedList[i].occupied.getFaceCoord(face) < newCoord) {
			newPlacedList[i+1] = placedList[i];
			i--;
		}
		newPlacedList[i+1] = placed;
		while (i>=0) {
			newPlacedList[i] = placedList[i];
			i--;
		}
		return newPlacedList;
	}
	
	/**
	 * 把一个盒子放在 this 返回新的 pallet 表示放完后的状态
	 * @param p
	 * @return
	 */
	@SuppressWarnings("unchecked")
	public Pallet place(Placement p) {
		if (p.boxPath == null) {
			p.boxPath = Vacuum.computeBoxPath(p.releaseL, p.releaseW, p.releaseH, p.ort,
					this.loadingSpace, p.pushAxis);
		}
		Pallet newP = this.copy();
		newP.spaceManager = this.spaceManager.copyAfterRemoveOneBox(p.boxType);
		newP.loadedCountPerType = Arrays.copyOf(this.loadedCountPerType, this.loadedCountPerType.length);
		
		// 清空之前计算的可行摆放方式
		newP.feasiblePlacements = new ArrayList[sysInfo.inst.boxType.length];
		
		// 新放的 box
		newP.placed = new PlacedBox(p, this.placed);
		newP.loadedCount = this.loadedCount + 1;
		newP.loadedCountPerType = Arrays.copyOf(this.loadedCountPerType, this.loadedCountPerType.length);
		newP.loadedCountPerType[p.boxType] = this.loadedCountPerType[p.boxType] + 1;
		newP.loadedVolume = this.loadedVolume + p.occupied.volume;
		
		newP.placedDescL1 = insertDesc(this.placedDescL1, newP.placed, Face.L1); 
		newP.placedDescW1 = insertDesc(this.placedDescW1, newP.placed, Face.W1); 
		newP.placedDescH1 = insertDesc(this.placedDescH1, newP.placed, Face.H1); 
		newP.placedDescL2 = insertDesc(this.placedDescL2, newP.placed, Face.L2); 
		newP.placedDescW2 = insertDesc(this.placedDescW2, newP.placed, Face.W2); 
		newP.placedDescH2 = insertDesc(this.placedDescH2, newP.placed, Face.H2); 
	
		newP.lGrid = IntArray.insertAscendingNoDuplicate(this.lGrid, p.occupied.l2);
		newP.wGrid = IntArray.insertAscendingNoDuplicate(this.wGrid, p.occupied.w2);
		newP.hGrid = IntArray.insertAscendingNoDuplicate(this.hGrid, p.occupied.h2);
		
		// 新放的 box 占用了空间
		newP.spaceManager.updateSpaces(p.occupied);
		
//		newP.validate();
		
		return newP;
	}
	
	// check if all placedbox are inside loading space
	// no placed box overlap with free space
	// no placed box overlap
	// sysInfo.conf.ignoreCollision = true, or
	// 		boxPath, dropPath and gripperPath doe not overlap with previous placed boxes
	// 		dropping is valid
	// placed boxes are supported
	public void validate() {
		Space loadingSpace = new Space(0,0,0,sysInfo.inst.L, sysInfo.inst.W, sysInfo.inst.H);
		Placement[] placements = getPlacements();
		for (int j=0; j<placements.length; j++) {
			Placement p = placements[j];
			if (!loadingSpace.contains(p.occupied)) {
				genDebugCode();
				throw new RuntimeException("placed box outside loading space, pid: "+pid+", placement: "+p);
			}
			
			// does not intersect with any space
			for (Space s:this.spaceManager.getFreeSpace()) {
				if (p.occupied.intersectTest(s)) {
					genDebugCode();
					throw new RuntimeException("placement: "+p+" intersects with free space: "+s);
				}
			}
			
			if ((!sysInfo.conf.ignoreCollision) && p.dh > p.ort.maxDropH) {
				genDebugCode();
				throw new RuntimeException("placement: "+p+", drop height exceed maxDropH in orientation: "+p.ort);
			}
			
			int[] supportFlag = new int[BaseRegion.values().length];
			
			for (int k=0; k<j; k++) {
				Placement placed = placements[k];
				if (p.occupied.intersectTest(placed.occupied)) {
					genDebugCode();
					p.print("  ", System.out); System.out.flush();
					placed.print("  ", System.out); System.out.flush();
					throw new RuntimeException("j="+j+" placement: "+p+", box overlap with existing: "+placed+" k="+k);
				}
				if (sysInfo.conf.ignoreCollision) {
					continue;
				}
				if (p.dropPath.intersectTest(placed.occupied)) {
					genDebugCode();
					p.print("  ", System.out); System.out.flush();
					placed.print("  ", System.out); System.out.flush();
					throw new RuntimeException("j="+j+" placement: "+p+", drop path: "+p.dropPath+" overlap with existing: "+placed.occupied+" k="+k);
				}
				if (p.boxPath.intersectTest(placed.occupied)) {
					genDebugCode();
					p.print("  ", System.out); System.out.flush();
					placed.print("  ", System.out); System.out.flush();
					throw new RuntimeException("placement: "+p+", box path: "+p.boxPath+" overlap with existing: "+placed.occupied+" k="+k);
				}
				if (p.gripperPath.intersectTest(placed.occupied)) {
					genDebugCode();
					p.print("  ", System.out); System.out.flush();
					placed.print("  ", System.out); System.out.flush();
					throw new RuntimeException("j="+j+" placement: "+p+", gripper path: "+p.gripperPath+" overlap with existing: "+placed.occupied+" k="+k);
				}
				
				if (placed.occupied.h1 > 0) {
					int[] sf = placed.occupied.countSupportRegion(p.occupied, 
							p.ort.h * sysInfo.conf.maxDropHeightRatio, sysInfo.conf.minOverlapRatioForSupport);
					for (int f=0; f<sf.length; f++) {
						supportFlag[f] += sf[f];
					}
				}
			}

			// 检查是否support
			if (p.occupied.h1 > 0) {
				int supportRegionCount = 0;
				for (int sr = 0; sr<supportFlag.length; sr++) {
					if (supportFlag[sr] > 0) {
						supportRegionCount ++;
					}
				}
				if (supportRegionCount < 0) {
					genDebugCode();
					throw new RuntimeException("placement: "+p+", is not fully supported. SupportFlag: "+Arrays.toString(supportFlag));
				}
			}

			// TODO: 检查是否吸附
		}
	}

	
	/**
	 * 给定每种盒子的个数，计算在当前pallet中所有可行的摆放方式
	 * 只计算盒子数量大于0的种类，只返回可行placement数量大于0的种类
	 * @param boxCount
	 * @return [i]表示第i种有placement的盒子对应的所有可行placement
	 */
	protected ArrayList<ArrayList<Placement>> computeFeasiblePalcements(int[] boxCount) {
		ArrayList<ArrayList<Placement>> result = new ArrayList<ArrayList<Placement>>();
		for (int t=0;t<boxCount.length;t++) {
			if (boxCount[t]==0) {continue;}
			ArrayList<Placement> placements = computeFeasiblePalcements(t);
			if (placements.size()>0) {
				result.add(placements);
			}
		}
		return result;
	}
	
//	public boolean debug;
	
	// 返回或者计算 type t 盒子对应的可行摆放方式
	protected ArrayList<Placement> computeFeasiblePalcements(int boxType){
		ArrayList<Placement> placements = this.feasiblePlacements[boxType];
		if (placements==null) {
			HashMap<Placement, Placement> pTable = new  HashMap<Placement, Placement>();
			placements = new ArrayList<Placement>();
			this.feasiblePlacements[boxType] = placements;
			
			BoxType type = sysInfo.boxTypes[boxType];
			for (Orientation ort:type.distinctOrt) { // all possible unique orientation
//				System.out.println("---- ort: "+ort);
				for (Space s: spaceManager.getFreeSpace()) {
//					System.out.println("==== free space: "+s);
					if(!s.largeEnoughFor(ort.l, ort.w, ort.h)) {continue;}
					// s可以容纳当前的盒子的旋转方式ort
					// 沿着L去找L推送的所有可行placements
					
					createLWGridForPush(s,ort);

					if (sysInfo.conf.enableLPush) {
//						System.out.println("==== try L-push, before count: "+placements.size());
						findPlacementsLPush(s, boxType, ort, placements, pTable);
					}
					if (sysInfo.conf.enableWPush) {
//						System.out.println("==== try W-push, before count: "+placements.size());
						findPlacementsWPush(s, boxType, ort, placements,  pTable);
					}
					if (sysInfo.conf.enableHPush) {
//						System.out.println("==== try H-push, before count: "+placements.size());
						findPlacementsHPush(s, boxType, ort, placements,  pTable);
					}
//					System.out.println("==== placement count: "+placements.size());
				}
			}
			
		}
		return placements;
	}
	
	/**
	 * 把一个 boxType 的盒子按照旋转方式 ort 能沿各种方向推入space的最终位置找到。
	 * 1. 总是推导 s 最里面
	 * 2. 停的位置为 normal position
	 * 3. 没有考虑支撑，及抓手是否可以无障碍推送
	 * @param s			目标空间
	 * @param boxType	待放盒子类型
	 * @param ort		待放盒子的摆放方式
	 * @param placements	保存找到的placement
	 * @param pTable		保存找到的placement
	 */
	protected void findPlacementsLPush(Space s, int boxType, Orientation ort, ArrayList<Placement> placements, HashMap<Placement, Placement> pTable) {
		if ((!sysInfo.conf.ignoreCollision) && !sysInfo.gripper.canLPush(ort)) { return; }

		// 试图找出所有可以把盒子可以推到的位置
		// 尝试L-push下可用空间s所有可以放ort的位置
		if(s.pushProjectionL==null) {
			s.pushProjectionL = createProjectionAndHCandidateForLPush(s);//,ort);
		}
		

		// L-Push
		int[] lGrid = null;
		switch (gridSearch) {
		case PushToEnd: lGrid = new int[] {s.l1}; break;
		case GridOnFloor: 
			if (s.h1 == 0) {
				lGrid = select(this.lGrid,s.l1,s.l2-ort.l);
			} else {
				lGrid = new int[] {s.l1};
			}
			break;
		case UseGrid: 
			if (s.h1 == 0) {
				lGrid = select(this.lGrid,s.l1,s.l2-ort.l);
			} else {
				lGrid = s.lGrid; 
			}
			break;
		}
		int[] wGrid = null;
		if (s.h1 == 0) {
			wGrid = select(this.wGrid,s.w1,s.w2-ort.w);
		} else {
			wGrid = s.wGrid;
		}
		int[] hGrid = select(s.hGridLPush,s.h1,Math.min(s.h1+ort.maxDropH, s.h2-ort.h));
		PairIterator iter = GridPointBySegmentTree.findGridPoints(
					s.pushProjectionL, ort.w, ort.h, wGrid, hGrid);
		
//		if (debug == true && boxType == 4 && ort.ort == 5 && s.l1 == 0 && s.w1 == 51 && s.h1 == 65) {
//			System.out.println("---- space: "+s);
//			System.out.println("========== lGrid: "+Arrays.toString(lGrid));
//			System.out.println("========== wGrid: "+Arrays.toString(wGrid));
//			System.out.println("========== s.hGridLPush: "+Arrays.toString(s.hGridLPush));
//			System.out.println("Region X1,Y1,X2,Y2: "+s.pushProjectionL.X1+","+s.pushProjectionL.Y1+","+s.pushProjectionL.X2+","+s.pushProjectionL.Y2);
//			for (int i=0; i<s.pushProjectionL.n; i++) {
//				System.out.println("placed x1,y1,x2,y2: "+s.pushProjectionL.x1[i]+","+s.pushProjectionL.y1[i]+","+s.pushProjectionL.x2[i]+","+s.pushProjectionL.x2[i]);				
//			}
//			System.out.println("ort.w: "+ort.w+", ort.h: "+ort.h);
//		}

		Pair loc = null;
		while ((loc = iter.next()) != null) {
			for (int releaseL:lGrid) {

				Placement p = new Placement();
				p.boxType = boxType;
				p.ort = ort;
				p.releaseL = releaseL;
				p.releaseW = loc.x;
				p.releaseH = loc.y;
				p.spaceH = s.h1;

//				if (debug == true && boxType == 4 && ort.ort == 5 && s.h1 == 65) {
//					System.out.println("-- loc.x: "+loc.x+", loc.y: "+loc.y);
//				}
				
				// A placement to push the same location already exists
				if (pTable.get(p) != null) { continue; }
	
				// 1. 找下落过程中碰到的第一个盒子，并计算高度差
				int supportBoxIdx = searchFirstSupport(p,ort);
				p.dh = searchForDropHeight(p, supportBoxIdx);	
				
				// 如果下落太多，就不行
				if ((!sysInfo.conf.ignoreCollision) && p.dh > ort.maxDropH) { continue; }
				// 2. 计算盒子所占用空间及盒子下落轨迹
				p.computeOccupiedAndDrop();
	
				// 3. 计算是否有支撑
				if (!supported(p,supportBoxIdx)) { continue; }
	
				// 4. 抓手能否图送到这个位置, 尝试两个旋转方式，任何一个方式可以都行
				// a. 按抓手的正常摆放方式推送，计算抓手的轨迹
				Vacuum gripper = sysInfo.gripper;
				Space gripperPath = gripper.gripperPathL(p.releaseL,p.releaseW,p.releaseH, 
										ort, sysInfo.inst.L, Align.org);
				// 抓手的下边界必须高于托盘的底面且抓手轨迹与已有盒子不碰撞
				if(sysInfo.conf.ignoreCollision ||
						gripperPath.h1 >= 0 && !checkCollisionForLPush(gripperPath)) { // 已经找到可行方案					
					p.gripperPath = gripperPath;   // org
					p.pushAxis = PushAxis.L;
					p.align = Align.org;
					placements.add(p);
					pTable.put(p,p);
					continue;
				}
				// b. 抓手旋转90度，计算抓手轨迹
				gripperPath = gripper.gripperPathL(p.releaseL,p.releaseW,p.releaseH, 
						ort, sysInfo.inst.L, Align.rotated);
				// 抓手的下边界必须高于托盘的底面且抓手轨迹与已有盒子不碰撞
				if (gripperPath.h1 >= 0 && !checkCollisionForLPush(gripperPath)) {
					p.gripperPath = gripperPath;  //rotated
					p.pushAxis = PushAxis.L;
					p.align = Align.rotated;
					placements.add(p);
					pTable.put(p,p);
				}
			} // for releaseL
		} // while releaseH and releaseW
	}
	
//	public static boolean debug = false;
	
	private void createLWGridForPush(Space s, Orientation ort) {
		//在h轴上投影，与Space相交且高度在合适范围之内的已放盒子的上表面是潜在支撑ort的，根据它们来计算摆放ort的可能L与W坐标
		int maxH2 = s.h2 - ort.h;   // 上表面超过maxH2的盒子不可能支撑ort
		int minH2 = s.h1 - ort.maxDropH;  // 上表面低于minH2的盒子不可能支撑ort
		
		IntegerDynamicTable lTable = new IntegerDynamicTable(10);
		lTable.append(s.l1); lTable.append(s.l2-ort.l);

		IntegerDynamicTable wTable = new IntegerDynamicTable(10);
		wTable.append(s.w1); wTable.append(s.w2-ort.w);
		
		for (int i=0; i<this.placedDescH2.length;i++) {
			Space curSpace = this.placedDescH2[i].occupied;
			if (curSpace.h2>maxH2) {continue;}
			if (curSpace.h2<minH2) {break;}
			if (s.projectionIntersectH(curSpace)) { 
				if (curSpace.l1>s.l1 && curSpace.l1+ort.l < s.l2) {
					lTable.append(curSpace.l1);
				}
				if (curSpace.l2>s.l1+ort.l && curSpace.l2<s.l2) {
					lTable.append(curSpace.l2-ort.l);
				}
				if (curSpace.w1>s.w1 && curSpace.w1+ort.w < s.w2) {
					wTable.append(curSpace.w1);
				}
				if (curSpace.w2>s.w1+ort.w && curSpace.w2<s.w2) {
					wTable.append(curSpace.w2-ort.w);
				}
			}
		}
		s.lGrid = lTable.toArray(); Arrays.sort(s.lGrid);
		s.wGrid = wTable.toArray(); Arrays.sort(s.wGrid);
	}
	
	/**
	 * 找到所有可能阻挡新盒子沿j轴进入空间s的已摆放盒子的投影。并且计算可能成为新盒子推送位置的的H的值，按升序存在s.hCandidateForLPush中
	 * @param s
	 * @return
	 */
	private LayoutST createProjectionAndHCandidateForLPush(Space s) { //{, Orientation ort) {		
		IntegerDynamicTable hTable = new IntegerDynamicTable(10); // 阻挡盒子的上边界是我们关心的推送的盒子的可能h坐标,详见doc/2021-05-20-1与-2
		hTable.append(s.h1);

		boolean print = false;
//		if (debug && s.l1 == 0 && s.w1 == 51 && s.h1 == 65 && s.l2 == 24 && s.w2 == 100 && s.h2 == 150) {
//			System.out.println("xxxxxxxxxxxxx");
//			print = true;
//		}
				
		// 有一块长方形区间所在位置为x1,y1,x2,y2
		// 有n个长方形与这个区间重叠
		int count=0;
		for (int i=0; i<this.placedDescL2.length; i++) {
			Space curSpace = this.placedDescL2[i].occupied;
			
//			if (print && curSpace.l1 == 24 && curSpace.w1 == 38 && curSpace.h1 == 51 &&
//					curSpace.l2 == 48 && curSpace.w2 == 53 && curSpace.h2 == 69) {
//				System.out.println("curSpace.l2 <= s.l2? "+(curSpace.l2 <= s.l2));
//			}
			
			if (curSpace.l2 <= s.l2) { break; }  // 只要 curSpace.l2 <= s.l2 就一定不会阻碍推送
			if (s.projectionIntersectL(curSpace)) { 
				count++; 
				
//				if (print && curSpace.l1 == 24 && curSpace.w1 == 38 && curSpace.h1 == 51 &&
//						curSpace.l2 == 48 && curSpace.w2 == 53 && curSpace.h2 == 69) {
//					System.out.println("?????curSpace.l2 <= s.l2? "+(curSpace.l2 <= s.l2));
//				}
				
				//if (curSpace.h2+ort.h <= s.h2 && curSpace.h2 <= s.h1 + ort.maxDropH) { // 掉落超过 maxDropH 的位置不可行
				if (curSpace.h2 < s.h2) { // h2 in [s.h1, s.h2)
					hTable.append(curSpace.h2);
				}
			}
		}
		s.hGridLPush = hTable.toArray(); Arrays.sort(s.hGridLPush);
		
		int[] x1=new int[count];
		int[] y1=new int[count];
		int[] x2=new int[count];
		int[] y2=new int[count];
		count=0;
		for (int i=0; i<this.placedDescL2.length; i++) {
			Space curSpace = this.placedDescL2[i].occupied;
			if (curSpace.l2 <= s.l2) { break; }  // 只要 curSpace.l2 <= s.l2 就一定不会阻碍推送
			if (s.projectionIntersectL(curSpace)) { 
				x1[count] = curSpace.w1;
				y1[count] = curSpace.h1;
				x2[count] = curSpace.w2;
				y2[count] = curSpace.h2;
				
//				if (print) {
//					System.out.println(count+" x1, y1, x2, y2: "+x1[count]+","+y1[count]+","+x2[count]+","+y2[count]);
//				}
				
				count++;
			}
		}
//		if (print) {
//			System.out.println(" X1, Y1, X2, Y2: "+s.w1+","+s.h1+","+s.w2+","+s.h2);
//		}
		// 重复的投影面会被LayoutST忽略
		LayoutST layout = new LayoutST(s.w1,s.h1,s.w2,s.h2,count,x1,y1,x2,y2);
//		if (print) {
//			System.out.println(" ------ "+layout.n);
//			for (int i=0; i<layout.n; i++) {
//				System.out.println(i+" x1, y1, x2, y2: "+layout.x1[i]+","+layout.y1[i]+","+layout.x2[i]+","+layout.y2[i]);
//			}
//		}
		
		return layout;
	}
	
	// select all a[i] >= min && <= max
	private int[] select(int[] a, int min, int max) {
		int count = 0;
		for (int i=0; i<a.length; i++) {
			if (min <= a[i] && a[i] <= max) { count++; }
		}
		int[] b = new int[count];
		count = 0;
		for (int i=0; i<a.length; i++) {
			if (min <= a[i] && a[i] <= max) { b[count++] = a[i]; }
		}
		return b;
	}
	
	/**
	 * 在从 p.releaseH 开始往下找到 ort 可以自由下落的高度
	 * @param p
	 * @param ort
	 * @return dropHeight of ort
	 */
	private int searchForDropHeight(Placement p, int supportBoxIdx) {
		if (supportBoxIdx == this.placedDescH2.length) {
			return p.releaseH;
		}
		return p.releaseH - this.placedDescH2[supportBoxIdx].occupied.h2;
	}
	// 从 p.releaseL, p.releaseW, p.releaseH 盒子下落中遇到的第一盒子的上表面
	private int searchFirstSupport(Placement p, Orientation ort) {
		int lowestPermittedH = p.releaseH - ort.maxDropH;
		int L1 = p.releaseL;
		int L2 = p.releaseL + ort.l;
		int W1 = p.releaseW;
		int W2 = p.releaseW + ort.w;
		
//		System.out.println("xxxxx lowestPermittedH: "+lowestPermittedH+"; L1,L2,W1,W2: "+L1+","+L2+","+W1+","+W2);
		
		for (int i=0; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied;
			if (curSpace.h2 > p.releaseH) { continue; }
			if (curSpace.h2 < lowestPermittedH) { return i; }

			// curSpace and ort intersect in LxW plane
			if (curSpace.l1 < L2 && curSpace.w1 < W2 && L1 < curSpace.l2 && W1 < curSpace.w2) {
				return i;
			}
		}
		return this.placedDescH2.length;
	}
	
	/**
	 * 计算 occupied 是否被下面的盒子支撑
	 * @param p
	 * @return
	 */
	private boolean supported(Placement p, int firstSupportBoxIdx) {
		if (firstSupportBoxIdx == this.placedDescH2.length) { return true; } // 地板一定可以支撑
		
		double lowestPermittedH = p.occupied.h1 - sysInfo.conf.maxHGapForSupport; 
		
		double L1 = p.occupied.l1, Lc = (p.occupied.l1 + p.occupied.l2)/2.0, L2 = p.occupied.l2;
		double W1 = p.occupied.w1, Wc = (p.occupied.w1 + p.occupied.w2)/2.0, W2 = p.occupied.w2;
		double minOverlapL = p.occupied.dL * sysInfo.conf.minOverlapRatioForSupport;
		double minOverlapW = p.occupied.dW * sysInfo.conf.minOverlapRatioForSupport;
		
		int[] supportFlag = new int[4];
		int flagCount = 0;
		
		for (int i=firstSupportBoxIdx; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied;
			if (curSpace.h2 < lowestPermittedH) { break; }
			// curSpace and ort intersect in LxW plane
			if (curSpace.l1 < L2 && curSpace.w1 < W2 && L1 < curSpace.l2 && W1 < curSpace.w2) {
				if (supportFlag[0] == 0 && Space.overlap(curSpace.l1, curSpace.l2, L1, Lc) >= minOverlapL &&
						Space.overlap(curSpace.w1, curSpace.w2, W1, Wc) >= minOverlapW) {
					supportFlag[0] = 1;
					flagCount++;
					if (flagCount >= 3) { return true; }
				}
				if (supportFlag[1] == 0 && Space.overlap(curSpace.l1, curSpace.l2, L1, Lc) >= minOverlapL &&
						Space.overlap(curSpace.w1, curSpace.w2, Wc, W2) >= minOverlapW) {
					supportFlag[1] = 1;
					flagCount++;
					if (flagCount >= 3) { return true; }
				}
				if (supportFlag[2] == 0 && Space.overlap(curSpace.l1, curSpace.l2, Lc, L2) >= minOverlapL &&
						Space.overlap(curSpace.w1, curSpace.w2, W1, Wc) >= minOverlapW) {
					supportFlag[2] = 1;
					flagCount++;
					if (flagCount >= 3) { return true; }
				}
				if (supportFlag[3] == 0 && Space.overlap(curSpace.l1, curSpace.l2, Lc, L2) >= minOverlapL &&
						Space.overlap(curSpace.w1, curSpace.w2, Wc,W2) >= minOverlapW) {
					supportFlag[3] = 1;
					flagCount++;
					if (flagCount >= 3) { return true; }
				}
			}
		}
		return false;
	}
	

	
	// 检查已摆放的盒子与机械手的手臂在L方向走过的空间是否发生碰撞
	private boolean checkCollisionForLPush(Space gripperPath) {
		for (int i=0; i<this.placedDescL2.length; i++) {
			Space curSpace = this.placedDescL2[i].occupied; // 已经摆放盒子的当前space
			if (curSpace.l2 <= gripperPath.l1) { return false; } // 剩余的盒子一定不会相交
			if (gripperPath.projectionIntersectL(curSpace)) { return true;}
		}
		return false;
	}
	
	
	
	
	
	/**
	 * 把一个 boxType 的盒子按照旋转方式 ort 能沿各种方向推入space的最终位置找到。
	 * 1. 总是推导 s 最里面
	 * 2. 停的位置为 normal position
	 * 3. 没有考虑支撑，及抓手是否可以无障碍推送
	 * @param s			目标空间
	 * @param boxType	待放盒子类型
	 * @param ort		待放盒子的摆放方式
	 * @param placements	保存找到的placement
	 * @param pTable		保存找到的placement
	 */
	protected void findPlacementsWPush(Space s, int boxType, Orientation ort, ArrayList<Placement> placements, HashMap<Placement, Placement> pTable) {
		if ((!sysInfo.conf.ignoreCollision) && !sysInfo.gripper.canWPush(ort)) { return; }

		// 试图找出所有可以把盒子可以推到的位置
		// 尝试W-push下可用空间s所有可以放ort的位置
		if(s.pushProjectionW==null) {
			s.pushProjectionW = createProjectionAndHCandidateForWPush(s); //,ort);
		}

		// W-Push
		int[] wGrid = null;
		switch (gridSearch) {
		case PushToEnd: wGrid = new int[] {s.w1}; break;
		case UseGrid: 
			if (s.h1 == 0) {
				wGrid = select(this.wGrid,s.w1,s.w2-ort.w);
			} else {
				wGrid = s.wGrid; 
			}
			break;
		case GridOnFloor: 
			if (s.h1 == 0) {
				wGrid = select(this.wGrid,s.w1,s.w2-ort.w);
			} else {
				wGrid = new int[] {s.w1};
			}
			break;
		}
		int[] lGrid = null;
		if (s.h1 == 0) {
			lGrid = select(this.lGrid,s.l1,s.l2-ort.l);
		} else {
			lGrid = s.lGrid;
		}
		int[] hGrid = select(s.hGridWPush,s.h1,Math.min(s.h1+ort.maxDropH, s.h2-ort.h));
		PairIterator iter = GridPointBySegmentTree.findGridPoints(
				s.pushProjectionW, ort.l, ort.h, lGrid, hGrid);

		Pair loc = null;
		while ((loc = iter.next()) != null) {
			for (int releaseW:wGrid) {
				Placement p = new Placement();
				p.boxType = boxType;
				p.ort = ort;
				
				p.releaseL = loc.x;			
				p.releaseW = releaseW;
				p.releaseH = loc.y;
				p.spaceH = s.h1;
				
				// A placement to push the same location already exists
				if (pTable.get(p) != null) { continue; }
	
		
				// 1. 找下落过程中碰到的第一个盒子，并计算高度差
				int supportBoxIdx = searchFirstSupport(p,ort);
				p.dh = searchForDropHeight(p, supportBoxIdx);
				// 如果下落太多，就不行
				if ((!sysInfo.conf.ignoreCollision) && p.dh > ort.maxDropH) { continue; }
				// 2. 计算盒子所占用空间及盒子下落轨迹
				p.computeOccupiedAndDrop();
	
				// 3. 计算是否有支撑
				if (!supported(p,supportBoxIdx)) { continue; }
	
				
				// 4. 抓手能否图送到这个位置, 尝试两个旋转方式，任何一个方式可以都行
				// a. 按抓手的正常摆放方式推送，计算抓手的轨迹
				Vacuum gripper = sysInfo.gripper;
				Space gripperPath = gripper.gripperPathW(p.releaseL,p.releaseW,p.releaseH, 
										ort, sysInfo.inst.W, Align.org);
				// 抓手的下边界必须高于托盘的底面且抓手轨迹与已有盒子不碰撞
				if (sysInfo.conf.ignoreCollision ||
						gripperPath.h1 >= 0 && !checkCollisionForWPush(gripperPath)) { // 已经找到可行方案
					p.gripperPath = gripperPath;   // org
					p.pushAxis = PushAxis.W;
					p.align = Align.org;
					placements.add(p);
					pTable.put(p,p);
					continue;
				}
				// b. 抓手旋转90度，计算抓手轨迹
				gripperPath = gripper.gripperPathW(p.releaseL,p.releaseW,p.releaseH, 
						ort, sysInfo.inst.W, Align.rotated);
				// 抓手的下边界必须高于托盘的底面且抓手轨迹与已有盒子不碰撞
				if (gripperPath.h1 >= 0 && !checkCollisionForWPush(gripperPath)) {
					p.gripperPath = gripperPath;  //rotated
					p.pushAxis = PushAxis.W;
					p.align = Align.rotated;
					placements.add(p);
					pTable.put(p,p);
				}
			}
		}
	}
	
	private LayoutST createProjectionAndHCandidateForWPush(Space s) {//, Orientation ort) {
		//在h轴上投影，与Space相交且高度在合适范围之内的已放盒子的上表面是潜在支撑ort的，根据它们来计算摆放ort的可能L与W坐标
		
		IntegerDynamicTable hTable = new IntegerDynamicTable(10);
		hTable.append(s.h1);
		
		// 有一块长方形区间所在位置为x1,y1,x2,y2
		// 有n个长方形与这个区间重叠
		int count=0;
		for (int i=0; i<this.placedDescW2.length; i++) {
			Space curSpace = this.placedDescW2[i].occupied;
			if (curSpace.w2 <= s.w2) { break; } // 只要 curSpace.w2 <= s.w2 就一定不会阻碍推送
			if (s.projectionIntersectW(curSpace)) { 
				count++;
				
//				if (curSpace.h2+ort.h <= s.h2 && curSpace.h2 <= s.h1 + ort.maxDropH) { // 掉落超过 maxDropH 的位置不可行
				if (curSpace.h2 < s.h2) { // 掉落超过 maxDropH 的位置不可行
					hTable.append(curSpace.h2);
				}
			}
		}
		s.hGridWPush = hTable.toArray(); Arrays.sort(s.hGridWPush);
		
		int[] x1=new int[count];
		int[] y1=new int[count];
		int[] x2=new int[count];
		int[] y2=new int[count];
		count=0;
		for (int i=0; i<this.placedDescW2.length; i++) {
			Space curSpace = this.placedDescW2[i].occupied;
			if (curSpace.w2 <= s.w2) { break; } // 只要 curSpace.w2 <= s.w2 就一定不会阻碍推送
			if (s.projectionIntersectW(curSpace)) {
				x1[count] = curSpace.l1;
				y1[count] = curSpace.h1;
				x2[count] = curSpace.l2;
				y2[count] = curSpace.h2;
				count++;
			}
		}
		// 重复的投影面会被LayoutST忽略
		return new LayoutST(s.l1,s.h1,s.l2,s.h2,count,x1,y1,x2,y2);
	}
	
	// 检查已摆放的盒子与机械手的手臂在W方向走过的空间是否发生碰撞
	private boolean checkCollisionForWPush(Space gripperPath) {
		for (int i=0; i<this.placedDescW2.length; i++) {
			Space curSpace = this.placedDescW2[i].occupied; // 已经摆放盒子的当前space
			if (curSpace.w2 <= gripperPath.w1) { return false; } // 剩余的盒子一定不会相交
			if (gripperPath.projectionIntersectW(curSpace)) { return true;}
		}
		return false;
	}

	
	
	
	/**
	 * 把一个 boxType 的盒子按照旋转方式 ort 能沿各种方向推入space的最终位置找到。
	 * 1. 总是推导 s 最里面
	 * 2. 停的位置为 normal position
	 * 3. 没有考虑支撑，及抓手是否可以无障碍推送
	 * @param s			目标空间
	 * @param boxType	待放盒子类型
	 * @param ort		待放盒子的摆放方式
	 * @param placements	保存找到的placement
	 * @param pTable		保存找到的placement
	 */
	protected void findPlacementsHPush(Space s, int boxType, Orientation ort, ArrayList<Placement> placements, HashMap<Placement, Placement> pTable) {
		if ((!sysInfo.conf.ignoreCollision) && !sysInfo.gripper.canHPush(ort)) { return; }

		// 试图找出所有可以把盒子可以推到的位置
		// 尝试H-push下可用空间s所有可以放ort的位置
		if(s.pushProjectionH==null) {
			s.pushProjectionH = createProjectionForHPush(s,ort);
		}

		// H-Push
		int[] lGrid = null;
		int[] wGrid = null;
		if (s.h1 == 0) {
//			lGrid = setToArr(this.lGrid,s.l1,s.l2-ort.l);
//			wGrid = setToArr(this.wGrid,s.w1,s.w2-ort.w);
			lGrid = select(this.lGrid,s.l1,s.l2-ort.l);
			wGrid = select(this.wGrid,s.w1,s.w2-ort.w);
		} else {
			lGrid = s.lGrid;
			wGrid = s.wGrid;
		}
		PairIterator iter = GridPointBySegmentTree.findGridPoints(
				s.pushProjectionH, ort.l, ort.w, lGrid, wGrid);


		Pair loc = null;
		while ((loc = iter.next()) != null) {
			Placement p = new Placement();
			p.boxType = boxType;
			p.ort = ort;
			p.releaseL = loc.x;
			p.releaseW = loc.y;
			p.releaseH = s.h1;
			p.spaceH = s.h1;
			
			// A placement to push the same location already exists
			if (pTable.get(p) != null) { continue; }

	
			// 1. 找下落过程中碰到的第一个盒子，并计算高度差
			int supportBoxIdx = searchFirstSupport(p,ort);
			p.dh = searchForDropHeight(p, supportBoxIdx);
			// 如果下落太多，就不行
			if ((!sysInfo.conf.ignoreCollision) && p.dh > ort.maxDropH) { continue; }
			// 2. 计算盒子所占用空间及盒子下落轨迹
			p.computeOccupiedAndDrop();

			// 3. 计算是否有支撑
			if (!supported(p,supportBoxIdx)) { continue; }

			
			// 4. 抓手能否图送到这个位置, 尝试两个旋转方式，任何一个方式可以都行
			// a. 按抓手的正常摆放方式推送，计算抓手的轨迹
			Vacuum gripper = sysInfo.gripper;
			Space gripperPath = gripper.gripperPathH(p.releaseL,p.releaseW,p.releaseH, 
									ort, sysInfo.inst.H, Align.org);
			if(sysInfo.conf.ignoreCollision ||
					!checkCollisionForHPush(gripperPath)) { // 已经找到可行方案
				p.gripperPath = gripperPath;   // org
				p.pushAxis = PushAxis.H;
				p.align = Align.org;
				placements.add(p);
				pTable.put(p,p);
				continue;
			}
			// b. 抓手旋转90度，计算抓手轨迹
			gripperPath = gripper.gripperPathH(p.releaseL,p.releaseW,p.releaseH, 
					ort, sysInfo.inst.H, Align.rotated);
			if (!checkCollisionForHPush(gripperPath)) {
				p.gripperPath = gripperPath;  //rotated
				p.pushAxis = PushAxis.H;
				p.align = Align.rotated;
				placements.add(p);
				pTable.put(p,p);
			}
		}
	}


	
	private LayoutST createProjectionForHPush(Space s, Orientation ort) {
		// 有一块长方形区间所在位置为x1,y1,x2,y2
		// 有n个长方形与这个区间重叠
		int count=0;
		for (int i=0; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied;
			if (curSpace.h2 <= s.h2) { break; } // 只要 curSpace.h2 <= s.h2 就一定不会阻碍推送
			if (s.projectionIntersectH(curSpace)) { count++; }
		}
		
		int[] x1=new int[count];
		int[] y1=new int[count];
		int[] x2=new int[count];
		int[] y2=new int[count];
		count=0;
		for (int i=0; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied;
			if (curSpace.h2 <= s.h2) { break; }	 // // 只要 curSpace.h2 <= s.h2 就一定不会阻碍推送
			if (s.projectionIntersectH(curSpace)) {
				x1[count] = curSpace.l1;
				y1[count] = curSpace.w1;
				x2[count] = curSpace.l2;
				y2[count] = curSpace.w2;
				count++;
			}
		}
		// 重复的投影面会被LayoutST忽略
		return new LayoutST(s.l1,s.w1,s.l2,s.w2,count,x1,y1,x2,y2);
	}

	
	// 检查已摆放的盒子与机械手的手臂在H方向走过的空间是否发生碰撞
	private boolean checkCollisionForHPush(Space gripperPath) {
		for (int i=0; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied; // 已经摆放盒子的当前space
			if (curSpace.h2 <= gripperPath.h1) { return false; } // 剩余的盒子一定不会相交
			if (gripperPath.projectionIntersectH(curSpace)) { return true;}
		}
		return false;
	}
	
	
	public boolean hasFreeSpace() {
		return this.spaceManager.getFreeSpace().size() > 0;
	}
	
	
	
	//////////////////////////////////////////
	
	public Placement[] getPlacements() {
		Placement[] result = new Placement[this.placedDescH2.length];
		int i=result.length-1;
		PlacedBox cur = this.placed;
		
		while (cur!=null) {
			result[i] = cur.p;
			cur = cur.prev;
			i -= 1;
		}
		assert i==-1;
		return result;
	}
	
	/**
	 * 沿着 this.prev 把链表中所有的pallet按倒叙找到存到数组
	 * @return
	 */
	public Pallet[] findAllPallets() {
		int count = 0;
		Pallet p = this;
		while (p != null) {
			p = p.prev;
			count++;
		}
		
		Pallet[] result = new Pallet[count];
		int i = count-1;
		p = this;
		while (p != null) {
			result[i] = p;
			p = p.prev;
			i -= 1;
		}
		return result;
	}
	
	public boolean isEmpty() { return this.loadedCount == 0; }
	
	/**
	 * 把每个可行的 placement 用一个 Mathematica notebook 问卷画出来。
	 * 最后一步是一个可行的 placement
	 * @param dir	保存生成的文件。
	 * @throws IOException
	 */
	public void drawPlacementsAndSpaces(File dir) throws IOException {
//		System.out.println("Usable spaces: "+spaceManager.getFreeSpace());
//		System.out.println("feasible placements: ");

		dir.mkdirs();
		int[] boxCount = this.spaceManager.getSpaceChecker().remainingBoxCount;
		ArrayList<ArrayList<Placement>> pList3 = computeFeasiblePalcements(boxCount);
		for (ArrayList<Placement> listForType:pList3) {
			for (int i=0; i<listForType.size(); i++) {
				Placement p = listForType.get(i);
				
//				System.out.println("  placement: "+p);
				String fileName = "placement-boxType="+p.boxType+"ort="+p.ort.ort+" r "+p.releaseL+" "+p.releaseW+" "+p.releaseH+" dh="+p.dh+" spaceH="+p.spaceH+" p="+i+".nb";
				
				Pallet newP = this.place(p);
//				System.out.println("    free space for newP: ");
//				System.out.println("    "+newP.spaceManager.getFreeSpace());
				newP.draw(new File(dir, fileName), null);
			}
		}
		
		ArrayList<Space> sapces = spaceManager.getFreeSpace();
		for (int i=0; i<sapces.size(); i++) {
			draw(new File(dir, "space-"+i+".nb"), sapces.get(i));
		}
	}
	
	/**
	 * 生成一个 Mathematica notebook 画逐步装载的过程。
	 * @param file	保存生成的 Mathematica notebook 文件
	 * @param space	如果不是 null; 会额外画一个透明空间，帮助检查剩余空间的位置。
	 * @throws IOException
	 */
	public void draw(File file, Space space) throws IOException {
		Mathematica.inst.drawToFile(file,
				this.getPlacements(), sysInfo.gripper, this.loadingSpace, space);
	}
	
	public void genDebugCode() {
		System.out.println("//////////////////////");
		System.out.println();
		System.out.println("File dir = new File(\"result/Pallet-debug\");");
		System.out.println("InstData inst = InstanceLoader.load(2, \""+sysInfo.inst.name+"\");");
		System.out.println();
		System.out.println("SysConfig sysConf = new SysConfig();");
		System.out.println("sysConf.vaccumType = Type."+sysInfo.conf.vaccumType+";");
		System.out.println("sysConf.theta = "+sysInfo.conf.theta+";");
		System.out.println("sysConf.maxDropHeightRatio = "+sysInfo.conf.maxDropHeightRatio+";");
		System.out.println("sysConf.maxHGapForSupport = "+sysInfo.conf.maxHGapForSupport+";");
		System.out.println("sysConf.minOverlapRatioForSupport = "+sysInfo.conf.minOverlapRatioForSupport+";");
		System.out.println("sysConf.knownBoxCount = "+sysInfo.conf.knownBoxCount+";");
		System.out.println("sysConf.openPalletCount = "+sysInfo.conf.openPalletCount+";");
		System.out.println("sysConf.boxCountInRange = "+sysInfo.conf.boxCountInRange+";");
		System.out.println();
		System.out.println("SystemInfo sysInfo = new SystemInfo(inst, sysConf);");
		System.out.println();

		Placement[] placements = getPlacements();
		int[] remainingCount = Arrays.copyOf(this.spaceManager.getSpaceChecker().remainingBoxCount,
				this.sysInfo.inst.getBoxTypeCount());
		for (Placement p:placements) {
			remainingCount[p.boxType] += 1;
		}
		System.out.print("int[] remainingCount = new int[] {");
		for (int t=0; t<sysInfo.inst.getBoxTypeCount(); t++) {
			if (t>0) { System.out.print(","); }
			System.out.print(remainingCount[t]);
		}
		System.out.println("};");
		System.out.println("SpaceChecker spaceChecker = new SpaceChecker(sysInfo.boxTypes, remainingCount);");
		System.out.println("Pallet P0 = new Pallet("+this.pid+", sysInfo,spaceChecker, GridSearch."+gridSearch+");");
		
		for (int i=0; i<placements.length; i++) {
			Placement p = placements[i];
			System.out.println();
			System.out.println("Placement p"+i+" = new Placement();");
			System.out.println("p"+i+".boxType = "+p.boxType+";");
			System.out.println("p"+i+".ort = sysInfo.boxTypes["+p.boxType+"].distinctOrt["+p.ort.ort+"];");
			System.out.println("p"+i+".releaseL = "+p.releaseL+";");
			System.out.println("p"+i+".releaseW = "+p.releaseW+";");
			System.out.println("p"+i+".releaseH = "+p.releaseH+";");
			System.out.println("p"+i+".dh = "+p.dh+";");
			System.out.println("p"+i+".spaceH = "+p.spaceH+";");
			System.out.println("p"+i+".pushAxis = PushAxis."+p.pushAxis+";");
			System.out.println("p"+i+".align = Align."+p.align+";");
			System.out.println("p"+i+".computePathsForDebug(sysInfo.gripper, P"+i+".loadingSpace);");
			System.out.println("Pallet P"+(i+1)+" = P"+i+".place(p"+i+");");
			System.out.println("// P"+(i+1)+".drawPlacementsAndSpaces(new File(dir, \"P"+(i+1)+"\"));");
		}
		System.out.println();
	}
		
	public static void main(String[] args) throws IOException {
		File dir = new File("result/Pallet-debug");
		InstData inst = InstanceLoader.load(2, "SD1-5-1000-small");

		SysConfig sysConf = new SysConfig();
		sysConf.vaccumType = Type.TwoByThree;
		sysConf.theta = 1.0;
		sysConf.maxDropHeightRatio = 0.1;
		sysConf.maxHGapForSupport = 0.5;
		sysConf.minOverlapRatioForSupport = 0.2;
		sysConf.knownBoxCount = 1000;
		sysConf.openPalletCount = 1;
		sysConf.boxCountInRange = 4;

		SystemInfo sysInfo = new SystemInfo(inst, sysConf);

		int[] remainingCount = new int[] {283,33,32,31,100};
		SpaceChecker spaceChecker = new SpaceChecker(sysInfo.boxTypes, remainingCount);
		Pallet P0 = new Pallet(5, sysInfo,spaceChecker, GridSearch.PushToEnd);

		Placement p0 = new Placement();
		p0.boxType = 4;
		p0.ort = sysInfo.boxTypes[4].distinctOrt[5];
		p0.releaseL = 0;
		p0.releaseW = 0;
		p0.releaseH = 0;
		p0.dh = 0;
		p0.spaceH = 0;
		p0.pushAxis = PushAxis.L;
		p0.align = Align.org;
		p0.computePathsForDebug(sysInfo.gripper, P0.loadingSpace);
		Pallet P1 = P0.place(p0);
		// P1.drawPlacementsAndSpaces(new File(dir, "P1"));

		
//		P68.debug = true;
//		ArrayList<ArrayList<Placement>> placements = P68.computeFeasiblePalcements(remainingCount);
//		for (ArrayList<Placement> lst:placements) {
//			for (Placement p:lst) {
//				
//			}
//		}
	}
}

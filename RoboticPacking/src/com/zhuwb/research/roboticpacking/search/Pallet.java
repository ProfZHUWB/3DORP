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
	public int pid; // pallet id, ��һ�� open ��Ϊ 0�� ˳����
	public SystemInfo sysInfo;
	public Space loadingSpace;
	
    // PushToEnd��  �����Σ�L-,W-push��Ѻ����Ƶ��ռ����ԭ�����
	// Grid:  L-push �ᵽ����ÿ�������ߣ����º���l1��l2�����к���l2����0һ����λ�ã�
	//        W-push �ᵽ����ÿ�������ߣ����º���w1��w2�����к���w2����0һ����λ�ã�
	// GridOnFloor:  ���� space �ĵ���߶�Ϊ 0 ʱ���� GridSearch
	public final GridSearch gridSearch; 
	
	public Pallet prev; // ���ѹرյ�pallet�б��У�����this֮ǰ��pallet; none��ʾthis�ǵ�һ��
	
	// ÿһ�ֺ��Ӷ�Ӧ�İڷŷ�ʽ
	// 1. type-->list of placement �����������ͣ��Ѽ���������п��е�placement
	// 2. type --> null �����������ͣ���û�м�����е�placement
	public  ArrayList<Placement>[] feasiblePlacements;  // [t]��ʾtype t���Ӷ�Ӧ�Ŀ���placements; null��ʾû�м����

	// �Ѿ�װ���box�б���LinkList��ʾ
	public  PlacedBox placed;			// ��ʾ�����Ѿ��ڷŵĺ��ӣ�Ҳ��ʾ�Ѱڷź����е����һ���ڷŵĺ���
	public int loadedCount = 0;		// �Ѿ�װ��this�ĺ��Ӹ���
	public int[] loadedCountPerType; 
	public double loadedVolume=0;	// �Ѿ�װ��this�ĺ��������
	
	public PlacedBox[] placedDescL1;    // �Ѿ��ڷŵĺ��Ӱ��� L1 ���꽵������
	public PlacedBox[] placedDescW1;	// �Ѿ��ڷŵĺ��Ӱ��� W1 ���꽵������
	public PlacedBox[] placedDescH1;	// �Ѿ��ڷŵĺ��Ӱ��� H1 ���꽵������
	public PlacedBox[] placedDescL2;    // �Ѿ��ڷŵĺ��Ӱ��� L2 ���꽵������
	public PlacedBox[] placedDescW2;	// �Ѿ��ڷŵĺ��Ӱ��� W2 ���꽵������
	public PlacedBox[] placedDescH2;	// �Ѿ��ڷŵĺ��Ӱ��� H2 ���꽵������
	
	// ��������ǽ��հڷŵģ���ôÿ�����ӵ�l1��Ȼ��0�����Ѿ��ڷŵĺ��ӵ�l2
	// lGrid x wGrid x hGrid ���������㣬���������н��հڷŵ�λ�� 
//	public TreeSet<Integer> lGrid;	// 0���Ѱڷŵĺ��ӵ�l2��
//	public TreeSet<Integer> wGrid;  // 0���Ѱڷŵĺ��ӵ�w2
//	public TreeSet<Integer> hGrid;  // 0���Ѱڷŵĺ��ӵ�h2
	int[] lGrid;
	int[] wGrid;
	int[] hGrid;

	// ���п��õ� Maximal Space
	public ArrayListInPlaceSpaceManager spaceManager;
	
	// ����һ���µ�pallet
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
	 * ��װָ���������Ľ����placed���뵽һ���б�����һ�����б�
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
	 * ��һ�����ӷ��� this �����µ� pallet ��ʾ������״̬
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
		
		// ���֮ǰ����Ŀ��аڷŷ�ʽ
		newP.feasiblePlacements = new ArrayList[sysInfo.inst.boxType.length];
		
		// �·ŵ� box
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
		
		// �·ŵ� box ռ���˿ռ�
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

			// ����Ƿ�support
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

			// TODO: ����Ƿ�����
		}
	}

	
	/**
	 * ����ÿ�ֺ��ӵĸ����������ڵ�ǰpallet�����п��еİڷŷ�ʽ
	 * ֻ���������������0�����ֻ࣬���ؿ���placement��������0������
	 * @param boxCount
	 * @return [i]��ʾ��i����placement�ĺ��Ӷ�Ӧ�����п���placement
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
	
	// ���ػ��߼��� type t ���Ӷ�Ӧ�Ŀ��аڷŷ�ʽ
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
					// s�������ɵ�ǰ�ĺ��ӵ���ת��ʽort
					// ����Lȥ��L���͵����п���placements
					
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
	 * ��һ�� boxType �ĺ��Ӱ�����ת��ʽ ort ���ظ��ַ�������space������λ���ҵ���
	 * 1. �����Ƶ� s ������
	 * 2. ͣ��λ��Ϊ normal position
	 * 3. û�п���֧�ţ���ץ���Ƿ�������ϰ�����
	 * @param s			Ŀ��ռ�
	 * @param boxType	���ź�������
	 * @param ort		���ź��ӵİڷŷ�ʽ
	 * @param placements	�����ҵ���placement
	 * @param pTable		�����ҵ���placement
	 */
	protected void findPlacementsLPush(Space s, int boxType, Orientation ort, ArrayList<Placement> placements, HashMap<Placement, Placement> pTable) {
		if ((!sysInfo.conf.ignoreCollision) && !sysInfo.gripper.canLPush(ort)) { return; }

		// ��ͼ�ҳ����п��԰Ѻ��ӿ����Ƶ���λ��
		// ����L-push�¿��ÿռ�s���п��Է�ort��λ��
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
	
				// 1. ����������������ĵ�һ�����ӣ�������߶Ȳ�
				int supportBoxIdx = searchFirstSupport(p,ort);
				p.dh = searchForDropHeight(p, supportBoxIdx);	
				
				// �������̫�࣬�Ͳ���
				if ((!sysInfo.conf.ignoreCollision) && p.dh > ort.maxDropH) { continue; }
				// 2. ���������ռ�ÿռ估��������켣
				p.computeOccupiedAndDrop();
	
				// 3. �����Ƿ���֧��
				if (!supported(p,supportBoxIdx)) { continue; }
	
				// 4. ץ���ܷ�ͼ�͵����λ��, ����������ת��ʽ���κ�һ����ʽ���Զ���
				// a. ��ץ�ֵ������ڷŷ�ʽ���ͣ�����ץ�ֵĹ켣
				Vacuum gripper = sysInfo.gripper;
				Space gripperPath = gripper.gripperPathL(p.releaseL,p.releaseW,p.releaseH, 
										ort, sysInfo.inst.L, Align.org);
				// ץ�ֵ��±߽����������̵ĵ�����ץ�ֹ켣�����к��Ӳ���ײ
				if(sysInfo.conf.ignoreCollision ||
						gripperPath.h1 >= 0 && !checkCollisionForLPush(gripperPath)) { // �Ѿ��ҵ����з���					
					p.gripperPath = gripperPath;   // org
					p.pushAxis = PushAxis.L;
					p.align = Align.org;
					placements.add(p);
					pTable.put(p,p);
					continue;
				}
				// b. ץ����ת90�ȣ�����ץ�ֹ켣
				gripperPath = gripper.gripperPathL(p.releaseL,p.releaseW,p.releaseH, 
						ort, sysInfo.inst.L, Align.rotated);
				// ץ�ֵ��±߽����������̵ĵ�����ץ�ֹ켣�����к��Ӳ���ײ
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
		//��h����ͶӰ����Space�ཻ�Ҹ߶��ں��ʷ�Χ֮�ڵ��ѷź��ӵ��ϱ�����Ǳ��֧��ort�ģ���������������ڷ�ort�Ŀ���L��W����
		int maxH2 = s.h2 - ort.h;   // �ϱ��泬��maxH2�ĺ��Ӳ�����֧��ort
		int minH2 = s.h1 - ort.maxDropH;  // �ϱ������minH2�ĺ��Ӳ�����֧��ort
		
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
	 * �ҵ����п����赲�º�����j�����ռ�s���Ѱڷź��ӵ�ͶӰ�����Ҽ�����ܳ�Ϊ�º�������λ�õĵ�H��ֵ�����������s.hCandidateForLPush��
	 * @param s
	 * @return
	 */
	private LayoutST createProjectionAndHCandidateForLPush(Space s) { //{, Orientation ort) {		
		IntegerDynamicTable hTable = new IntegerDynamicTable(10); // �赲���ӵ��ϱ߽������ǹ��ĵ����͵ĺ��ӵĿ���h����,���doc/2021-05-20-1��-2
		hTable.append(s.h1);

		boolean print = false;
//		if (debug && s.l1 == 0 && s.w1 == 51 && s.h1 == 65 && s.l2 == 24 && s.w2 == 100 && s.h2 == 150) {
//			System.out.println("xxxxxxxxxxxxx");
//			print = true;
//		}
				
		// ��һ�鳤������������λ��Ϊx1,y1,x2,y2
		// ��n������������������ص�
		int count=0;
		for (int i=0; i<this.placedDescL2.length; i++) {
			Space curSpace = this.placedDescL2[i].occupied;
			
//			if (print && curSpace.l1 == 24 && curSpace.w1 == 38 && curSpace.h1 == 51 &&
//					curSpace.l2 == 48 && curSpace.w2 == 53 && curSpace.h2 == 69) {
//				System.out.println("curSpace.l2 <= s.l2? "+(curSpace.l2 <= s.l2));
//			}
			
			if (curSpace.l2 <= s.l2) { break; }  // ֻҪ curSpace.l2 <= s.l2 ��һ�������谭����
			if (s.projectionIntersectL(curSpace)) { 
				count++; 
				
//				if (print && curSpace.l1 == 24 && curSpace.w1 == 38 && curSpace.h1 == 51 &&
//						curSpace.l2 == 48 && curSpace.w2 == 53 && curSpace.h2 == 69) {
//					System.out.println("?????curSpace.l2 <= s.l2? "+(curSpace.l2 <= s.l2));
//				}
				
				//if (curSpace.h2+ort.h <= s.h2 && curSpace.h2 <= s.h1 + ort.maxDropH) { // ���䳬�� maxDropH ��λ�ò�����
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
			if (curSpace.l2 <= s.l2) { break; }  // ֻҪ curSpace.l2 <= s.l2 ��һ�������谭����
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
		// �ظ���ͶӰ��ᱻLayoutST����
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
	 * �ڴ� p.releaseH ��ʼ�����ҵ� ort ������������ĸ߶�
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
	// �� p.releaseL, p.releaseW, p.releaseH ���������������ĵ�һ���ӵ��ϱ���
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
	 * ���� occupied �Ƿ�����ĺ���֧��
	 * @param p
	 * @return
	 */
	private boolean supported(Placement p, int firstSupportBoxIdx) {
		if (firstSupportBoxIdx == this.placedDescH2.length) { return true; } // �ذ�һ������֧��
		
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
	

	
	// ����Ѱڷŵĺ������е�ֵ��ֱ���L�����߹��Ŀռ��Ƿ�����ײ
	private boolean checkCollisionForLPush(Space gripperPath) {
		for (int i=0; i<this.placedDescL2.length; i++) {
			Space curSpace = this.placedDescL2[i].occupied; // �Ѿ��ڷź��ӵĵ�ǰspace
			if (curSpace.l2 <= gripperPath.l1) { return false; } // ʣ��ĺ���һ�������ཻ
			if (gripperPath.projectionIntersectL(curSpace)) { return true;}
		}
		return false;
	}
	
	
	
	
	
	/**
	 * ��һ�� boxType �ĺ��Ӱ�����ת��ʽ ort ���ظ��ַ�������space������λ���ҵ���
	 * 1. �����Ƶ� s ������
	 * 2. ͣ��λ��Ϊ normal position
	 * 3. û�п���֧�ţ���ץ���Ƿ�������ϰ�����
	 * @param s			Ŀ��ռ�
	 * @param boxType	���ź�������
	 * @param ort		���ź��ӵİڷŷ�ʽ
	 * @param placements	�����ҵ���placement
	 * @param pTable		�����ҵ���placement
	 */
	protected void findPlacementsWPush(Space s, int boxType, Orientation ort, ArrayList<Placement> placements, HashMap<Placement, Placement> pTable) {
		if ((!sysInfo.conf.ignoreCollision) && !sysInfo.gripper.canWPush(ort)) { return; }

		// ��ͼ�ҳ����п��԰Ѻ��ӿ����Ƶ���λ��
		// ����W-push�¿��ÿռ�s���п��Է�ort��λ��
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
	
		
				// 1. ����������������ĵ�һ�����ӣ�������߶Ȳ�
				int supportBoxIdx = searchFirstSupport(p,ort);
				p.dh = searchForDropHeight(p, supportBoxIdx);
				// �������̫�࣬�Ͳ���
				if ((!sysInfo.conf.ignoreCollision) && p.dh > ort.maxDropH) { continue; }
				// 2. ���������ռ�ÿռ估��������켣
				p.computeOccupiedAndDrop();
	
				// 3. �����Ƿ���֧��
				if (!supported(p,supportBoxIdx)) { continue; }
	
				
				// 4. ץ���ܷ�ͼ�͵����λ��, ����������ת��ʽ���κ�һ����ʽ���Զ���
				// a. ��ץ�ֵ������ڷŷ�ʽ���ͣ�����ץ�ֵĹ켣
				Vacuum gripper = sysInfo.gripper;
				Space gripperPath = gripper.gripperPathW(p.releaseL,p.releaseW,p.releaseH, 
										ort, sysInfo.inst.W, Align.org);
				// ץ�ֵ��±߽����������̵ĵ�����ץ�ֹ켣�����к��Ӳ���ײ
				if (sysInfo.conf.ignoreCollision ||
						gripperPath.h1 >= 0 && !checkCollisionForWPush(gripperPath)) { // �Ѿ��ҵ����з���
					p.gripperPath = gripperPath;   // org
					p.pushAxis = PushAxis.W;
					p.align = Align.org;
					placements.add(p);
					pTable.put(p,p);
					continue;
				}
				// b. ץ����ת90�ȣ�����ץ�ֹ켣
				gripperPath = gripper.gripperPathW(p.releaseL,p.releaseW,p.releaseH, 
						ort, sysInfo.inst.W, Align.rotated);
				// ץ�ֵ��±߽����������̵ĵ�����ץ�ֹ켣�����к��Ӳ���ײ
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
		//��h����ͶӰ����Space�ཻ�Ҹ߶��ں��ʷ�Χ֮�ڵ��ѷź��ӵ��ϱ�����Ǳ��֧��ort�ģ���������������ڷ�ort�Ŀ���L��W����
		
		IntegerDynamicTable hTable = new IntegerDynamicTable(10);
		hTable.append(s.h1);
		
		// ��һ�鳤������������λ��Ϊx1,y1,x2,y2
		// ��n������������������ص�
		int count=0;
		for (int i=0; i<this.placedDescW2.length; i++) {
			Space curSpace = this.placedDescW2[i].occupied;
			if (curSpace.w2 <= s.w2) { break; } // ֻҪ curSpace.w2 <= s.w2 ��һ�������谭����
			if (s.projectionIntersectW(curSpace)) { 
				count++;
				
//				if (curSpace.h2+ort.h <= s.h2 && curSpace.h2 <= s.h1 + ort.maxDropH) { // ���䳬�� maxDropH ��λ�ò�����
				if (curSpace.h2 < s.h2) { // ���䳬�� maxDropH ��λ�ò�����
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
			if (curSpace.w2 <= s.w2) { break; } // ֻҪ curSpace.w2 <= s.w2 ��һ�������谭����
			if (s.projectionIntersectW(curSpace)) {
				x1[count] = curSpace.l1;
				y1[count] = curSpace.h1;
				x2[count] = curSpace.l2;
				y2[count] = curSpace.h2;
				count++;
			}
		}
		// �ظ���ͶӰ��ᱻLayoutST����
		return new LayoutST(s.l1,s.h1,s.l2,s.h2,count,x1,y1,x2,y2);
	}
	
	// ����Ѱڷŵĺ������е�ֵ��ֱ���W�����߹��Ŀռ��Ƿ�����ײ
	private boolean checkCollisionForWPush(Space gripperPath) {
		for (int i=0; i<this.placedDescW2.length; i++) {
			Space curSpace = this.placedDescW2[i].occupied; // �Ѿ��ڷź��ӵĵ�ǰspace
			if (curSpace.w2 <= gripperPath.w1) { return false; } // ʣ��ĺ���һ�������ཻ
			if (gripperPath.projectionIntersectW(curSpace)) { return true;}
		}
		return false;
	}

	
	
	
	/**
	 * ��һ�� boxType �ĺ��Ӱ�����ת��ʽ ort ���ظ��ַ�������space������λ���ҵ���
	 * 1. �����Ƶ� s ������
	 * 2. ͣ��λ��Ϊ normal position
	 * 3. û�п���֧�ţ���ץ���Ƿ�������ϰ�����
	 * @param s			Ŀ��ռ�
	 * @param boxType	���ź�������
	 * @param ort		���ź��ӵİڷŷ�ʽ
	 * @param placements	�����ҵ���placement
	 * @param pTable		�����ҵ���placement
	 */
	protected void findPlacementsHPush(Space s, int boxType, Orientation ort, ArrayList<Placement> placements, HashMap<Placement, Placement> pTable) {
		if ((!sysInfo.conf.ignoreCollision) && !sysInfo.gripper.canHPush(ort)) { return; }

		// ��ͼ�ҳ����п��԰Ѻ��ӿ����Ƶ���λ��
		// ����H-push�¿��ÿռ�s���п��Է�ort��λ��
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

	
			// 1. ����������������ĵ�һ�����ӣ�������߶Ȳ�
			int supportBoxIdx = searchFirstSupport(p,ort);
			p.dh = searchForDropHeight(p, supportBoxIdx);
			// �������̫�࣬�Ͳ���
			if ((!sysInfo.conf.ignoreCollision) && p.dh > ort.maxDropH) { continue; }
			// 2. ���������ռ�ÿռ估��������켣
			p.computeOccupiedAndDrop();

			// 3. �����Ƿ���֧��
			if (!supported(p,supportBoxIdx)) { continue; }

			
			// 4. ץ���ܷ�ͼ�͵����λ��, ����������ת��ʽ���κ�һ����ʽ���Զ���
			// a. ��ץ�ֵ������ڷŷ�ʽ���ͣ�����ץ�ֵĹ켣
			Vacuum gripper = sysInfo.gripper;
			Space gripperPath = gripper.gripperPathH(p.releaseL,p.releaseW,p.releaseH, 
									ort, sysInfo.inst.H, Align.org);
			if(sysInfo.conf.ignoreCollision ||
					!checkCollisionForHPush(gripperPath)) { // �Ѿ��ҵ����з���
				p.gripperPath = gripperPath;   // org
				p.pushAxis = PushAxis.H;
				p.align = Align.org;
				placements.add(p);
				pTable.put(p,p);
				continue;
			}
			// b. ץ����ת90�ȣ�����ץ�ֹ켣
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
		// ��һ�鳤������������λ��Ϊx1,y1,x2,y2
		// ��n������������������ص�
		int count=0;
		for (int i=0; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied;
			if (curSpace.h2 <= s.h2) { break; } // ֻҪ curSpace.h2 <= s.h2 ��һ�������谭����
			if (s.projectionIntersectH(curSpace)) { count++; }
		}
		
		int[] x1=new int[count];
		int[] y1=new int[count];
		int[] x2=new int[count];
		int[] y2=new int[count];
		count=0;
		for (int i=0; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied;
			if (curSpace.h2 <= s.h2) { break; }	 // // ֻҪ curSpace.h2 <= s.h2 ��һ�������谭����
			if (s.projectionIntersectH(curSpace)) {
				x1[count] = curSpace.l1;
				y1[count] = curSpace.w1;
				x2[count] = curSpace.l2;
				y2[count] = curSpace.w2;
				count++;
			}
		}
		// �ظ���ͶӰ��ᱻLayoutST����
		return new LayoutST(s.l1,s.w1,s.l2,s.w2,count,x1,y1,x2,y2);
	}

	
	// ����Ѱڷŵĺ������е�ֵ��ֱ���H�����߹��Ŀռ��Ƿ�����ײ
	private boolean checkCollisionForHPush(Space gripperPath) {
		for (int i=0; i<this.placedDescH2.length; i++) {
			Space curSpace = this.placedDescH2[i].occupied; // �Ѿ��ڷź��ӵĵ�ǰspace
			if (curSpace.h2 <= gripperPath.h1) { return false; } // ʣ��ĺ���һ�������ཻ
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
	 * ���� this.prev �����������е�pallet�������ҵ��浽����
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
	 * ��ÿ�����е� placement ��һ�� Mathematica notebook �ʾ�������
	 * ���һ����һ�����е� placement
	 * @param dir	�������ɵ��ļ���
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
	 * ����һ�� Mathematica notebook ����װ�صĹ��̡�
	 * @param file	�������ɵ� Mathematica notebook �ļ�
	 * @param space	������� null; ����⻭һ��͸���ռ䣬�������ʣ��ռ��λ�á�
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

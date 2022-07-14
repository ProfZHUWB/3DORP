package com.zhuwb.research.roboticpacking.search;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import org.junit.jupiter.api.Test;

import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.Placement;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.SystemInfo;
import com.zhuwb.research.roboticpacking.inst.Vacuum.Align;
import com.zhuwb.research.roboticpacking.inst.Vacuum.PushAxis;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.space.ArrayListInPlaceSpaceManager.SpaceChecker;
import com.zhuwb.research.roboticpacking.space.Space;

public class PalletTest {

//	@Test
//	void testSpaceManager1() throws IOException {
//		InstData inst = InstanceLoader.load(2,"SD0-2-1000-large-2");
////		System.out.println(inst.name);
//		SysConfig sysConf = new SysConfig();
//		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
////		System.out.println("box types:");
////		System.out.println(Arrays.toString(sysInfo.boxTypes));
//
//		int[] remainingCount = new int[] {1,1};
//		SpaceChecker spaceChecker = new SpaceChecker(sysInfo.boxTypes, remainingCount);
//		
//		Pallet P0 = new Pallet(0, sysInfo,spaceChecker, GridSearch.UseGrid);
////		System.out.println("Usable spaces: "+P0.spaceManager.getFreeSpace());
//	
//		Placement p1 = new Placement();
//		p1.boxType = 0;
//		p1.ort = sysInfo.boxTypes[p1.boxType].distinctOrt[0];
//		p1.releaseL = 0;
//		p1.releaseW = 0;
//		p1.releaseH = 2;
//		p1.dh = 2;
//		
//		p1.pushAxis = PushAxis.L;
//		p1.align = Align.org;
//		p1.computePathsForDebug(sysInfo.gripper, P0.loadingSpace);
//		
//		// 更新pallet的第二个状态
//		Pallet P1 = P0.place(p1);
//		assertEquals(3, P1.spaceManager.getFreeSpace().size());
////		System.out.println(P1.spaceManager.getFreeSpace());
//		
//		Placement p2 = new Placement(); // 第二个placement状态
//		p2.boxType = 1;
//		p2.ort = sysInfo.boxTypes[p1.boxType].distinctOrt[0];
//		p2.releaseL = 0;
//		p2.releaseW = 0;
//		p2.releaseH = 22;
//		p2.dh = 2;
//		
//		p2.pushAxis = PushAxis.H;
//		p2.align = Align.rotated;
//		p2.computePathsForDebug(sysInfo.gripper, P1.loadingSpace);
//
//		Pallet P2 = P1.place(p2);
//		assertEquals(0, P2.spaceManager.getFreeSpace().size()); // 因为没有盒子了,所有空间都没法利用
////		System.out.println(P2.spaceManager.getFreeSpace());	
//	}
	
	static class SpaceComparator implements Comparator<Space> {
		@Override
		public int compare(Space o1, Space o2) {
			if (o1.l1 < o2.l1) { return -1; }
			if (o1.l1 > o2.l1) { return 1; }
			if (o1.w1 < o2.w1) { return -1; }
			if (o1.w1 > o2.w1) { return 1; }
			if (o1.h1 < o2.h1) { return -1; }
			if (o1.h1 > o2.h1) { return 1; }
			if (o1.l2 < o2.l2) { return -1; }
			if (o1.l2 > o2.l2) { return 1; }
			if (o1.w2 < o2.w2) { return -1; }
			if (o1.w2 > o2.w2) { return 1; }
			if (o1.h2 < o2.h2) { return -1; }
			if (o1.h2 > o2.h2) { return 1; }
			return 0;
		}
		
		static SpaceComparator inst = new SpaceComparator();
	}
	
	void assertSpaceEquals(Space[] expected, ArrayList<Space> actual) {
		assertEquals(expected.length, actual.size());
		
		Arrays.sort(expected, SpaceComparator.inst);
		Collections.sort(actual, SpaceComparator.inst);
		for (int i=0; i<expected.length; i++) {
			assertEquals(expected[i], actual.get(i));
		}
	}

	@Test
	void testSpaceManager2() {
		int boxCount = 4;
		InstData inst = new InstData(120, 100, 150, 1, boxCount);
		inst.boxType = new int[][] {
			{55,50,40}
		};
		inst.ortPerm = new boolean[][] {{true,true,true,true,true,true}};
		inst.t = new int[] {0,0,0};
		inst.ort = new int[] {0,0,0};
		inst.name = "temp-inst";
		
		SysConfig sysConf = new SysConfig();
		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
//		System.out.println("box types:");
//		System.out.println(Arrays.toString(sysInfo.boxTypes));

		int[] remainingCount = new int[] {boxCount};
		SpaceChecker spaceChecker = new SpaceChecker(sysInfo.boxTypes, remainingCount);
		
		Pallet P0 = new Pallet(0, sysInfo,spaceChecker, GridSearch.UseGrid);
		assertSpaceEquals(new Space[] {
				new Space(0,0,0,120,100,150)
			}, P0.spaceManager.getFreeSpace());
//		System.out.println("Usable spaces: "+P0.spaceManager.getFreeSpace());
	
		Placement p1 = new Placement();
		p1.boxType = 0;
		p1.ort = sysInfo.boxTypes[p1.boxType].distinctOrt[0];
		p1.releaseL = 0;
		p1.releaseW = 0;
		p1.releaseH = 2;
		p1.dh = 2;
		
		p1.pushAxis = PushAxis.L;
		p1.align = Align.org;
		p1.computePathsForDebug(sysInfo.gripper, P0.loadingSpace);
				
		// 更新pallet的第二个状态
		Pallet P1 = P0.place(p1);
		assertSpaceEquals(new Space[] {
				new Space(55,0,0,120,100,150),
				new Space(0,0,40,120,100,150),
				new Space(0,50,0,120,100,150)
			}, P1.spaceManager.getFreeSpace());
//		System.out.println(P1.spaceManager.getFreeSpace());
				
		Placement p2 = new Placement(); // 第二个placement状态
		p2.boxType = 0;
		p2.ort = sysInfo.boxTypes[p1.boxType].distinctOrt[0];
		p2.releaseL = 60;
		p2.releaseW = 0;
		p2.releaseH = 5;
		p2.dh = 5;
		
		p2.pushAxis = PushAxis.H;
		p2.align = Align.rotated;
		p2.computePathsForDebug(sysInfo.gripper, P1.loadingSpace);

		Pallet P2 = P1.place(p2);
		assertSpaceEquals(new Space[] {
				new Space(0,0,40,120,100,150),
				new Space(0,50,0,120,100,150)
			}, P2.spaceManager.getFreeSpace());
//		System.out.println(P2.spaceManager.getFreeSpace());
		

		Placement p3 = new Placement(); // 第二个placement状态
		p3.boxType = 0;
		p3.ort = sysInfo.boxTypes[p1.boxType].distinctOrt[1];
		p3.releaseL = 0;
		p3.releaseW = 50;
		p3.releaseH = 0;
		p3.dh = 0;
		
		p3.pushAxis = PushAxis.W;
		p3.align = Align.org;
		p3.computePathsForDebug(sysInfo.gripper, P2.loadingSpace);
		

		Pallet P3 = P2.place(p3);
		assertSpaceEquals(new Space[] {
				new Space(0,0,50,120,100,150),
				new Space(55,0,40,120,100,150),
				new Space(0,0,40,120,50,150),
				new Space(55,50,0,120,100,150)
			}, P3.spaceManager.getFreeSpace());
//		System.out.println(P3.spaceManager.getFreeSpace());
//		System.out.println(Mathematica.inst.draw(P3.getPlacements(), sysInfo.gripper, sysInfo.inst.L, sysInfo.inst.W, sysInfo.inst.H));
	}
}

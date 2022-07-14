package com.zhuwb.research.roboticpacking.inst;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import com.zhuwb.research.roboticpacking.inst.Vacuum.Align;
import com.zhuwb.research.roboticpacking.inst.Vacuum.PushAxis;
import com.zhuwb.research.roboticpacking.search.AlgoConfig.GridSearch;
import com.zhuwb.research.roboticpacking.search.Pallet;
import com.zhuwb.research.roboticpacking.space.ArrayListInPlaceSpaceManager.SpaceChecker;
import com.zhuwb.research.roboticpacking.space.Space;

public class PlacementTest {

	@Test
	void testPath1() throws IOException {
		InstData inst = InstanceLoader.load(2,"SD0-2-1000-large-2");
//		System.out.println(inst.name);
		SysConfig sysConf = new SysConfig();
		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
		
		int[] remainingCount = new int[] {1,1};
		SpaceChecker spaceChecker = new SpaceChecker(sysInfo.boxTypes, remainingCount);
				
		Pallet P0 = new Pallet(0, sysInfo, spaceChecker, GridSearch.UseGrid);
		Space palletSpace = new Space(0, 0, 0, 120, 100, 150);
		assertEquals(1,P0.spaceManager.getFreeSpace().size());
		assertEquals(palletSpace,P0.spaceManager.getFreeSpace().get(0));

		
		Placement p1 = new Placement();
		p1.boxType = 0;
		p1.ort = sysInfo.boxTypes[p1.boxType].distinctOrt[0];
		p1.releaseL = 0;
		p1.releaseW = 0;
		p1.releaseH = 2;
		p1.dh = 2;
		
		p1.pushAxis = PushAxis.L;
		p1.align = Align.org;
		Space pallet = new Space(0,0,0, sysInfo.inst.L, sysInfo.inst.W, sysInfo.inst.H);
		p1.computePathsForDebug(sysInfo.gripper, pallet);
		
		assertEquals(new Space(0,0,0,18,18,20), p1.occupied);
		assertEquals(new Space(0,0,20,18,18,22), p1.dropPath);
		assertEquals(new Space(18,0,2,120,18,22), p1.boxPath);
		assertEquals(new Space(18,-12,7,120,38,42), p1.gripperPath);
				
//		System.out.println("placement p1:");
//		p1.print("    ", System.out);
	}

	
	@Test
	void testPath2() throws IOException {
		InstData inst = InstanceLoader.load(2,"SD0-2-1000-large-2");
		System.out.println(inst.name);
		SysConfig sysConf = new SysConfig();
		SystemInfo sysInfo = new SystemInfo(inst, sysConf);
		
		Placement p2 = new Placement(); // µÚ¶þ¸öplacement×´Ì¬
		p2.boxType = 0;
		p2.ort = sysInfo.boxTypes[p2.boxType].distinctOrt[0];
		p2.releaseL = 0;
		p2.releaseW = 0;
		p2.releaseH = 22;
		p2.dh = 2;
		
		p2.pushAxis = PushAxis.H;
		p2.align = Align.rotated;
		Space pallet = new Space(0,0,0, sysInfo.inst.L, sysInfo.inst.W, sysInfo.inst.H);
		p2.computePathsForDebug(sysInfo.gripper, pallet);

		assertEquals(new Space(0,0,20,18,18,40), p2.occupied);
		assertEquals(new Space(0,0,40,18,18,42), p2.dropPath);
		assertEquals(new Space(0,0,42,18,18,150), p2.boxPath);
		assertEquals(new Space(3,-12,42,38,38,150), p2.gripperPath);
		
//		System.out.println("placement p2:");
//		p2.print("    ", System.out);
	}
}

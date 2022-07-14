package com.zhuwb.research.roboticpacking.inst;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;
import com.zhuwb.research.roboticpacking.inst.Vacuum.Type;
import com.zhuwb.research.roboticpacking.space.Space;

public class VacuumTest {

	@BeforeAll
	static void setUpBeforeClass() throws Exception {
	}
	
	void assertOrientationDim(Orientation[] distinctOrt, int[][] dim, int[] maxDropH) {
		assertEquals(dim.length,distinctOrt.length);
		for (int ortIdx=0; ortIdx<dim.length; ortIdx++) {
			Orientation ort = distinctOrt[ortIdx];
			int[] lwh = dim[ortIdx];
			assertEquals(lwh[0],ort.l);
			assertEquals(lwh[1],ort.w);
			assertEquals(lwh[2],ort.h);
			assertEquals(maxDropH[ortIdx], ort.maxDropH);
		}
	}
	
	@Test
	void testBoxType1() {
		Vacuum gripper = Vacuum.create(1.0, Type.TwoByThree);
		BoxType bt = new BoxType(0, 41, 26, 10, new boolean[] {true,true,true,true,true,true}, gripper, 0.1);
		int[][] dim = new int[][] {
			{41,26,10},
			{41,10,26},
			{26,41,10},
			{26,10,41},
			{10,41,26},
			{10,26,41}};
		assertOrientationDim(bt.distinctOrt,dim, new int[] {1,2,1,4,2,4});
	}
	
	@Test
	void testBoxType2() {
		Vacuum gripper = Vacuum.create(1.0, Type.TwoByThree);
		BoxType bt = new BoxType(0, 41, 26, 26, new boolean[] {true,true,true,true,true,true}, gripper, 0.1);
		int[][] dim = new int[][] {
			{41,26,26},
			{26,41,26},
			{26,26,41}};
		assertOrientationDim(bt.distinctOrt,dim,new int[] {2,2,4});
	}

	@Test
	void testBoxType3() {
		Vacuum gripper = Vacuum.create(1.0, Type.TwoByThree);
		BoxType bt = new BoxType(0, 26, 26, 26, new boolean[] {true,true,true,true,true,true}, gripper, 0.1);
		int[][] dim = new int[][] {
			{26,26,26}};
		assertOrientationDim(bt.distinctOrt,dim,new int[] {2});
	}

	@Test
	void testBoxType4() {
		Vacuum gripper = Vacuum.create(1.0, Type.TwoByThree);
		BoxType bt = new BoxType(0, 41, 26, 10, new boolean[] {true,false,false,true,true,true}, gripper, 0.1);
		int[][] dim = new int[][] {
			{41,26,10},
//			{41,10,26},
//			{26,41,10},
			{26,10,41},
			{10,41,26},
			{10,26,41}};
		assertOrientationDim(bt.distinctOrt,dim,new int[] {1,4,2,4});
	}

	@Test
	void computMaxOffSet1() {
		double dL = Vacuum.computMaxOffSet(41, 50, 5, 3);
		assertEquals(0.5, dL);
		dL = Vacuum.computMaxOffSet(41, 35, 5, 2);
		assertEquals(8.0, dL);
		
		dL = Vacuum.computMaxOffSet(26, 50, 5, 3);
		assertEquals(8.0, dL);
		dL = Vacuum.computMaxOffSet(26, 35, 5, 2);
		assertEquals(0.5, dL);
		
		dL = Vacuum.computMaxOffSet(10, 50, 5, 3);
		assertEquals(0, dL);
		dL = Vacuum.computMaxOffSet(10, 35, 5, 2);
		assertEquals(7.5, dL);
	}

	
	public void assertDoubleA2(double[][] expected, double[][] actual) {
		for (int mode=0; mode<6; mode++) {
			for (int axis=0; axis<3; axis++) {
				assertEquals(expected[mode][axis],actual[mode][axis]);
			}
		}
	}
	
	@Test
	void testComputeGripperProjection1() {
		Vacuum gripper = Vacuum.create(1.0, Type.TwoByThree);
		Orientation ort = new Orientation(41, 26, 10, 0, 0.1);
		ort.gripperProjection = gripper.computeGripperProjection(ort);
		assertEquals(new Space(41,-4, -5,41,46,30), ort.gripperProjection[0][0]);
		assertEquals(new Space(41,-4,-20,41,31,30), ort.gripperProjection[0][1]);
		assertEquals(new Space(-4,26, -5,46,26,30), ort.gripperProjection[1][0]);
		assertEquals(new Space(11,26,-20,46,26,30), ort.gripperProjection[1][1]);
		assertEquals(new Space(-4,-4, 10,46,31,10), ort.gripperProjection[2][0]);
		assertEquals(new Space(11,-4, 10,46,46,10), ort.gripperProjection[2][1]);
	}

//	@Test
//	void testComputeMaxOffsetsForBox1() {
//		Vacuum v23 = Vacuum.create(1.0, Type.TwoByThree);
////		System.out.println(v23);
//		int[] b = new int[] {41, 26, 10};
//		
//		double[][] maxOffset = v23.computeMaxOffsetsForBox(b[0],b[1],b[2]);
////		for (double[]dm:maxOffset) {
////			System.out.println(Arrays.toString(dm));
////		}
//		
//		double[][] expected = new double[][] {
//			{0.5, 0.5, Double.NEGATIVE_INFINITY},
//			{8.0, 8.0, Double.NEGATIVE_INFINITY},
//			{Double.NEGATIVE_INFINITY, 8.0, 2.5},
//			{Double.NEGATIVE_INFINITY, 0.5, 0.0},
//			{0.5, Double.NEGATIVE_INFINITY, 2.5},
//			{8.0, Double.NEGATIVE_INFINITY, 0.0}};
//		
//		assertDoubleA2(expected, maxOffset);
//	}
//
//	@Test
//	void testComputeMaxOffsetsForBox2() {
//		Vacuum v23 = Vacuum.create(1.0, Type.TwoByThree);
////		System.out.println(v23);
//		int[] b = new int[] {41, 26, 9};
//		
//		try {
//			v23.computeMaxOffsetsForBox(b[0],b[1],b[2]);
//			fail();
//		} catch (IllegalArgumentException e) {
//		}		
//	}
//
//	@Test
//	void testComputeMaxOffsetsForBox3() {
//		Vacuum v23 = Vacuum.create(1.0, Type.TwoByThree);
////		System.out.println(v23);
//		int[] b = new int[] {42, 26, 10};
//		
//		double[][] maxOffset = v23.computeMaxOffsetsForBox(b[0],b[1],b[2]);
////		for (double[]dm:maxOffset) {
////			System.out.println(Arrays.toString(dm));
////		}
//		
//		double[][] expected = new double[][] {
//			{1.0, 0.5, Double.NEGATIVE_INFINITY},
//			{8.5, 8.0, Double.NEGATIVE_INFINITY},
//			{Double.NEGATIVE_INFINITY, 8.0, 2.5},
//			{Double.NEGATIVE_INFINITY, 0.5, 0.0},
//			{1.0, Double.NEGATIVE_INFINITY, 2.5},
//			{8.5, Double.NEGATIVE_INFINITY, 0.0}};
//		
//		assertDoubleA2(expected, maxOffset);
//	}
}

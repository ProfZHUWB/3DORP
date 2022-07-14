package com.zhuwb.research.roboticpacking.inst;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import com.google.gson.Gson;
import com.zhuwb.research.roboticpacking.inst.InstConfig.Distribution;
import com.zhuwb.research.roboticpacking.inst.InstConfig.Regularity;

public class Generator {
	
	/**
	 * @param rand 随机数
	 * @param L			托盘L
	 * @param W			托盘W
	 * @param palletCount 托盘数量
	 * @param boxTypeCount box种类数量
	 * @param boxTypeRegularity     种类的规整度
	 * @param boxTypeDistribution pro[i] 是第 i 种box的概率
	 * @param boxCountPerType box数量
	 * @param vaccumType 真空吸盘规格: 2x3 或 2x2
	 * @param theta      真空吸盘规格缩放比例
	 * @return instance 算例序列
	 */
	@SuppressWarnings("resource")
	public static InstData generate(Random rand, InstConfig conf) {
		int start = rand.nextInt(boxTypeSF.length-conf.boxTypeCount+1);//随机挑选连续种类的起点
		
		InstData inst = new InstData(conf.L, conf.W, conf.H, conf.boxTypeCount, conf.boxCount);		
		inst.name = conf.boxTypeRegularity+"-"+conf.boxTypeCount+"-"+conf.boxCount+"-"+conf.dist;

		
		int[][] boxtype = pickReg(rand,conf.boxTypeRegularity);//挑选出对应种类box的三维信息
		inst.boxType = Arrays.copyOfRange(boxtype, start, start+conf.boxTypeCount);
		for (int t=0; t<conf.boxTypeCount; t++) {
			inst.ortPerm[t] = new boolean[] {true,true,true,true,true,true};
		}
		
		// 按比例分布计算每种盒子的个数
		int[] boxCountPerType = genDistribuiton(conf.boxCount, conf.dist, conf.boxTypeCount);
		// 按每种数量生成盒子的到达顺序
		ArrayList<Integer> boxType = new ArrayList<>();
		for (int t=0; t<conf.boxTypeCount; t++) {
			for (int i=0; i<boxCountPerType[t]; i++) {
				boxType.add(t);
			}
		}
		// 随机打乱到达顺序
		Collections.shuffle(boxType, rand);
		
		for (int i=0; i<boxType.size(); i++) {
			inst.t[i] = boxType.get(i);
			inst.ort[i] = 0;
		}
		
//		bf.close();
		return inst;
	}
	
	
	/**
	 * 随机生成一种box，它的前两个维度在 [min, max] 之间，总体积接近但不超过 V，
	 * 第3个维度不超过 maxD
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxType(Random rand, int min, int max, long V, int minD, int maxD) {
		int range = max - min + 1;
		while (true) {
			int L = rand.nextInt(range) + min;
			int W = rand.nextInt(range) + min;
			int H = (int)(V / (L * W));
			if (minD < H && H <= maxD) {
				return new int[] {L,W,H};
			}
		}
	}
	
	/**
	 * 判断 L 是否与 dim 中的任一维度相同
	 * @param L
	 * @param dim
	 * @return
	 */
	private static boolean equalAnySide(int L, int[] dim) {
		for (int i=0; i<dim.length; i++) {
			if (L == dim[i]) { return true; }
		}
		return false;
	}
	
	static final int MAX_ATTEMPT = 10000;

	/**
	 * 随机生成一种box，它的一个维度与 boxType 中的一个相同
	 * 第2个维度在 [min, max] 之间，总体积接近但不超过 V，
	 * 第3个维度不超过 maxD
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxTypeSD1(Random rand, 
			int[] boxType, int min, int max, long V, int minD, int maxD) {
		int range = max - min + 1;
		for (int i=0; i<MAX_ATTEMPT; i++) {
			int L = boxType[rand.nextInt(3)]; // L 是 boxType 种随机挑选的一条边长度
			int W = rand.nextInt(range) + min;
			if (equalAnySide(W, boxType)) { continue; }
			int H = (int)(V / (L * W));
			if (H < minD) { continue; }
			if (H > maxD) { continue; }
			if (equalAnySide(H, boxType)) { continue; }
			return new int[] {L,W,H};
		}
		return null;
	}

	/**
	 * 随机生成一种box，它的前两个维度与 boxType 中的两个维度相同
	 * 第3个维度不超过 maxD, 总体积接近但不超过 V，第3个维度可能与 boxType 中的某个维度相同。
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxTypeSD2(Random rand, 
			int[] boxType, long V, int minD, int maxD) {
//		long boxV = ((long) boxType[0]) * boxType[1] * boxType[2];
//		while (true) {
//			int[] r = boxType.clone();
//			int idx = rand.nextInt(3);
//			long area = boxV / boxType[idx];
//			r[idx] = (int) (V / area);
//			if (r[idx] > maxD) { continue; }
//			return r;
//		}
		
		for (int att=0; att<MAX_ATTEMPT; att++) {
			int idx = rand.nextInt(3);
			int i=0;
			while (i == idx) { i++; }
			int L = boxType[i++];
			while (i == idx) { i++; }
			int W = boxType[i];
			
			int H = (int)(V / (L * W));
			if (H < minD) { continue; }
			if (H > maxD) { continue; }
//			if (equalAnySide(H, boxType)) { continue; }
			return new int[] {L,W,H};
		}
		return null;
	}

	/**
	 * 随机生成一种box，前2个维度在 [min, max] 之间，总体积接近但不超过 V，
	 * 第3个维度不超过 maxD
	 * 且没有任何维度与前一个box一样
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxTypeSD0(Random rand, 
			int[] boxType, int min, int max, long V, int minD, int maxD) {
		int range = max - min + 1;
		for (int i=0; i<MAX_ATTEMPT; i++) {
			int L =  rand.nextInt(range) + min;
			if (equalAnySide(L, boxType)) { continue; }
			int W = rand.nextInt(range) + min;
			if (equalAnySide(W, boxType)) { continue; }
			int H = (int)(V / (L * W));
			if (H < minD) { continue; }
			if (H > maxD) { continue; }
			if (equalAnySide(H, boxType)) { continue; }
			return new int[] {L,W,H};
		}
		return null;
	}

	
	// SF 的 7 种 box 的大小
	public static final int[][] boxTypeSF = {{20,18,10},//1,1,1
			{25,20,18},//2,1,2
			{30,25,20},//4,2,2
			{36,30,25},//4,4,4
			{53,32,23},//6,2,3
			{70,40,32},//6,6,6
			{57,35,57}};//6,6,6
	
	/**
	 * 生成 7 种 box 的形状
	 * @param rand  随机数生成器
	 * @param reg	SF: 采用SF的box种类；SD2: 相邻两种box有2个维度相同；SD1: 相邻两种box有1个维度相同；SD0：相邻两种box有0个维度相同 
	 * @return 7 种 box, 每种box的三维大小，长度单位cm
	 */
	public static int[][] pickReg(Random rand, Regularity reg){
		// 一种常用的 pallet 的尺寸为 1.2 m x 1.0 m，假设高度为 1.5 m
		int[] pallet = {120,100,150};
		
		// 生产的box种类大致按照体积从大到小排序
		// 第 i = 0,1,...,6 种box的体积 v_i 约为一个 pallet 的 1/n_i
		//    n_i 是 [2^(i+3), 2^(i+3) + 2^(i+1)] 之间的一个随机数 
		// 这个方法可以保证 n_i 在 8 - 640 之间，最大的box 8个可以填满一个pallet，最小的640个可以填满一个pallet
		int palletV = pallet[0]*pallet[1]*pallet[2];
		int[] possibleV = new int[boxTypeSF.length];
		for(int i=0;i<possibleV.length;i++){
			int n = i+3;
			possibleV[possibleV.length-1-i] = palletV/(rand.nextInt((int)Math.pow(2, n-2)+1)+(int)Math.pow(2, n)); 
		}
		
//		if (debug) { System.out.println("-----------"); }

		//技术 SF 每种box，长宽高的最小与最大值
		int[] sfMin = new int[boxTypeSF.length];
		int[] sfMax = new int[boxTypeSF.length];
		for(int i=0;i<boxTypeSF.length;i++){
			int minD = Integer.MAX_VALUE;
			int maxD = Integer.MIN_VALUE;
			for(int j=0;j<boxTypeSF[i].length;j++){
				if(boxTypeSF[i][j]<minD){
					minD = boxTypeSF[i][j];
				}
				if(boxTypeSF[i][j]>maxD){
					maxD = boxTypeSF[i][j];
				}

			}
			sfMin[i] = minD;
			sfMax[i] = maxD;
		}
		
		//		System.out.println(Arrays.deepToString(sfMinMax));
		int minD = 10; // 一个标准吸盘的直径
		int limitD = 100;//随机生成box的尺寸上限
		switch (reg) {
		case SF:
			return boxTypeSF.clone();
		case SD1:
			int[][] sd1 = new int[boxTypeSF.length][3];
			outer:
			while (true) {
				sd1[0] = genBoxType(rand, sfMin[0], sfMax[0], possibleV[0], minD, limitD);
				for(int i = 1;i<sd1.length;i++){//一个维度与前一种的随机一个维度相等，第二个维度为SFbox的上下限随机生成，第三个维度由体积约束，且保证相邻box至多只有一个维度相等
					sd1[i] = genBoxTypeSD1(rand,sd1[i-1],sfMin[i],sfMax[i],possibleV[i],minD, limitD);
					if (sd1[i] == null) { continue outer; }
				}
				break;
			}
			return sd1;
		case SD2:
			int[][] sd2 = new int[boxTypeSF.length][3];
			
//			if (debug) { System.out.println("-----1------"); }

			outer:
			while (true) {
				sd2[0] = genBoxType(rand, sfMin[0], sfMax[0], possibleV[0], minD, limitD);
				for(int i = 1;i<sd2.length;i++){
	//				if (debug) { System.out.println("----------- "+i); }
					sd2[i] = genBoxTypeSD2(rand,sd2[i-1],possibleV[i],minD, limitD);
					if(sd2[i] == null) { continue outer; }
				}
				break;
			}
			return sd2;
		case SD0:
			int[][] sd0 = new int[boxTypeSF.length][3];
			outer:
			while (true) {
				sd0[0] = genBoxType(rand, sfMin[0], sfMax[0], possibleV[0], minD, limitD);
				for(int i = 1;i<sd0.length;i++){
					sd0[i] = genBoxTypeSD0(rand,sd0[i-1],sfMin[i],sfMax[i],possibleV[i],minD, limitD);
					if (sd0[i] == null) { continue outer; }
				}
				break;
			}
			return sd0;
		default: throw new RuntimeException("Unsupported Regularity");
		}
	}

	
	/**
	 * 把 boxCount 尽量等数量的分配到 typeCount 种盒子
	 * @param boxCount 总盒子数量
	 * @param result   [start, start+typeCount) 用来保存分配结果 result[i] 第i种盒子的个数，除不尽的放在最后一种
	 * @param start
	 * @param typeCount 盒子种类数
	 */
	public static void equalDistribute(int boxCount, int[] result, int start, int typeCount) {
		if (boxCount < 0 || typeCount <= 0) {
			throw new IllegalArgumentException("boxCount: "+boxCount+" typeCount: "+typeCount);
		}
		int countPerType = boxCount / typeCount;
		for (int i=0; i<typeCount-1; i++) {
			result[start+i] = countPerType;
			boxCount -= countPerType;
		}
		result[start+typeCount-1] = boxCount;
	}
	
	/**
	 * 把盒子数，按分布方式分配到各种盒子。
	 * 
	 * 假设box种类按照体积从小到大排序，一共有 totalType 种，从0开始按顺序编号，0，1，2，...
	 * 从 start 开始连续 typeNum 种 box 是用到的种类，
	 * 为用到的种类按照指定的分布生成每种box占的比重
	 * 我们定义体积最小的 typeNum/3 种为小box；体积最大的 typeNum/3 种为大box；其余的为中box
	 * 不同种小（中、大）box比重相同。
	 * 
	 * @param boxCount  总盒子数
	 * @param dist      盒子数分布类型
	 * @param typeCount 盒子种类数
	 * @return [t] 第t种盒子的个数，加在一起等于盒子总数
	 */
	public static int[] genDistribuiton(int boxCount, Distribution dist, int typeCount) {
//		System.out.println("totalType: "+totalType+"; start: "+start+"; typeNum: "+typeNum+"; dist: "+dist);
		if (typeCount <= 0 || boxCount <=0) { throw new RuntimeException(); }
		
		int lowCount = (int) (Math.round(0.2*boxCount));
		int highCount = boxCount - 2 * lowCount;

		
		switch(typeCount) {
		case 1: return new int[] {boxCount};
		case 2:
			switch(dist) {
			case small: return new int[] {highCount, boxCount-highCount};
			case uniform:
			case medium:
				int half = boxCount / 2;
				return new int[] {half, boxCount-half};
			case large: return new int[] {boxCount-highCount, highCount};
			default: throw new IllegalArgumentException("unsupported dist: "+dist);
			}
		default:
			int smallTypeCount = typeCount / 3;
			int largeTypeCount = typeCount / 3;
			int mediumTypeCount = typeCount - smallTypeCount - largeTypeCount;

			int[] boxCountPerType = new int[typeCount];
			switch(dist){
			case uniform: equalDistribute(boxCount, boxCountPerType, 0, typeCount); break;
			case small:
				equalDistribute(highCount, boxCountPerType, 0, smallTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount, mediumTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount+mediumTypeCount, largeTypeCount);
				break;
			case medium:
				equalDistribute(lowCount, boxCountPerType, 0, smallTypeCount);
				equalDistribute(highCount, boxCountPerType, smallTypeCount, mediumTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount+mediumTypeCount, largeTypeCount);
				break;
			case large:
				equalDistribute(lowCount, boxCountPerType, 0, smallTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount, mediumTypeCount);
				equalDistribute(highCount, boxCountPerType, smallTypeCount+mediumTypeCount, largeTypeCount);
				break;
			default:
				throw new IllegalArgumentException("Unsupported distribution: "+dist);
			}
			return boxCountPerType;
		}
		
	}

	public static void genDataSet(File dir, long seed) throws IOException {
		Random rand = new Random(seed);
		
		// Input feature:
		Regularity[] boxTypeRegularityList = Regularity.values();
		int[] boxTypeCountList = new int[]{2,3,4,5,6,7};
		int[] boxCountList = new int[]{200,500,1000,2000};
		int[][] palletLWHList = new int[][] {{120,100,150}};
		Distribution[] boxTypeDistributionList = Distribution.values();
		
		for(Regularity boxTypeRegularity:boxTypeRegularityList){
			for(int boxTypeCount:boxTypeCountList){
				for(int boxCount:boxCountList){
					for(Distribution dist:boxTypeDistributionList){
						InstConfig conf = new InstConfig();
						int[] lwh = palletLWHList[rand.nextInt(palletLWHList.length)];
						conf.instSeed = seed;
						conf.L = lwh[0];
						conf.W = lwh[1];
						conf.H = lwh[2];
						conf.boxTypeRegularity = boxTypeRegularity;
						conf.boxTypeCount = boxTypeCount;
						conf.boxCount = boxCount;
						conf.dist = dist;
						
						InstData inst = Generator.generate(rand,conf);
						File confDir = new File(dir,"conf");
						confDir.mkdirs();
						// 将config存成json文件
						Gson gs = new Gson();
						String json = gs.toJson(conf);
						BufferedWriter bw = new BufferedWriter(new FileWriter(new File(confDir, inst.name+".json")));
						bw.write(json);
						bw.close();
							
						File instDataFile = new File(dir, "instData/"+inst.name+".json");
						InstanceLoader.write(inst, instDataFile);
						System.out.println("write inst: "+inst.name);
					}//dist
				}//boxCount
			} //boxType
		} // regularity
	}
	
	public static void main(String[] args) throws IOException {
		genDataSet(new File(InstanceLoader.testsetPrefix+"2"), 2);
//		genDataSet(new File(InstanceLoader.testsetPrefix+"3"), 3);
//		genDataSet(new File(InstanceLoader.testsetPrefix+"4"), 4);
	}
	
//	public static boolean debug = false;
}


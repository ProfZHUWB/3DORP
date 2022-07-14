package com.zhuwb.research.roboticpacking.inst;

import java.io.PrintStream;

import com.zhuwb.research.roboticpacking.inst.Vacuum.Type;

public class SysConfig implements Cloneable {
	
	public Type vaccumType = Type.TwoByThree; // 机械手吸盘的长宽数量比
	public double theta = 1.0;                // 机械手大小的缩放比例
	public double maxDropHeightRatio = 0.1;   // 一个盒子可以安全的自由下落盒子在H-轴上10%的长度

	// 盒子 b 对盒子 a 提供支持的条件：
	//     a.h1 - b.h2 <= maxGapForSupport  考虑到一些包装材料可能会有小形变，高度差不大时底下的盒子可以支撑上面的
	// 盒子a的底部分成相等的4块
	//   A1 A2
	//   A3 A4
	// 盒子b与Ai在 L-轴上的重叠部分要达到 minOverlapRatioForSupport * (a.l2-a.l1)
	//        且在 W-轴上的重叠部分要达到 minOverlapRatioForSupport * (a.w2-a.w1)
	// 则b对Ai 提供足够支撑
	// 盒子a的4部分底面有>=3个获得支撑则a是稳定的
	public double maxHGapForSupport = 0.5;
	public double minOverlapRatioForSupport = 0.2;  // Zhou的alpha

	// changeable:
	public int knownBoxCount = 5;             // known 系统知道未来的信息数量, N_k
	public int openPalletCount = 3;           // pn 装载时同时打开的托盘数量, P
	public int boxCountInRange = 3;           //oper 机械手可以操作的个数, N_b

	public static String getHeader() {
		return "vaccumType,theta,maxDropHeightRatio,maxHGapForSupport,minOverlapRatioForSupport,knownBoxCount,openPalletCount,boxCountInRange";
	}
	public String toString() {
		return vaccumType+","+theta +","+maxDropHeightRatio+","+maxHGapForSupport+","+minOverlapRatioForSupport+","+knownBoxCount+","+openPalletCount+","+boxCountInRange;
	}
	
	public void print(PrintStream ps, String linePrefix) {
		ps.println(linePrefix+"vaccumType: "+vaccumType);
		ps.println(linePrefix+"theta: "+theta);
		ps.println(linePrefix+"maxDropHeightRatio: "+maxDropHeightRatio);
		ps.println(linePrefix+"maxHGapForSupport: "+maxHGapForSupport);
		ps.println(linePrefix+"minOverlapRatioForSupport: "+minOverlapRatioForSupport);
		ps.println(linePrefix+"knownBoxCount: "+knownBoxCount);
		ps.println(linePrefix+"openPalletCount: "+openPalletCount);
		ps.println(linePrefix+"boxCountInRange: "+boxCountInRange);
	}
		
	public static SysConfig defaultConf = new SysConfig();

	@Override
	public SysConfig clone() {
		try {
			return (SysConfig) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
	}
}

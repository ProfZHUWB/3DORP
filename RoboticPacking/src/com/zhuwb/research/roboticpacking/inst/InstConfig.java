package com.zhuwb.research.roboticpacking.inst;

import java.io.PrintStream;

public class InstConfig implements Cloneable {
	public static enum Regularity {
		SD0, SD1, SD2, SF
	}
	public static enum Distribution {
		uniform, small, medium, large
	}
	public long instSeed=2;
	public int L = 120, W = 100, H = 150;    // length, width, height of pallet
	public int boxTypeCount = 7;             // number of box types
	public Regularity boxTypeRegularity = Regularity.SF; 
	public Distribution dist = Distribution.uniform;	// distribution of boxes across types
	public int boxCount;       // total number of boxes
	
	
	public static String getHeader() {
		return "instSeed,L,W,H,boxTypeCount,boxTypeRegularity,dist,boxCount";
	}
	public String toString() {
		return instSeed + ","+L+","+W +","+H+","+boxTypeCount+","+boxTypeRegularity+","+dist+","+boxCount;
	}
	public void print(PrintStream ps, String linePrefix) {
		ps.println(linePrefix+"instSeed: "+instSeed);
		ps.println(linePrefix+"L x W x H: "+L+" x "+W+" x "+H);
		ps.println(linePrefix+"boxTypeCount: "+boxTypeCount);
		ps.println(linePrefix+"boxTypeRegularity: "+boxTypeRegularity);
		ps.println(linePrefix+"dist: "+dist);
		ps.println(linePrefix+"boxCount: "+boxCount);
	}
		
	public static InstConfig defaultConf = new InstConfig();


	@Override
	public InstConfig clone() {
		try {
			return (InstConfig) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
	}
}

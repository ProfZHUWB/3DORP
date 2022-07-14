package com.zhuwb.research.roboticpacking.inst;

public class InstData {
	public String name;
	public int L,W,H; // length, width, height of a pallet
	public int[][] boxType; // boxType[t] the length, width, height of type t boxes
	public boolean ortPerm[][]; // ortPerm[i][j] = true a type i box can be placed in j-th orientation

	public int[] t; // t[i] is the type of i-th arrived boxes
	public int[] ort; // ort[i] is the orientation of i-th arrived boxes
	
	public InstData(int L, int W, int H, int boxTypeCount, int boxCount) {
		this.L = L;
		this.W = W;
		this.H = H;
		this.boxType = new int[boxTypeCount][];
		this.ortPerm = new boolean[boxTypeCount][];
		this.t = new int[boxCount];
		this.ort = new int[boxCount];
	}
	
	public int getBoxTypeCount() {
		return this.boxType.length;
	}
	public int getBoxCount() {
		return this.t.length;
	}
	public double getPalletVolume() {
		return ((double) L) *W*H;
	}
	public int[] getBoxCountPerType() {
		int[] boxCountPerType = new int[boxType.length];
		for (int i=0; i<t.length; i++) {
			int type = t[i]; 
			boxCountPerType[type]+=1;
		}
		return boxCountPerType;
	}
}
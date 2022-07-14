package com.zhuwb.research.roboticpacking.inst;

import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;
import com.zhuwb.research.roboticpacking.space.Space;

public class Vacuum {
	public double L,W,T; //机器抓手的长、宽与厚度,例如(50,35,5)
	public double r,h;   //真空吸盘半径与厚度，例如(5,5)
	public int M,N;   // M^g, N^g: 吸盘行数与列数，行对应 W; 列对应 L
	
	public Vacuum(double l, double w, double t, double r, double h, int m, int n) {
		L = l;
		W = w;
		T = t;
		this.r = r;
		this.h = h;
		M = m;
		N = n;		
	}
	
	public String toString() {
		return "("+L+" x "+W+" x "+T+") ("+M+" x "+N+") r: "+r+" h: "+h;
	}

	public static enum Type {
		TwoByThree, TwoByTwo, OneByOne
	}
	
	/**
	 * 按照标准机器抓手的规格缩放一定比例 scale 创建一个新抓手
	 * @param scale
	 * @param t
	 * @return
	 */
	public static Vacuum create(double scale, Type t) {
		switch(t) {
		case TwoByThree: return new Vacuum(50*scale,35*scale,5*scale,5*scale,5*scale,2,3);
		case TwoByTwo: return new Vacuum(35*scale,35*scale,5*scale,5*scale,5*scale,2,2);
		case OneByOne: return new Vacuum(20*scale,20*scale,5*scale,5*scale,5*scale,1,1);
		}
		throw new RuntimeException("Unsupported specification!!!");
	}
	
	/**
	 * 假设吸附面沿一个坐标轴的长度为 blen
	 * 抓手按照某个吸附方式在个坐标轴上的长度为 gripperLeng, 吸盘个数为 m,吸盘半径为r
	 * 计算在稳定吸附要求下，抓手中心与吸附面中心在这个轴上的最大允许偏移量。
	 * 
	 * @param blen			盒子的一条边（同时定义了以盒子为基准的局部坐标系的一个坐标轴）
	 * @param gripperLength 
	 * @param r
	 * @param m
	 * @return
	 */
	public static double computMaxOffSet(int blen, double gripperLen, double r, int m) {
		assert blen > 0 && gripperLen >0 && r>0 && m > 0;
		
		// 均匀放置 m 个半径为 r 的吸盘，吸盘之间（及吸盘到抓手边界）的间隔
		double g = (gripperLen - 2 * r * m) / (m + 1);
		
		double suckerAndGapSize = 2*r + g;
		
		// 长度 blen 可以容纳的完整吸盘个数 （不考虑抓手上吸盘个数，认为要多少有多少）
		int maxSuckerCount = (int)Math.floor((blen-2*r)/suckerAndGapSize)+1;
		if (m < maxSuckerCount) { maxSuckerCount = m; } // 最多可以用的吸盘数量不能超过 m 个
		assert maxSuckerCount >= 0;
		
		switch (maxSuckerCount) {
		case 0: throw new IllegalArgumentException("blen: "+blen+" too small to commodate a full sucker of radius: "+r);
		case 1:
			// 2. 当只能容纳1个吸盘时，要用离抓手中间最近的吸盘
			if (m%2 == 1) { // 抓手有奇数个吸盘时，用最中间的那个
				return blen / 2.0 - r;
			}
			// 抓手有偶数个吸盘时，用中间偏左的那个吸盘
			return blen / 2.0 + g/2;
		default: // maxCount >= 2
			// 1. 当可以容纳2个或以上吸盘时，最大偏移量
			return blen / 2.0 + gripperLen / 2 - maxSuckerCount * suckerAndGapSize;
		}
	}
	
	private Space createRounded(double l1, double w1, double h1, double l2, double w2, double h2) {
		return new Space((int)Math.round(l1), (int)Math.round(w1), (int)Math.round(h1), 
				         (int)Math.round(l2), (int)Math.round(w2), (int)Math.round(h2));
	}
	
	
	public static enum PushAxis {
		L, W, H;
	}
	
	public static enum Align {
		org, rotated  // 分别表示抓手的两种旋转方式：org 没有旋转；rotated 旋转90度，L与W互换
	}
	
	/**
	 * 计算各种推送方式和抓手方向下，抓手在吸附上的投影
	 * ‘zhou’列表示在周游的论文中的推送模式
	 * ‘L-’列表示抓手的哪条边放在L-轴上，如W(M)表示抓手的W放在这个轴，括号中的M表示共有M个吸盘沿着这个轴
	 * ‘W-’列表示抓手的哪条边放在L-轴上
	 * ‘H-’列表示抓手的哪条边放在L-轴上
	 * pushAxis align zhou  L-    W-    H-
	 *  L       org    2          L(N)  W(M)
	 *  L     rotated  3          W(M)  L(N)
	 *  W       org    4    L(N)        W(M)
	 *  W     rotated  5    W(M)        L(N)
	 *  H       org    0    L(N)  W(M)
	 *  H     rotated  1    W(M)  L(N)
	 * 
	 * 因为pallet中摆放的盒子会尽量集中在离原点近的角落，所以抓手在吸附面试上会尽量偏向远离原点的方向，
	 * 可以尽量减少与已摆放盒子的碰撞。
	 * @param ort
	 * @param pushAxis
	 * @param align
	 * @return
	 */
	public Space[][] computeGripperProjection(Orientation ort) {
		Space[][] projection = new Space[PushAxis.values().length][Align.values().length];
		
		double startL, startW, startH, endL, endW, endH;
		// L-push, maximize W- and H- so as to minimize the chance of collide with placed boxes
		// pushAxis align zhou  L-    W-    H-
		//  L       org    2          L(N)  W(M)
		//  L     rotated  3          W(M)  L(N)
		startL = ort.l; endL = ort.l;
		startW = ort.w/2.0 + computMaxOffSet(ort.w, L, r, N) - L/2.0;
		endW = startW + L;
		startH = ort.h/2.0 + computMaxOffSet(ort.h, W, r, M) - W/2.0;
		endH = startH + W;
		projection[PushAxis.L.ordinal()][Align.org.ordinal()]
				= createRounded(startL,startW,startH,endL,endW,endH);
		
		startL = ort.l; endL = ort.l;
		startW = ort.w/2.0 + computMaxOffSet(ort.w, W, r, M) - W/2.0;
		endW = startW + W;
		startH = ort.h/2.0 + computMaxOffSet(ort.h, L, r, N) - L/2.0;
		endH = startH + L;
		projection[PushAxis.L.ordinal()][Align.rotated.ordinal()]
				= createRounded(startL,startW,startH,endL,endW,endH);
				
		// pushAxis align zhou  L-    W-    H-
		//  W       org    4    L(N)        W(M)
		//  W     rotated  5    W(M)        L(N)
		startL = ort.l/2.0 + computMaxOffSet(ort.l, L, r, N) - L/2.0;
		endL = startL + L;
		startW = ort.w; endW = ort.w;
		startH = ort.h/2.0 + computMaxOffSet(ort.h, W, r, M) - W/2.0;
		endH = startH + W;
		projection[PushAxis.W.ordinal()][Align.org.ordinal()]
				= createRounded(startL,startW,startH,endL,endW,endH);
		
		startL = ort.l/2.0 + computMaxOffSet(ort.l, W, r, M) - W/2.0;
		endL = startL + W;
		startW = ort.w; endW = ort.w;
		startH = ort.h/2.0 + computMaxOffSet(ort.h, L, r, N) - L/2.0;
		endH = startH + L;
		projection[PushAxis.W.ordinal()][Align.rotated.ordinal()]
				= createRounded(startL,startW,startH,endL,endW,endH);
		
		// pushAxis align zhou  L-    W-    H-
		//  H       org    0    L(N)  W(M)
		//  H     rotated  1    W(M)  L(N)
		startL = ort.l/2.0 + computMaxOffSet(ort.l, L, r, N) - L/2.0;
		endL = startL + L;
		startW = ort.w/2.0 + computMaxOffSet(ort.w, W, r, M) - W/2.0;
		endW = startW + W;
		startH = ort.h; endH = ort.h;
		projection[PushAxis.H.ordinal()][Align.org.ordinal()]
				= createRounded(startL,startW,startH,endL,endW,endH);
				
		startL = ort.l/2.0 + computMaxOffSet(ort.l, W, r, M) - W/2.0;
		endL = startL + W;
		startW = ort.w/2.0 + computMaxOffSet(ort.w, L, r, N) - L/2.0;
		endW = startW + L;
		startH = ort.h; endH = ort.h;
		projection[PushAxis.H.ordinal()][Align.rotated.ordinal()]
				= createRounded(startL,startW,startH,endL,endW,endH);
		
		return projection;
	}	
	
	// 一个以ort旋转方式摆放的盒子在(rL,rW,rH)处释放。计算抓手在align对齐下沿L推送形成的轨迹
	public Space gripperPathL(int rL,int rW,int rH, Orientation ort, int palletL, Align align) {
		Space proj = ort.gripperProjection[PushAxis.L.ordinal()][align.ordinal()];
		Space s = proj.translate(rL, rW, rH);
		s.l2 = palletL;
		return s;
	}	
	// 一个以ort旋转方式摆放的盒子在(rL,rW,rH)处释放。计算抓手在align对齐下沿W推送形成的轨迹
	public Space gripperPathW(int rL,int rW,int rH, Orientation ort, int palletW, Align align) {
		Space proj = ort.gripperProjection[PushAxis.W.ordinal()][align.ordinal()];
		Space s = proj.translate(rL, rW, rH);
		s.w2 = palletW;
		return s;
	}
	// 一个以ort旋转方式摆放的盒子在(rL,rW,rH)处释放。计算抓手在align对齐下沿H推送形成的轨迹
	public Space gripperPathH(int rL,int rW,int rH, Orientation ort, int palletH, Align align) {
		Space proj = ort.gripperProjection[PushAxis.H.ordinal()][align.ordinal()];
		Space s = proj.translate(rL, rW, rH);
		s.h2 = palletH;
		return s;
	}
	
	// 一个以ort旋转方式摆放的盒子在(rL,rW,rH)处释放。计算盒子沿 pushAxis 轴推送形成的轨迹
	public static Space computeBoxPath(int rL,int rW,int rH, Orientation ort, Space pallet, PushAxis pushAxis) {
		switch (pushAxis) {
		case L: return new Space(rL+ort.l, rW,       rH,       pallet.l2,  rW+ort.w, rH+ort.h);
		case W: return new Space(rL,       rW+ort.w, rH,       rL+ort.l, pallet.w2,  rH+ort.h);
		case H: return new Space(rL,       rW,       rH+ort.h, rL+ort.l, rW+ort.w, pallet.h2);
		default: throw new IllegalArgumentException("pushAxis: "+pushAxis);
		}
	}
	
	public int[] computeGripperLocation(Space gripperPath, PushAxis pushAxis, Space pallet) {
		switch (pushAxis) {
		case L: return new int[] {pallet.l2, gripperPath.w1, gripperPath.h1};
		case W: return new int[] {gripperPath.l1, pallet.w2, gripperPath.h1};
		case H: return new int[] {gripperPath.l1, gripperPath.w1, pallet.h2};
		default: throw new IllegalArgumentException("pushAxis: "+pushAxis);
		}
	}

	
	public boolean canLPush(Orientation ort) {
		double d = this.r*2;
		return ort.w >= d && ort.h >= d;
	}

	public boolean canWPush(Orientation ort) {
		double d = this.r*2;
		return ort.l >= d && ort.h >= d;
	}

	public boolean canHPush(Orientation ort) {
		double d = this.r*2;
		return ort.l >= d && ort.w >= d;
	}

	/*
	 * pushAxis align zhou  L-    W-    H-
	 *  L       org    2          L(N)  W(M)
	 *  L     rotated  3          W(M)  L(N)
	 *  W       org    4    L(N)        W(M)
	 *  W     rotated  5    W(M)        L(N)
	 *  H       org    0    L(N)  W(M)
	 *  H     rotated  1    W(M)  L(N)
	 */
//	// gLen[pushAxis][align][axis]: 吸附方式 mode 下对应盒子轴 axis 的吸盘的长度
	public double[][][] getGripperLength() {
		return new double[][][] {
			{{this.T,  this.L, this.W},
			 {this.T,  this.W, this.L}},
			{{this.L,  this.T, this.W},
			 {this.W,  this.T, this.L}},
			{{this.L,  this.W, this.T},
			 {this.W,  this.L, this.T}}};
	}
//	public double[] computeGripperLength(int mode) {
//		switch (mode) {
//		case 0: return new double[]{this.L,  this.W, this.T};
//		case 1: return new double[]{this.W,  this.L, this.T};
//		case 2: return new double[]{this.T,  this.L, this.W};
//		case 3: return new double[]{this.T,  this.W, this.L};
//		case 4: return new double[]{this.L,  this.T, this.W};
//		case 5: return new double[]{this.W,  this.T, this.L};
//		default: throw new IllegalArgumentException("gripping mode: "+mode+" not supported, only supports {0,1,2,3,4,5,6}");
//		}
//	}
//	
	// SuckerThicness[mode][axis]: 吸附方式 mode 下对应盒子轴 axis 的吸盘的厚度
	public double[][][] getSuckerThicness() {
		return new double[][][] {
			{{this.h, 0, 0},
			 {this.h, 0, 0}},
			{{0, this.h, 0},
			 {0, this.h, 0}},
			{{0,  0, this.h},
			 {0,  0, this.h}}};
	}
	// suckerCount[mode][axis]: 吸附方式 mode 下对应盒子轴 axis 的吸盘个数
	public int[][][] getSuckerCount() {
		return new int[][][] {
			{{1,          this.N,         this.M},
			 {1,          this.M,         this.N}},
			{{this.N,     1,              this.M},
			 {this.M,     1,              this.N}},
			{{this.N,     this.M,         1},
			 {this.M,     this.N,         1}}};
	}
	// firstSuckerCenterRelativePosition[mode][axis]
	// 抓取模式 mode 下最坐下角吸盘相对于抓手左下角坐标沿 box 的 axis 轴的偏移量
	// 是第一 sucer 的中心相对于抓手原点的偏移量
	public double[][][] getFirstSuckerCenterRelativePosition() {
		double cL = (L - N * 2 * r) / (N+1) + r;
		double cW = (W - M * 2 * r) / (M+1) + r;

		return new double[][][] {
			{{0, cL, cW},
			 {0, cW, cL}},
			{{cL, 0, cW},
			 {cW, 0, cL}},
			{{cL, cW, 0},
			 {cW, cL, 0}}};
	}
	// suckerCenterDistance[mode][axis]
	// 抓取模式 mode 相邻两个sucker的中心坐标沿 box 的 axis 轴的差
	public double[][][] getSuckerCenterDistance() {
		double dL = (L - N * 2 * r) / (N+1) + 2 * r;
		double dW = (W - M * 2 * r) / (M+1) + 2 * r;
		
		return new double[][][] {
			{{0, dL, dW},
			 {0, dW, dL}},
			{{dL, 0, dW},
			 {dW, 0, dL}},
			{{dL, dW, 0},
			 {dW, dL, 0}}};
	}

	public static void main(String[] args) {
//		BoxType bt = new BoxType(41, 26, 10, new boolean[] {true,true,true,true,true,true}, gripper);
//			{41,26,10},
//			{41,10,26},
//			{26,41,10},
//			{26,10,41},
//			{10,41,26},
//			{10,26,41}};
//		Orientation ort = bt.distinctOrt[0]; // 41,26,10
//		
//		Vacuum gripper = Vacuum.create(1.0, Type.TwoByThree);
//		Orientation ort = new Orientation(41, 26, 10, 0, 0.1);
//		ort.gripperProjection = gripper.computeGripperProjection(ort);
//		Space[] list = new Space[] {
//				new Space(41,-4,-5,41,46,30),
//				new Space(41,-4,-20,41,31,30),
//				new Space(-4,26,-5,46,26,30),
//				new Space(11,26,-20,46,26,30),
//				new Space(-4,-4,10,46,31,10),
//				new Space(11,-4,10,46,46,10)
//		};
	}
}

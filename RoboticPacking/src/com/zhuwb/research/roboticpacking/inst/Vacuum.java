package com.zhuwb.research.roboticpacking.inst;

import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;
import com.zhuwb.research.roboticpacking.space.Space;

public class Vacuum {
	public double L,W,T; //����ץ�ֵĳ���������,����(50,35,5)
	public double r,h;   //������̰뾶���ȣ�����(5,5)
	public int M,N;   // M^g, N^g: �����������������ж�Ӧ W; �ж�Ӧ L
	
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
	 * ���ձ�׼����ץ�ֵĹ������һ������ scale ����һ����ץ��
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
	 * ������������һ��������ĳ���Ϊ blen
	 * ץ�ְ���ĳ��������ʽ�ڸ��������ϵĳ���Ϊ gripperLeng, ���̸���Ϊ m,���̰뾶Ϊr
	 * �������ȶ�����Ҫ���£�ץ��������������������������ϵ��������ƫ������
	 * 
	 * @param blen			���ӵ�һ���ߣ�ͬʱ�������Ժ���Ϊ��׼�ľֲ�����ϵ��һ�������ᣩ
	 * @param gripperLength 
	 * @param r
	 * @param m
	 * @return
	 */
	public static double computMaxOffSet(int blen, double gripperLen, double r, int m) {
		assert blen > 0 && gripperLen >0 && r>0 && m > 0;
		
		// ���ȷ��� m ���뾶Ϊ r �����̣�����֮�䣨�����̵�ץ�ֱ߽磩�ļ��
		double g = (gripperLen - 2 * r * m) / (m + 1);
		
		double suckerAndGapSize = 2*r + g;
		
		// ���� blen �������ɵ��������̸��� ��������ץ�������̸�������ΪҪ�����ж��٣�
		int maxSuckerCount = (int)Math.floor((blen-2*r)/suckerAndGapSize)+1;
		if (m < maxSuckerCount) { maxSuckerCount = m; } // �������õ������������ܳ��� m ��
		assert maxSuckerCount >= 0;
		
		switch (maxSuckerCount) {
		case 0: throw new IllegalArgumentException("blen: "+blen+" too small to commodate a full sucker of radius: "+r);
		case 1:
			// 2. ��ֻ������1������ʱ��Ҫ����ץ���м����������
			if (m%2 == 1) { // ץ��������������ʱ�������м���Ǹ�
				return blen / 2.0 - r;
			}
			// ץ����ż��������ʱ�����м�ƫ����Ǹ�����
			return blen / 2.0 + g/2;
		default: // maxCount >= 2
			// 1. ����������2������������ʱ�����ƫ����
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
		org, rotated  // �ֱ��ʾץ�ֵ�������ת��ʽ��org û����ת��rotated ��ת90�ȣ�L��W����
	}
	
	/**
	 * ����������ͷ�ʽ��ץ�ַ����£�ץ���������ϵ�ͶӰ
	 * ��zhou���б�ʾ�����ε������е�����ģʽ
	 * ��L-���б�ʾץ�ֵ������߷���L-���ϣ���W(M)��ʾץ�ֵ�W��������ᣬ�����е�M��ʾ����M���������������
	 * ��W-���б�ʾץ�ֵ������߷���L-����
	 * ��H-���б�ʾץ�ֵ������߷���L-����
	 * pushAxis align zhou  L-    W-    H-
	 *  L       org    2          L(N)  W(M)
	 *  L     rotated  3          W(M)  L(N)
	 *  W       org    4    L(N)        W(M)
	 *  W     rotated  5    W(M)        L(N)
	 *  H       org    0    L(N)  W(M)
	 *  H     rotated  1    W(M)  L(N)
	 * 
	 * ��Ϊpallet�аڷŵĺ��ӻᾡ����������ԭ����Ľ��䣬����ץ�������������ϻᾡ��ƫ��Զ��ԭ��ķ���
	 * ���Ծ����������Ѱڷź��ӵ���ײ��
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
	
	// һ����ort��ת��ʽ�ڷŵĺ�����(rL,rW,rH)���ͷš�����ץ����align��������L�����γɵĹ켣
	public Space gripperPathL(int rL,int rW,int rH, Orientation ort, int palletL, Align align) {
		Space proj = ort.gripperProjection[PushAxis.L.ordinal()][align.ordinal()];
		Space s = proj.translate(rL, rW, rH);
		s.l2 = palletL;
		return s;
	}	
	// һ����ort��ת��ʽ�ڷŵĺ�����(rL,rW,rH)���ͷš�����ץ����align��������W�����γɵĹ켣
	public Space gripperPathW(int rL,int rW,int rH, Orientation ort, int palletW, Align align) {
		Space proj = ort.gripperProjection[PushAxis.W.ordinal()][align.ordinal()];
		Space s = proj.translate(rL, rW, rH);
		s.w2 = palletW;
		return s;
	}
	// һ����ort��ת��ʽ�ڷŵĺ�����(rL,rW,rH)���ͷš�����ץ����align��������H�����γɵĹ켣
	public Space gripperPathH(int rL,int rW,int rH, Orientation ort, int palletH, Align align) {
		Space proj = ort.gripperProjection[PushAxis.H.ordinal()][align.ordinal()];
		Space s = proj.translate(rL, rW, rH);
		s.h2 = palletH;
		return s;
	}
	
	// һ����ort��ת��ʽ�ڷŵĺ�����(rL,rW,rH)���ͷš���������� pushAxis �������γɵĹ켣
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
//	// gLen[pushAxis][align][axis]: ������ʽ mode �¶�Ӧ������ axis �����̵ĳ���
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
	// SuckerThicness[mode][axis]: ������ʽ mode �¶�Ӧ������ axis �����̵ĺ��
	public double[][][] getSuckerThicness() {
		return new double[][][] {
			{{this.h, 0, 0},
			 {this.h, 0, 0}},
			{{0, this.h, 0},
			 {0, this.h, 0}},
			{{0,  0, this.h},
			 {0,  0, this.h}}};
	}
	// suckerCount[mode][axis]: ������ʽ mode �¶�Ӧ������ axis �����̸���
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
	// ץȡģʽ mode �������½����������ץ�����½������� box �� axis ���ƫ����
	// �ǵ�һ sucer �����������ץ��ԭ���ƫ����
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
	// ץȡģʽ mode ��������sucker������������ box �� axis ��Ĳ�
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

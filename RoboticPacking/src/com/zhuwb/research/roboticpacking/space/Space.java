package com.zhuwb.research.roboticpacking.space;

import com.zhuwb.research.rpp2i.boxpp.segmenttree.LayoutST;

/**
 * Represent a usable space inside some pallet
 * A cuboid with vertex (l1, w1, h1) and (l2, w2, h2)
 * @author iwenc
 */
public class Space implements Cloneable { 
	public int l1,w1,h1;
	public int l2,w2,h2;
	
	public final int dL, dW, dH;
	
	public final double volume;

	public static enum Face {
		L1, L2, W1, W2, H1, H2;
	}
	
	public int getFaceCoord(Face f) {
		switch (f) {
		case L1: return this.l1;
		case L2: return this.l2;
		case W1: return this.w1;
		case W2: return this.w2;
		case H1: return this.h1;
		case H2: return this.h2;
		default: throw new IllegalArgumentException("unsupported face: "+f);
		}
	}
	
	public Space(int l1, int w1, int h1, int l2, int w2, int h2) {
		this.l1 = l1;
		this.w1 = w1;
		this.h1 = h1;
		this.l2 = l2;
		this.w2 = w2;
		this.h2 = h2;
		this.dL = l2 - l1;
		this.dW = w2 - w1;
		this.dH = h2 - h1;
		this.volume = 1.0 * dL * dW * dH;
	}
	
	@Override
	public Object clone() {
		return new Space(l1, w1, h1, l2, w2, h2);
	}
	
	@Override
	public boolean equals(Object o) {
		Space h = (Space)o;
		return l1 == h.l1 && l2 == h.l2 && w1 == h.w1 && 
				w2 == h.w2 && h1 == h.h1 && h2 == h.h2;
	}
	
	@Override
	public String toString() {
		return "(" + l1 + ", " + w1 + ", " + h1 + ") - (" + l2 + ", " + w2 + ", " + h2 + ")";
	}
	
	public boolean contains(Space h) {
		return l1 <= h.l1 && h.l2 <= l2 &&
				w1 <= h.w1 && h.w2 <= w2 &&
				h1 <= h.h1 && h.h2 <= h2;
	}
	
	public boolean intersectTest(Space h) {
		return l1 < h.l2 && w1 < h.w2 && h.l1 < l2 && h.w1 < w2 && h1 < h.h2 && h.h1 < h2;
	}
	
//	public void union(Space h) {
//		if (l1 > h.l1) l1 = h.l1;
//		if (w1 > h.w1) w1 = h.w1;
//		if (h1 > h.h1) h1 = h.h1;
//		if (l2 < h.l2) l2 = h.l2;
//		if (w2 < h.w2) w2 = h.w2;
//		if (h2 < h.h2) h2 = h.h2;
//		volume = 1.0 * (l2 - l1) * (w2 - w1) * (h2 - h1);
//	}
//	
//	public static Space getUnion(Space h1, Space h2) {
//		return new Space(Math.min(h1.l1, h2.l1), Math.min(h1.w1, h2.w1), Math.min(h1.h1, h2.h1),
//					Math.max(h1.l2, h2.l2), Math.max(h1.w2, h2.w2), Math.max(h1.h2, h2.h2));
//	}
//	
//	public double getVolume() {
//		return volume;
//	}
	
	public boolean largeEnoughFor(int l, int w, int h) {
		return this.dL >= l && this.dW >= w && this.dH >= h;
	}

	
	//assume that the intersection is non-zero
	public Space intersect(Space o) {
		return new Space(Math.max(l1, o.l1), Math.max(w1, o.w1), Math.max(h1, o.h1),
				Math.min(l2, o.l2), Math.min(w2, o.w2), Math.min(h2, o.h2));
		//, this.Max_Corner_Count);
	}	
	
	public double[] computeCenter() {
		return new double[] {(l1+l2)/2.0, (w1+w2)/2.0, (h1+h2)/2.0};
	}
	
	public Space translate(int dl, int dw, int dh) {
		return new Space(dl+l1, dw+w1, dh+h1, dl+l2, dw+w2, dh+h2);
	}
	

	/////////////////////////////////////////////////////////
	// 只要 ort 是一样的，无论 L-,W-,H-push都一样
	public int[] wGrid;
	public int[] lGrid;

	// L-push的投影
	public LayoutST pushProjectionL;
	public int[] hGridLPush;
	
	public LayoutST pushProjectionW;
	public int[] hGridWPush;
	
	public LayoutST pushProjectionH;
	
	// projection  x1 y1 x2 y2 start end
	//  0 (L-)     w1 h1 w2 h2  l1    l2
	//  1 (W-)     l1 h1 l2 h2  w1    w2
	//  2 (H-)     l1 w1 l2 w2  h1    h2	
	// this.space与指定的space沿某个轴(L,W,H)垂直的面进行投影，是否相交
	public boolean projectionIntersectL(Space s) {
		return w1 < s.w2 && s.w1 < w2 && h1 < s.h2 && s.h1 < h2; //L-push
	}
	public boolean projectionIntersectW(Space s) {
		return l1 < s.l2 && s.l1 < l2 && h1 < s.h2 && s.h1 < h2; //W-push
	}
	public boolean projectionIntersectH(Space s) {
		return l1 < s.l2 && w1 < s.w2 && s.l1 < l2 && s.w1 < w2; //H-push
	}
	
	public int overlapOnL(Space s) {
		int dL = Math.min(this.l2, s.l2) - Math.max(this.l1, s.l1);
		if (dL < 0) { dL = 0; }
		return dL;
	}
	public int overlapOnW(Space s) {
		int dW = Math.min(this.w2, s.w2) - Math.max(this.w1, s.w1);
		if (dW < 0) { dW = 0; }
		return dW;
	}
	public int overlapOnH(Space s) {
		int dH = Math.min(this.h2, s.h2) - Math.max(this.h1, s.h1);
		if (dH < 0) { dH = 0; }
		return dH;
	}

	public static enum BaseRegion {
		A, B, C, D
	}
	
	/**
	 * Compute overlap length of [aL1, aL2] and [bL1, bL2]
	 * @param aL1
	 * @param aL2
	 * @param bL1
	 * @param bL2
	 * @return
	 */
	public static double overlap(double aL1, double aL2, double bL1, double bL2) {
		double start = Math.max(aL1, bL1);
		double end = Math.min(aL2, bL2);
		if (start >= end) { return 0; }
		return end-start;
	}

	/**
	 * 计算 this 可以支撑 top 底面的哪个部分
	 *        ------------
	 *    W大 |  B  | D   |
	 *        |-----|-----
	 *    W小 |  A  | C   |
	 *        ------------
	 *          L小   L大 
	 * @param top
	 * @param allowedHGap
	 * @param minOverlapRatioForSupport
	 * @return
	 */
	public int[] countSupportRegion(Space top, double allowedHGap, double minOverlapRatioForSupport) {
		int[] supportFlag = new int[BaseRegion.values().length];
		
		if (this.h2 > top.h1) { return supportFlag; }
		if (this.h2 + allowedHGap < top.h1) { return supportFlag; }
		
		double L1 = top.l1, Lc = (top.l1 + top.l2)/2.0, L2 = top.l2;
		double W1 = top.w1, Wc = (top.w1 + top.w2)/2.0, W2 = top.w2;
		double minOverlapL = top.dL * minOverlapRatioForSupport;
		double minOverlapW = top.dW * minOverlapRatioForSupport;
		
		
		Space curSpace = this;
		if (curSpace.l1 < L2 && curSpace.w1 < W2 && L1 < curSpace.l2 && W1 < curSpace.w2) {
			if (overlap(curSpace.l1, curSpace.l2, L1, Lc) >= minOverlapL &&
						overlap(curSpace.w1, curSpace.w2, W1, Wc) >= minOverlapW) {
				supportFlag[BaseRegion.A.ordinal()] = 1;
			}
			if (overlap(curSpace.l1, curSpace.l2, L1, Lc) >= minOverlapL &&
						overlap(curSpace.w1, curSpace.w2, Wc, W2) >= minOverlapW) {
				supportFlag[BaseRegion.B.ordinal()] = 1;
			}
			if (overlap(curSpace.l1, curSpace.l2, Lc, L2) >= minOverlapL &&
						overlap(curSpace.w1, curSpace.w2, W1, Wc) >= minOverlapW) {
				supportFlag[BaseRegion.C.ordinal()] = 1;
			}
			if (overlap(curSpace.l1, curSpace.l2, Lc, L2) >= minOverlapL &&
						overlap(curSpace.w1, curSpace.w2, Wc,W2) >= minOverlapW) {
				supportFlag[BaseRegion.D.ordinal()] = 1;
			}
		}
		return supportFlag;
	}
	
//	public boolean projectionIntersect(Space s, int j) {
//		switch(j) {
//		case 0: return w1 < s.w2 && s.w1 < w2 && h1 < s.h2 && s.h1 < h2; //L-push
//		case 1: return l1 < s.l2 && s.l1 < l2 && h1 < s.h2 && s.h1 < h2; //W-push
//		case 2: return l1 < s.l2 && w1 < s.w2 && s.l1 < l2 && s.w1 < w2; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");
//		}
//	}
	
//	// projection  x1 y1 x2 y2 start end
//	//  0 (L-)     w1 h1 w2 h2  l1    l2
//	//  1 (W-)     l1 h1 l2 h2  w1    w2
//	//  2 (H-)     l1 w1 l2 w2  h1    h2	
//	public int getX1(int j) {
//		switch(j) {
//		case 0: return w1; //L-push
//		case 1: return l1; //W-push
//		case 2: return l1; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");
//		}
//	}
//	// projection  x1 y1 x2 y2 start end
//	//  0 (L-)     w1 h1 w2 h2  l1    l2
//	//  1 (W-)     l1 h1 l2 h2  w1    w2
//	//  2 (H-)     l1 w1 l2 w2  h1    h2	
//	public int getY1(int j) {
//		switch(j) {
//		case 0: return h1; //L-push
//		case 1: return h1; //W-push
//		case 2: return w1; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");
//		}		
//	}
//	// projection  x1 y1 x2 y2 start end
//	//  0 (L-)     w1 h1 w2 h2  l1    l2
//	//  1 (W-)     l1 h1 l2 h2  w1    w2
//	//  2 (H-)     l1 w1 l2 w2  h1    h2	
//	public int getX2(int j) {
//		switch(j) {
//		case 0: return w2; //L-push
//		case 1: return l2; //W-push
//		case 2: return l2; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");
//		}
//	}
//	// projection  x1 y1 x2 y2 start end
//	//  0 (L-)     w1 h1 w2 h2  l1    l2
//	//  1 (W-)     l1 h1 l2 h2  w1    w2
//	//  2 (H-)     l1 w1 l2 w2  h1    h2	
//	public int getY2(int j) {
//		switch(j) {
//		case 0: return h2; //L-push
//		case 1: return h2; //W-push
//		case 2: return w2; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");
//		}
//	}
//	// projection  x1 y1 x2 y2 start end
//	//  0 (L-)     w1 h1 w2 h2  l1    l2
//	//  1 (W-)     l1 h1 l2 h2  w1    w2
//	//  2 (H-)     l1 w1 l2 w2  h1    h2	
//	// 检测 this space 是否会阻挡 box 沿 j-轴 push 进入 s
//	public boolean willBlock(Space s, int j) {
//		switch(j) {
//		case 0: return this.l2 > s.l1; //L-push
//		case 1: return this.w2 > s.w1; //W-push
//		case 2: return this.h2 > s.h1; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");	
//		}
//	}
//	public int getStart(int j) {
//		switch(j) {
//		case 0: return l1; //L-push
//		case 1: return w1; //W-push
//		case 2: return h1; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");
//		}
//	}
//	public int getEnd(int j) {
//		switch(j) {
//		case 0: return l2; //L-push
//		case 1: return w2; //W-push
//		case 2: return h2; //H-push
//		default: throw new IllegalArgumentException("j: "+j+" not {0,1,2}");
//		}
//	}
}

package com.zhuwb.research.roboticpacking.search;

import com.zhuwb.research.roboticpacking.inst.Placement;
import com.zhuwb.research.roboticpacking.space.Space;

// FitnessZhouNewType： 有多个托盘时，新盒子尽量放进已经有这种盒子的托盘中，
// 与 Zhou 比在 OpenPalletCount 实验中前两个例子没有影响
public enum Fitness {
	Zhou {

		@Override
		public double[] fitness(AlgoConfig conf, Pallet P, Placement p) {

			return new double[] {
					calRegularityL(P, p, conf.hweight) + calRegularityW(P, p, conf.hweight) + calRegularityH(P, p),
					
					touchAreaL1(P, p) + touchAreaW1(P, p) + touchAreaH1(P, p), 
					
					calSupport(P, p), -p.spaceH
					};
		}

	},
	
	Regularity {
		@Override
		public double[] fitness(AlgoConfig conf, Pallet P, Placement p) {
			return new double[] {
					matchFaceL1(P, p) + matchFaceW1(P, p) + matchFaceH1(P, p) + 
					matchFaceL2(P, p) + matchFaceW2(P, p) + matchFaceH2(P, p), 
					
					calRegularityL(P, p, conf.hweight) + calRegularityW(P, p, conf.hweight) + calRegularityH(P, p),
					
					touchAreaL1(P, p) + touchAreaW1(P, p) + touchAreaH1(P, p), 
					
					calSupport(P, p),
					
					-p.occupied.h1
					};
		}
	},
	
	// 如果添加的 box 与 pallet 中已有不同，会导致比较不规整
	// 其它条件相同，希望把 box 放进已经有同种 box 的 pallet 中
	ZhouNewType {
		@Override
		public double[] fitness(AlgoConfig conf, Pallet P, Placement p) {
			double newTypeWeight = -1;
			double penaltyForNewType = 0;
			if (P.loadedCountPerType[p.boxType] == 0) {
				double areaOfThreeFaces = p.ort.l * p.ort.w + p.ort.w * p.ort.h + p.ort.l * p.ort.h;
				penaltyForNewType = areaOfThreeFaces * newTypeWeight;
			}

			return new double[] {
					calRegularityL(P, p, conf.hweight) + calRegularityW(P, p, conf.hweight) + calRegularityH(P, p) + penaltyForNewType,
					
					touchAreaL1(P, p) + touchAreaW1(P, p) + touchAreaH1(P, p), 
					
					calSupport(P, p), -p.spaceH
					};
		}
	};
	
	public abstract double[] fitness(AlgoConfig conf, Pallet P, Placement p);
	
	// 计算盒子的支撑率
	public static double calSupport(Pallet P, Placement p){
		Space occupied = p.occupied;
		// 盒子直接落在Pallet的底部
		if (occupied.h1==0) {
			return 1.0;
		}
		
		double bottom = p.ort.l * p.ort.w;  // 盒子的底面积
		double sum = 0;
		for (int i=0; i<P.placedDescH2.length; i++) {
			Space ocp = P.placedDescH2[i].p.occupied;
			if (ocp.h2 == occupied.h1 && ocp.projectionIntersectH(occupied)) {
				sum += occupied.overlapOnL(ocp) * occupied.overlapOnW(ocp);
			}
		}
		return sum/bottom;
	}
	
	// 计算沿L轴方向规整的面积
	public static double calRegularityL(Pallet P, Placement p, double hweight) {
		Space occupied = p.occupied;
		double sum = 0;
		for (int i=0; i<P.placedDescL2.length; i++) {
			Space ocp = P.placedDescL2[i].p.occupied;
			if (ocp.l2 == occupied.l1 && ocp.projectionIntersectL(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.h1==occupied.h1){     // H规整，需要乘以一个zweight
					sum+= hweight*occupied.overlapOnW(ocp) * occupied.overlapOnH(ocp);
				}
				if (ocp.w2==occupied.w2 && ocp.w1==occupied.w1) {  // W规整，不需要乘以一个zweight
					sum+= occupied.overlapOnW(ocp) * occupied.overlapOnH(ocp);
				}
			}
		}
		return sum;
	}
	
	// 计算沿W轴方向规整的面积
	public static double calRegularityW(Pallet P, Placement p, double hweight) {
		Space occupied = p.occupied;
		double sum = 0;
		for (int i=0; i<P.placedDescW2.length; i++) {
			Space ocp = P.placedDescW2[i].p.occupied;
			if (ocp.w2 == occupied.w1 && ocp.projectionIntersectW(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.h1==occupied.h1){     // H规整，需要乘以一个zweight
					sum+= hweight*occupied.overlapOnL(ocp) * occupied.overlapOnH(ocp);
				}
				if (ocp.l2==occupied.l2 && ocp.l1==occupied.l1) {  //  L规整，不需要乘以一个zweight
					sum+= occupied.overlapOnL(ocp) * occupied.overlapOnH(ocp);
				}
			}
		}
		return sum;
	}
	
	// 计算沿H轴方向规整的面积
	public static double calRegularityH(Pallet P, Placement p) {
		Space occupied = p.occupied;
		double sum = 0;
		for (int i=0; i<P.placedDescH2.length; i++) {
			Space ocp = P.placedDescH2[i].p.occupied;
			if (ocp.h2 == occupied.h1 && ocp.projectionIntersectH(occupied)) {
				if ((ocp.l2==occupied.l2 && ocp.l1==occupied.l1) || (ocp.w2==occupied.w2 && ocp.w1==occupied.w1)){     
					sum+= occupied.overlapOnW(ocp) * occupied.overlapOnL(ocp);
				}
			}
		}
		return sum;
	}
	
	public static double touchAreaL1(Pallet P, Placement p) {
		Space occupied = p.occupied;
		
		// l方向与pallet接触
		if (occupied.l1==0) {
			return p.ort.w*p.ort.h;
		}

		double sum = 0;
		for (int i=0; i<P.placedDescL2.length; i++) {
			Space ocp = P.placedDescL2[i].p.occupied;
			if (ocp.l2 == occupied.l1 && ocp.projectionIntersectL(occupied)) {
				sum += occupied.overlapOnW(ocp) * occupied.overlapOnH(ocp);
			}
		}
		
		return sum;
	}

	public static double touchAreaW1(Pallet P, Placement p) {
		Space occupied = p.occupied;
		
		// w方向与pallet接触
		if (occupied.w1==0) {
			return p.ort.l*p.ort.h;
		}

		double sum = 0;
		for (int i=0; i<P.placedDescW2.length; i++) {
			Space ocp = P.placedDescW2[i].p.occupied;
			if (ocp.w2 == occupied.w1 && ocp.projectionIntersectW(occupied)) {
				sum += occupied.overlapOnL(ocp) * occupied.overlapOnH(ocp);
			}
		}
		
		return sum;
	}
	
	public static double touchAreaH1(Pallet P, Placement p) {
		Space occupied = p.occupied;
		
		// H方向与pallet接触
		if (occupied.h1==0) {
			return p.ort.l*p.ort.w;
		}

		double sum = 0;
		for (int i=0; i<P.placedDescH2.length; i++) {
			Space ocp = P.placedDescH2[i].p.occupied;
			if (ocp.h2 == occupied.h1 && ocp.projectionIntersectH(occupied)) {
				sum += occupied.overlapOnL(ocp) * occupied.overlapOnW(ocp);
			}
		}
		
		return sum;
	}

	/////////////////////////////////////////////////////////////
	/**
	 * 如果 p 的L1面与 一个相邻的盒子的L2面完全重叠就返回 1 否则 0
	 * @param P
	 * @param p
	 * @return
	 */
	public static int matchFaceL1(Pallet P, Placement p) {
		Space occupied = p.occupied;
		for (int i=0; i<P.placedDescL2.length; i++) {
			Space ocp = P.placedDescL2[i].p.occupied;
			if (ocp.l2 == occupied.l1 && ocp.projectionIntersectL(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.w2==occupied.w2 && ocp.h1==occupied.h1 && ocp.w1==occupied.w1) { return 1; }
				return 0;
			}
			if (ocp.l2 < occupied.l1) { break; }
		}
		return 0;
	}

	/**
	 * 如果 p 的L2面与一个相邻的盒子的L1面完全重叠就返回 1 否则 0
	 * @param P
	 * @param p
	 * @return
	 */
	public static int matchFaceL2(Pallet P, Placement p) {
		Space occupied = p.occupied;
		for (int i=0; i<P.placedDescL1.length; i++) {
			Space ocp = P.placedDescL1[i].p.occupied;
			if (ocp.l1 == occupied.l2 && ocp.projectionIntersectL(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.w2==occupied.w2 && ocp.h1==occupied.h1 && ocp.w1==occupied.w1) { return 1; }
				return 0;
			}
			if (ocp.l1 < occupied.l2) { break; }
		}
		return 0;
	}

	
	/**
	 * 如果 p 的W1面与一个相邻的盒子的W2面完全重叠就返回 1 否则 0
	 * @param P
	 * @param p
	 * @return
	 */
	public static int matchFaceW1(Pallet P, Placement p) {
		Space occupied = p.occupied;
		for (int i=0; i<P.placedDescW2.length; i++) {
			Space ocp = P.placedDescW2[i].p.occupied;
			if (ocp.w2 == occupied.w1 && ocp.projectionIntersectL(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.l2==occupied.l2 && ocp.h1==occupied.h1 && ocp.l1==occupied.l1) { return 1; }
				return 0;
			}
			if (ocp.w2 < occupied.w1) { break; }
		}
		return 0;
	}
	
	/**
	 * 如果 p 的W2面与一个相邻的盒子的W1面完全重叠就返回 1 否则 0
	 * @param P
	 * @param p
	 * @return
	 */
	public static int matchFaceW2(Pallet P, Placement p) {
		Space occupied = p.occupied;
		for (int i=0; i<P.placedDescW1.length; i++) {
			Space ocp = P.placedDescW1[i].p.occupied;
			if (ocp.w1 == occupied.w2 && ocp.projectionIntersectL(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.l2==occupied.l2 && ocp.h1==occupied.h1 && ocp.l1==occupied.l1) { return 1; }
				return 0;
			}
			if (ocp.w1 < occupied.w2) { break; }
		}
		return 0;
	}
	
	
	/**
	 * 如果 p 的H1面与一个相邻的盒子的H2面完全重叠就返回 1 否则 0
	 * @param P
	 * @param p
	 * @return
	 */
	public static int matchFaceH1(Pallet P, Placement p) {
		Space occupied = p.occupied;
		for (int i=0; i<P.placedDescH2.length; i++) {
			Space ocp = P.placedDescH2[i].p.occupied;
			if (ocp.h2 == occupied.h1 && ocp.projectionIntersectH(occupied)) {
				if (ocp.l2==occupied.l2 && ocp.w2==occupied.w2 && ocp.l1==occupied.l1 && ocp.w1==occupied.w1) { return 1; }
				return 0;
			}
			if (ocp.h2 < occupied.h1) { break; }
		}
		return 0;
	}
	
	/**
	 * 如果 p 的H1面与一个相邻的盒子的H2面完全重叠就返回 1 否则 0
	 * @param P
	 * @param p
	 * @return
	 */
	public static int matchFaceH2(Pallet P, Placement p) {
		Space occupied = p.occupied;
		for (int i=0; i<P.placedDescH1.length; i++) {
			Space ocp = P.placedDescH1[i].p.occupied;
			if (ocp.h1 == occupied.h2 && ocp.projectionIntersectH(occupied)) {
				if (ocp.l2==occupied.l2 && ocp.w2==occupied.w2 && ocp.l1==occupied.l1 && ocp.w1==occupied.w1) { return 1; }
				return 0;
			}
			if (ocp.h1 < occupied.h2) { break; }
		}
		return 0;
	}
	
}
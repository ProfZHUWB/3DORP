package com.zhuwb.research.roboticpacking.search;

import com.zhuwb.research.roboticpacking.inst.Placement;
import com.zhuwb.research.roboticpacking.space.Space;

// FitnessZhouNewType�� �ж������ʱ���º��Ӿ����Ž��Ѿ������ֺ��ӵ������У�
// �� Zhou ���� OpenPalletCount ʵ����ǰ��������û��Ӱ��
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
	
	// �����ӵ� box �� pallet �����в�ͬ���ᵼ�±Ƚϲ�����
	// ����������ͬ��ϣ���� box �Ž��Ѿ���ͬ�� box �� pallet ��
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
	
	// ������ӵ�֧����
	public static double calSupport(Pallet P, Placement p){
		Space occupied = p.occupied;
		// ����ֱ������Pallet�ĵײ�
		if (occupied.h1==0) {
			return 1.0;
		}
		
		double bottom = p.ort.l * p.ort.w;  // ���ӵĵ����
		double sum = 0;
		for (int i=0; i<P.placedDescH2.length; i++) {
			Space ocp = P.placedDescH2[i].p.occupied;
			if (ocp.h2 == occupied.h1 && ocp.projectionIntersectH(occupied)) {
				sum += occupied.overlapOnL(ocp) * occupied.overlapOnW(ocp);
			}
		}
		return sum/bottom;
	}
	
	// ������L�᷽����������
	public static double calRegularityL(Pallet P, Placement p, double hweight) {
		Space occupied = p.occupied;
		double sum = 0;
		for (int i=0; i<P.placedDescL2.length; i++) {
			Space ocp = P.placedDescL2[i].p.occupied;
			if (ocp.l2 == occupied.l1 && ocp.projectionIntersectL(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.h1==occupied.h1){     // H��������Ҫ����һ��zweight
					sum+= hweight*occupied.overlapOnW(ocp) * occupied.overlapOnH(ocp);
				}
				if (ocp.w2==occupied.w2 && ocp.w1==occupied.w1) {  // W����������Ҫ����һ��zweight
					sum+= occupied.overlapOnW(ocp) * occupied.overlapOnH(ocp);
				}
			}
		}
		return sum;
	}
	
	// ������W�᷽����������
	public static double calRegularityW(Pallet P, Placement p, double hweight) {
		Space occupied = p.occupied;
		double sum = 0;
		for (int i=0; i<P.placedDescW2.length; i++) {
			Space ocp = P.placedDescW2[i].p.occupied;
			if (ocp.w2 == occupied.w1 && ocp.projectionIntersectW(occupied)) {
				if (ocp.h2==occupied.h2 && ocp.h1==occupied.h1){     // H��������Ҫ����һ��zweight
					sum+= hweight*occupied.overlapOnL(ocp) * occupied.overlapOnH(ocp);
				}
				if (ocp.l2==occupied.l2 && ocp.l1==occupied.l1) {  //  L����������Ҫ����һ��zweight
					sum+= occupied.overlapOnL(ocp) * occupied.overlapOnH(ocp);
				}
			}
		}
		return sum;
	}
	
	// ������H�᷽����������
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
		
		// l������pallet�Ӵ�
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
		
		// w������pallet�Ӵ�
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
		
		// H������pallet�Ӵ�
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
	 * ��� p ��L1���� һ�����ڵĺ��ӵ�L2����ȫ�ص��ͷ��� 1 ���� 0
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
	 * ��� p ��L2����һ�����ڵĺ��ӵ�L1����ȫ�ص��ͷ��� 1 ���� 0
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
	 * ��� p ��W1����һ�����ڵĺ��ӵ�W2����ȫ�ص��ͷ��� 1 ���� 0
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
	 * ��� p ��W2����һ�����ڵĺ��ӵ�W1����ȫ�ص��ͷ��� 1 ���� 0
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
	 * ��� p ��H1����һ�����ڵĺ��ӵ�H2����ȫ�ص��ͷ��� 1 ���� 0
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
	 * ��� p ��H1����һ�����ڵĺ��ӵ�H2����ȫ�ص��ͷ��� 1 ���� 0
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
package com.zhuwb.research.roboticpacking.inst;

import java.util.ArrayList;
import java.util.Arrays;

import com.zhuwb.research.roboticpacking.space.Space;

public class BoxType {
	public static class Orientation {
		public int l, w, h; // box �� L-, W-, H- �ϵĴ�С
		public int ort;
		public int maxDropH;
		
		public Orientation(int l, int w, int h, int ort, double maxDropHeightRatio) {
			switch(ort) {
			case 0: this.l = l; this.w = w; this.h = h; break;
			case 1: this.l = l; this.w = h; this.h = w; break;
			case 2: this.l = w; this.w = l; this.h = h; break;
			case 3: this.l = w; this.w = h; this.h = l; break;
			case 4: this.l = h; this.w = l; this.h = w; break;
			case 5: this.l = h; this.w = w; this.h = l; break;
			default: throw new IllegalArgumentException("ort: "+ort+" not in {0,1,2,3,4,5}");
			}
			this.ort = ort;
			this.maxDropH = (int) (this.h * maxDropHeightRatio);
		}
		
		@Override
		public boolean equals(Object o) {
			if (!(o instanceof Orientation)) { return false; }
			
			Orientation ort = (Orientation) o;
			return this.l == ort.l && this.w == ort.w && this.h == ort.h;
		}
		
		// [pushAxis][align], ��pushAxis�����Ͳ�ȡalign��ʽ�ڷ�ץ��ʱ��ץ�����������ϵ�ͶӰ
		// ��һ����άspace������ά������������ͬ����ʾһ����άͶӰ
		public Space[][] gripperProjection;
		
		public String toString() {
			return "ort: "+ort+"; ("+l+" "+w+" "+h+"); maxDropH: "+maxDropH;
		}
		
		public double getVolume() {
			return ((double)l)*w*h;
		}
	}
	
		
	
	public int idx;
	public Orientation[] distinctOrt;
	
	public BoxType(int idx, int l, int w, int h, boolean[] permittedOrt, Vacuum gripper, double maxDropHeightRatio) {
		this.idx = idx;
		ArrayList<Orientation> ortList = new ArrayList<>();
		for (int i=0; i<permittedOrt.length; i++) {
			if (!permittedOrt[i]) { continue; }
			
			Orientation ort = new Orientation(l, w, h, i, maxDropHeightRatio);
			boolean exist = false;
			for (Orientation existOrt:ortList) {
				if (existOrt.equals(ort)) {
					exist = true;
					break;
				}
			}
			if (!exist) {
				ortList.add(ort);
				ort.gripperProjection = gripper.computeGripperProjection(ort);
			}
		}
		this.distinctOrt = ortList.toArray(new Orientation[0]);
	}
	
	public double getVolume() {
		return this.distinctOrt[0].getVolume();
	}
	
	public String toString() {
		return "idx: "+idx+"; distictOrt: "+distinctOrt.length+" "+Arrays.toString(distinctOrt);
	}
}

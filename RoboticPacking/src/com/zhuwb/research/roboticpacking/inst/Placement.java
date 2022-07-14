package com.zhuwb.research.roboticpacking.inst;

import java.io.PrintStream;

import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;
import com.zhuwb.research.roboticpacking.inst.Vacuum.Align;
import com.zhuwb.research.roboticpacking.inst.Vacuum.PushAxis;
import com.zhuwb.research.roboticpacking.space.Space;

// ���� placement �ȼ���� �������͡�ort���ڷ�λ��һ�¡�
// �ڷ�λ�ý��Ƽ���Ϊ releaseL, releaseW, spaceH (ʵ���Ͽ��ܽ������)
public class Placement {
	public int boxType;			// ��װ�ĺ�������
	public Orientation ort;		// ��װ���ӵ���ת��ʽ�������˺����� L- W- H-��ռ�õĳ���
	public int releaseL, releaseW, releaseH; // ץ���ͷŴ�װ����ʱ��������ԭ��������������	
	
	public int dh;			// ץ���ͷŴ�װ����ʱ������������λ�õĴ�ֱ׹�����
	public int spaceH;		// �������ڿռ�ĵ���߶�
	
	public PushAxis pushAxis;	// ץ���� pushAxis ���ʹ�װ���ӵ�
	public Align align;
	public Space occupied;		// ��װ���Ӵ�������λ��ռ�õĿռ�
	public Space boxPath;		// ����
	public Space gripperPath;
	public Space dropPath;
	
	public String toString() {
		return "boxType: "+boxType+"; ort:"+ort.ort+"; release at: ("+releaseL+" "+releaseW+" "+releaseH+")"+"; dh: "+dh+"; spaceH: "+spaceH+"; pushAxis: "+pushAxis+"; align: "+align;
	}
	
	public void print(String linePrefix, PrintStream ps) {
		ps.println(linePrefix+this.toString());
		ps.println(linePrefix+"box size: "+ort.l+" "+ort.w+" "+ort.h);
		ps.println(linePrefix+"pushAxis: "+pushAxis+"; align: "+align);
		ps.println(linePrefix+"occupied: "+occupied);
		ps.println(linePrefix+"occupied before drop: "+occupied.translate(0, 0, dh));
		ps.println(linePrefix+"dropPath: "+dropPath);
		ps.println(linePrefix+"boxPath: "+boxPath);
		ps.println(linePrefix+"gripperPath: "+gripperPath);
	}
	
	public void computeOccupiedAndDrop() {
		Placement p = this;
		int endL = p.releaseL + ort.l;
		int endW = p.releaseW + ort.w;
		int endH = p.releaseH + ort.h;
		p.dropPath = new Space(p.releaseL,p.releaseW,endH-p.dh,
				               endL,endW,endH);
		p.occupied = new Space(p.releaseL,p.releaseW,p.releaseH-p.dh,
								endL, endW, endH-p.dh);
	}

	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + boxType;
		result = prime * result + ort.ort;
//		result = prime * result + releaseH;
		result = prime * result + spaceH;
		result = prime * result + releaseL;
		result = prime * result + releaseW;
		return result;
	}



	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Placement other = (Placement) obj;
		if (boxType != other.boxType)
			return false;
		if (ort != other.ort)
			return false;
		if (spaceH != other.spaceH)
			return false;
//		if (releaseH != other.releaseH)
//			return false;
		if (releaseL != other.releaseL)
			return false;
		if (releaseW != other.releaseW)
			return false;
		return true;
	}

	public void computePathsForDebug(Vacuum gripper, Space pallet) {
		this.computeOccupiedAndDrop();
		switch (this.pushAxis) {
		case L: this.gripperPath = gripper.gripperPathL(this.releaseL,this.releaseW,this.releaseH, 
														this.ort, pallet.l2, this.align);
				break;
		case W: this.gripperPath = gripper.gripperPathW(this.releaseL,this.releaseW,this.releaseH, 
														this.ort, pallet.w2, this.align);
				break;
		case H: this.gripperPath = gripper.gripperPathH(this.releaseL,this.releaseW,this.releaseH, 
														this.ort, pallet.h2, this.align);
				break;
		default: throw new IllegalArgumentException("pushAxis: "+pushAxis);
		}
		this.boxPath = Vacuum.computeBoxPath(this.releaseL, this.releaseW, this.releaseH, this.ort,
				pallet, this.pushAxis);
	}


	public static void main(String[] args) {
//		Hashtable<Placement, Placement> loc2placement = new Hashtable<>();
//		Placement p1 = new Placement();
//		p1.boxType = 1;
//		p1.ort = 2;
//		p1.releaseL = 1;
//		p1.releaseW = 2;
//		p1.releaseH = 3;
//		loc2placement.put(p1, p1);
//		Placement p = loc2placement.get(p1);
//		System.out.println(p);
	}
}

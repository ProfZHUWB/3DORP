package com.zhuwb.research.roboticpacking.inst;

public class SystemInfo {
	public final InstData inst;
	public final BoxType[] boxTypes;
	public final Vacuum gripper;
	public final SysConfig conf;
	public final int[] boxCountPerType; // [t] 第t种盒子待装数量
	public final double totalBoxVolume;
	public final double palletVolume; // loading volume of a pallet
	
	
	public SystemInfo(InstData inst, SysConfig conf) {
		this.inst = inst;
		this.conf = conf;
		this.palletVolume = this.inst.getPalletVolume();
		this.gripper = Vacuum.create(conf.theta, conf.vaccumType);
		this.boxTypes = new BoxType[inst.boxType.length];
		for (int t=0; t<inst.boxType.length; t++) {
			int[] box = inst.boxType[t];
			this.boxTypes[t] = new BoxType(t, box[0], box[1], box[2], inst.ortPerm[t],gripper, conf.maxDropHeightRatio);
		}
		this.boxCountPerType = inst.getBoxCountPerType();
		double totalBoxVolume = 0;
		for (int t=0; t<this.boxCountPerType.length; t++) {
			totalBoxVolume += boxTypes[t].getVolume() * this.boxCountPerType[t];
		}
		this.totalBoxVolume = totalBoxVolume;
	}
}
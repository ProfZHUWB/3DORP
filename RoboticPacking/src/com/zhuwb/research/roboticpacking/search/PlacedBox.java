package com.zhuwb.research.roboticpacking.search;

import com.zhuwb.research.roboticpacking.inst.Placement;
import com.zhuwb.research.roboticpacking.space.Space;

public class PlacedBox {
	public Placement p;
	public Space occupied;
	
	public PlacedBox prev; // 已装盒子列表中this前面的一个; null表示当前盒子是第一个

	public PlacedBox(Placement p, PlacedBox prev) {
		this.p = p;
		this.occupied = p.occupied;
		this.prev = prev;
	}
	
	public String toString() {
		return p.toString()+"; occupied: "+occupied;
	}
}

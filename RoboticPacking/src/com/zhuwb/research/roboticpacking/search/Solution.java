package com.zhuwb.research.roboticpacking.search;

import java.io.File;
import java.io.IOException;

import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.Placement;

public class Solution {
	public State s;
	public Pallet[] closed;
	public Pallet[] open;
	
	public Solution(State s) {
		super();
		this.s = s;
		this.closed = s.getClosedPallets();
		int nonEmpty = 0;
		for (int i=0; i<s.open.length; i++) {
			if (s.open[i].isEmpty()) { continue; }
			nonEmpty++;
		}
		this.open = new Pallet[nonEmpty];
		int idx = 0;
		for (int i=0; i<s.open.length; i++) {
			if (s.open[i].isEmpty()) {
				continue;
			}
			this.open[idx++] = s.open[i];
		}
	}
	
	public double averageUtilClosed() {
		if (this.closed.length == 0) { return 0; }
		
		double boxVolume = 0;
		for (int i=0; i<this.closed.length; i++) {
			boxVolume += this.closed[i].loadedVolume;
		}
		return boxVolume / s.sysInfo.inst.L / s.sysInfo.inst.W / s.sysInfo.inst.H / this.closed.length; 
	}
	
	public double averageUtil() {
		double boxVolume = 0;
		for (int i=0; i<this.closed.length; i++) {
			boxVolume += this.closed[i].loadedVolume;
		}
		for (int i=0; i<this.open.length; i++) {
			boxVolume += this.open[i].loadedVolume;
		}
		int palletCount = this.closed.length + this.open.length;
		return boxVolume / s.sysInfo.inst.L / s.sysInfo.inst.W / s.sysInfo.inst.H / palletCount; 		
	}
	
	public int countLoadedBoxes() {
		int count = 0;
		for (int i=0; i<this.closed.length; i++) {
			count += this.closed[i].getPlacements().length;
		}
		for (int i=0; i<this.open.length; i++) {
			count += this.open[i].getPlacements().length;
		}
		return count;
	}
	
	public void validate() {
		InstData inst = s.sysInfo.inst;
		int[] boxCountPerType = inst.getBoxCountPerType();
		int[] loadedBoxCountPerType = new int[inst.getBoxTypeCount()];

		Pallet[] pallets = new Pallet[this.closed.length+this.open.length];
		for (int i=0; i<this.closed.length; i++) {
			pallets[i] = this.closed[i];
		}
		for (int i=0; i<this.open.length; i++) {
			pallets[this.closed.length+i] = this.open[i];
		}

		for (int i=0; i<pallets.length; i++) {
			Placement[] placements = pallets[i].getPlacements();
			for (int j=0; j<placements.length; j++) {
				Placement p = placements[j];
				loadedBoxCountPerType[p.boxType] += 1;
			}
			pallets[i].validate();
		}
		
		int totalLoadedBoxCount = 0;
		for (int t=0; t<inst.getBoxTypeCount(); t++) {
			if (loadedBoxCountPerType[t] != boxCountPerType[t]) {
				throw new RuntimeException(
					String.format("box type %i, loadedCount:  %i != actualCount: %i", 
							t, loadedBoxCountPerType[t], boxCountPerType[t]));
			}
			totalLoadedBoxCount += loadedBoxCountPerType[t];
		}
		if (totalLoadedBoxCount != inst.getBoxCount()) {
			throw new RuntimeException(
					String.format("loaded box count:  %i != actual count: %i", 
							totalLoadedBoxCount, inst.getBoxCount()));
		}
	}
	
	
	public void draw(File dir, boolean debug) throws IOException {
		for (int i=0; i<closed.length; i++) {
			closed[i].draw(new File(dir, "closed-"+i+".nb"), null);
			if (debug) {
				closed[i].drawPlacementsAndSpaces(new File(dir, "closed-"+i));
			}
		}
		for (int i=0; i<open.length; i++) {
			open[i].draw(new File(dir, "open-"+i+".nb"), null);
			if (debug) {
				open[i].drawPlacementsAndSpaces(new File(dir, "open-"+i));
			}
		}
	}
	
}

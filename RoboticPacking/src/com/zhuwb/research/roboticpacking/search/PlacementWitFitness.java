package com.zhuwb.research.roboticpacking.search;

import java.util.Arrays;

import com.zhuwb.research.roboticpacking.inst.Placement;

public class PlacementWitFitness implements Comparable<PlacementWitFitness>{
	private final double palletFitness; // 由 state 综合几个 pallet 决定
	public final double[] fitness;
	public final int opIdx; // index of open pallet
	public final Placement placement;


	public PlacementWitFitness(double palletFitness, double[] fitness, int openIdx, Placement placement) {
		super();
		this.palletFitness = palletFitness;
		this.fitness = fitness;
		this.opIdx = openIdx;
		this.placement = placement;
	}

	@Override
	public int compareTo(PlacementWitFitness o) {
		if (this.palletFitness < o.palletFitness) { return -1; }
		if (this.palletFitness > o.palletFitness) { return 1; }
		for(int i=0;i<this.fitness.length;i++) {
			if (fitness[i]<o.fitness[i]) {return -1;}
			if (fitness[i]>o.fitness[i]) {return 1;}
		}
		if (this.opIdx<o.opIdx) {return -1;}
		if (this.opIdx>o.opIdx) {return 1;}
		return 0;
	}
	

	public String toString() {
		return "palletFitness: "+palletFitness+"; fitness: "+Arrays.toString(fitness)+"; p: " + placement + "; occupied: " + placement.occupied;
	}
}
package com.zhuwb.research.roboticpacking.inst;

import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.TreeSet;

import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;
import com.zhuwb.research.rpp3i.sclp.common.KnapsackFitnessCalc;

public class KnapsackUB {
	public static void increaseBoxCount(TreeMap<Integer, Integer> l2count, TreeSet<Integer> lengths, int boxCount) {
		for (int len:lengths) {
			Integer existing = l2count.get(len);
			if (existing == null) {
				existing = boxCount;
			} else {
				existing = existing + boxCount;
			}
			l2count.put(len,  existing);
		}
	}

	static int[] keys;
	static int[] values;
	public static void toArrays(TreeMap<Integer, Integer> l2count) {
		keys = new int[l2count.size()];
		values = new int[l2count.size()];
		int i=0;
		for (Entry<Integer, Integer> e:l2count.entrySet()) {
			keys[i] = e.getKey();
			values[i] = e.getValue();
			i++;
		}
	}
	
	public static double upperBound(SystemInfo sysInfo) {
		InstData inst = sysInfo.inst;
		int[] countPerType = inst.getBoxCountPerType();
		TreeMap<Integer, Integer> l2count = new TreeMap<>();
		TreeMap<Integer, Integer> w2count = new TreeMap<>();
		TreeMap<Integer, Integer> h2count = new TreeMap<>();
		TreeMap<Integer, Integer> v2count = new TreeMap<>();
		
		for (int t=0; t<sysInfo.boxTypes.length; t++) {
			BoxType bt = sysInfo.boxTypes[t];
			TreeSet<Integer> lengths = new TreeSet<>();
			TreeSet<Integer> widths = new TreeSet<>();
			TreeSet<Integer> heights = new TreeSet<>();
			
			for (Orientation ort:bt.distinctOrt) {
				lengths.add(ort.l);
				widths.add(ort.w);
				heights.add(ort.h);
			}
			
			int boxCount = countPerType[t];
			increaseBoxCount(l2count, lengths, boxCount);
			increaseBoxCount(w2count, widths, boxCount);
			increaseBoxCount(h2count, heights, boxCount);
			
			if (bt.getVolume() > Integer.MAX_VALUE) { throw new RuntimeException(); }
			int volume = ((int) bt.getVolume());
			Integer existing = v2count.get(volume);
			if (existing == null) {
				existing = boxCount;
			} else {
				existing = existing + boxCount;
			}
			v2count.put(volume, existing);
		}

		toArrays(l2count);
		int[] usableLength = KnapsackFitnessCalc.knapsack(inst.L, keys, values);
		int uL = usableLength[inst.L];		
		toArrays(w2count);
		int[] usableWidth = KnapsackFitnessCalc.knapsack(inst.W, keys, values);
		int uW = usableWidth[inst.W];
		toArrays(h2count);
		int[] usableHeight = KnapsackFitnessCalc.knapsack(inst.H, keys, values);
		int uH = usableHeight[inst.H];

		if (((double) uL)*uW*uH > Integer.MAX_VALUE) { throw new RuntimeException(); }
		int reducedPalletVolume = uL * uW * uH;
		toArrays(v2count);
		int[] usableVolume = KnapsackFitnessCalc.knapsack(reducedPalletVolume, keys, values);
		int uV = usableVolume[reducedPalletVolume];
		
		if (sysInfo.inst.name.equals("SD1-2-200-small")) {
			System.out.println("l2count: "+l2count);
			System.out.println("w2count: "+w2count);
			System.out.println("h2count: "+h2count);
			System.out.println("uL: "+uL);
			System.out.println("uW: "+uW);
			System.out.println("uH: "+uH);
			System.out.println("v2count: "+v2count);
			System.out.println("reducedPalletVolume: "+reducedPalletVolume);
			System.out.println("uV: "+uV);
		}
		
		return ((double) uV) / inst.getPalletVolume();
	}

}

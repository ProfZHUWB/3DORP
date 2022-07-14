package com.zhuwb.research.roboticpacking.space;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import com.zhuwb.research.roboticpacking.inst.BoxType;
import com.zhuwb.research.roboticpacking.inst.BoxType.Orientation;

/**
 * Manage all free spaces inside a pallet
 * Store spaces as ArrayList and update in place.
 * 
 * @author iwenc
 */
public class ArrayListInPlaceSpaceManager {
		
	public static class SpaceChecker {
		public final BoxType[] boxTypes;
		public final int[] remainingBoxCount; // [t] number of boxes to be load in the future
		
		public SpaceChecker(BoxType[] boxTypes, int[] remainingBoxCount) {
			super();
			this.boxTypes = boxTypes;
			this.remainingBoxCount = remainingBoxCount;
		}
		
		/**
		 * 生成一个新的checker，它的剩余盒子比this少一个
		 * @param boxType
		 * @return
		 */
		public SpaceChecker removeOneBox(int boxType) {
			int[] remainingBoxCount = Arrays.copyOf(this.remainingBoxCount, this.remainingBoxCount.length);
			remainingBoxCount[boxType] -= 1;
			return new SpaceChecker(this.boxTypes, remainingBoxCount);
		}

		/**
		 * If s is too small to accommodate any of a remaining box it can never be used in future loading.
		 * A space is useful only if it is large enough to accommodate at least one future box. 
		 */
		public boolean useful(Space s) {
			for (int t=0; t<this.boxTypes.length; t++) {
				if (remainingBoxCount[t] == 0) { continue; }
				
				for (Orientation ort: this.boxTypes[t].distinctOrt) {
					if (ort.l <= s.dL && ort.w <= s.dW && ort.h <= s.dH) {
						return true;
					}
				}
			}
			return false;
		}
	}
	
	private ArrayList<Space> spaceList;
	private SpaceChecker checker;
	public SpaceChecker getSpaceChecker() {
		return this.checker;
	}
	
	/**
	 * Create a manager that manages all free spaces inside s
	 * @param s
	 * @param checker	Check if a generated space is useful
	 */
	public ArrayListInPlaceSpaceManager(Space s, SpaceChecker checker) {
		this.spaceList = new ArrayList<Space>();
		this.checker = checker;
		this.spaceList.add(s);
	}
	
	private ArrayListInPlaceSpaceManager(ArrayList<Space> spaceList, SpaceChecker checker) {
		this.spaceList = spaceList;
		this.checker = checker;
	}
	public ArrayListInPlaceSpaceManager copyAfterRemoveOneBox(int boxType) {
		SpaceChecker checker = this.checker.removeOneBox(boxType);
		ArrayList<Space> spaces = new ArrayList<Space>(this.spaceList.size());
		for (Space s:this.spaceList) {
			if (checker.useful(s)) {
				spaces.add((Space)s.clone());
			}
		}
		return new ArrayListInPlaceSpaceManager(spaces,checker);
	}
	
	/**
	 * Cut space s into smaller space after part of it is occupied occupied
	 * Add resulting smaller spaces into mList
	 * @param s
	 * @param occupied
	 * @param mList
	 */
	protected static void cut(Space s, Space occupied, ArrayList<Space>mList)
	{
		if (s.l1 != occupied.l1) {
			mList.add(new Space(s.l1, s.w1, s.h1, occupied.l1, s.w2, s.h2)); //, Max_Corner_Count));
		}
		if (s.l2 != occupied.l2) {
			mList.add(new Space(occupied.l2, s.w1, s.h1, s.l2, s.w2, s.h2));  //, Max_Corner_Count));
		}
		if (s.w1 != occupied.w1) {
			mList.add(new Space(s.l1, s.w1, s.h1, s.l2, occupied.w1, s.h2)); //, Max_Corner_Count));
		}
		if (s.w2 != occupied.w2) {
			mList.add(new Space(s.l1, occupied.w2, s.h1, s.l2, s.w2, s.h2)); //, Max_Corner_Count));
		}
		if (s.h1 != occupied.h1) {
			mList.add(new Space(s.l1, s.w1, s.h1, s.l2, s.w2, occupied.h1)); //, Max_Corner_Count));
		}
		if (s.h2 != occupied.h2) {
			mList.add(new Space(s.l1, s.w1, occupied.h2, s.l2, s.w2, s.h2)); //, Max_Corner_Count));
		}
	}
	


	private Comparator<Space>volumeComparator = new Comparator<Space>() {
		public int compare(Space a, Space b) {
			double comp = b.volume - a.volume;
			if (comp < 0) {
				return -1;
			} else if (comp > 0) {
				return 1;
			} else {
				return 0;
			}
		}
	};

	/**
	 * Update the list of spaces after taken is occupied
	 * All spaces that overlap with taken in dividied into smaller spaces
	 * Each new space is added into manager if it is useful according to checker.
	 * @param taken
	 */
	public void updateSpaces(Space taken) {
		//Space taken = new Space(pb.x, pb.y, pb.z, pb.x + pb.block.length, pb.y + pb.block.width, pb.z + pb.block.height); //, maxCornerPerSpace);
		
		ArrayList<Space> deletedSpaces = this.deleteOverlap(taken);
		
		ArrayList<Space>newSpaces = new ArrayList<Space>();

		for (Space space : deletedSpaces) {
			Space is = space.intersect(taken);
			cut(space, is, newSpaces);
		}
		
		//O(klogk)
		Collections.sort(newSpaces, volumeComparator);
		
		ArrayList<Space> validNewSpace = new ArrayList<Space>(newSpaces.size());
		//O(k*k)
		for (Space space : newSpaces) {
			if (checker.useful(space)) {
				validNewSpace.add(space);
			}
		}
		
		this.insert(validNewSpace);		
	}
	
	/**
	 * Delete all spaces that overlap with s and return deleted spaces
	 * @param s
	 * @return
	 */
	protected ArrayList<Space> deleteOverlap(Space s) {
		ArrayList<Space> result = new ArrayList<Space>();
		for (int i=0; i<spaceList.size();) {
			Space space = spaceList.get(i);
			if (space.intersectTest(s)) {
				result.add(space);
				
				int last = spaceList.size()-1;
				if (i<last) {
					spaceList.set(i, spaceList.get(last));
				}
				spaceList.remove(last);
			} else {
				i++;
			}
		}

		return result;
	}
	
	
	/**
	 * Check if any spaces in this manager contains the given space
	 * @param space
	 * @return
	 */
	protected boolean contains(Space space) {
		for (Space s : spaceList) {
			if (s.contains(space)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Insert given space into this manager if it is not contained in any other spaces
	 * @param space
	 */
	protected void insert(Space space) {
		if (!contains(space)) {
			spaceList.add(space);
		}
	}
	
	/**
	 * Insert all spaces that are not contained in any other spaces in this manager into this manager.
	 * @param spaceList
	 */
	protected void insert(ArrayList<Space> spaceList) {
		// delete contained space first
		spaceList = deleteContained(spaceList);

		for (Space s:spaceList) {
			insert(s);
		}
	}
	
	protected static ArrayList<Space> deleteContained(ArrayList<Space> spaceList) {
		ArrayList<Space> result = new ArrayList<Space>();
		for (int i=0; i<spaceList.size(); i++) {
			Space s1 = spaceList.get(i);
			boolean contained = false;
			for (int j=0; j<spaceList.size(); j++) {
				if (j!=i) {
					Space s2 = spaceList.get(j);
					if (s2.contains(s1)) {
						contained = true;
						break;
					}
				}
			}
			if (!contained) {
				result.add(s1);
			}
		}
		return result;
	}

	public ArrayList<Space> getFreeSpace() {
		return this.spaceList;
	}

//	public void deleteSpace(Space s) {
//		for (int i=0; i<spaceList.size();) {
//			Space space = spaceList.get(i);
//			if (space == s || space.equals(s)) {
//				int last = spaceList.size()-1;
//				if (i<last) {
//					spaceList.set(i, spaceList.get(last));
//				}
//				spaceList.remove(last);
//				break;
//			} else {
//				i++;
//			}
//		}
//	}
//
//	public void deleteSpace(ArrayList<Space> sList) {
//		for (Space s:sList) {
//			deleteSpace(s);
//		}
//	}


}

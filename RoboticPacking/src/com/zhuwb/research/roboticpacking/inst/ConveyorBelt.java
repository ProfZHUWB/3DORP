package com.zhuwb.research.roboticpacking.inst;

/**
 * 仿真盒子在传送带上依次到达机器手范围
 * 当机器手从范围中取走一个盒子，后面的盒子依次向前移动一个位置。
 * 也可以用来模拟托盘被关闭移走后，后面的空托盘从传送带向前依次移动一个位置。
 * @author iwenc
 *
 */
public class ConveyorBelt implements Cloneable {
	ConveyorBelt prev;
	int removedIdx;
	
	public final int[] idxInRange;	// [i] is the index of box or pallet (as in arrival squence)
	public int idxInRangeCount;
	public int nextIdx;
	Integer maxIdx;
	
	
	/**
	 * 创建一个传送带
	 * @param range		机器手的操作范围
	 * @param maxIdx	总共盒子的个数，从0开始编号；null 表示有无限多个（用来模拟托盘）
	 */
	public ConveyorBelt(int range, Integer maxIdx) {
		this.idxInRange = new int[range];
		this.idxInRangeCount = range;
		if (maxIdx != null && maxIdx < this.idxInRangeCount) { idxInRangeCount = maxIdx; }
		for (int i=0; i<idxInRangeCount; i++) {
			this.idxInRange[i] = i;
		}
		this.nextIdx = this.idxInRangeCount;
		this.maxIdx = maxIdx;
	}
	
	/**
	 * Remove the box at the idx-th position in the operation range
	 * @param idx
	 * @return
	 */
	public ConveyorBelt remove(int idx) {
		if (idx >= this.idxInRangeCount) { throw new IllegalArgumentException("idx: "+idx+" must < "+this.idxInRangeCount); }
		
		ConveyorBelt next = null;
		try {
			next = (ConveyorBelt) this.clone();
		} catch (CloneNotSupportedException e) { throw new RuntimeException(e); }

		next.prev = this;
		next.removedIdx = this.idxInRange[idx];
//		System.out.println("removed idx: "+next.removedIdx);
		for (int i=idx; i<this.idxInRangeCount-1; i++) {
			next.idxInRange[i] = next.idxInRange[i+1];
		}
//		System.out.println("... next.idxInRange: "+Arrays.toString(next.idxInRange));
		
		// there are more incoming boxes, replace box at idx by incoming one
		if (next.maxIdx == null || next.nextIdx < next.maxIdx) {
			next.idxInRange[next.idxInRangeCount-1] = next.nextIdx;
			next.nextIdx += 1;
		} else {
			next.idxInRangeCount -= 1;
		}
		
		return next;
	}
	
	/**
	 * @param activeOpenIdx	要标记的活跃位置；-1：不用标记活跃位置 
	 * @param maxRemoved    null 打印所有已经移除的; 否则只打印最多 maxRemoved 个
	 * @return
	 */
	public String toString(int activeOpenIdx, Integer maxRemoved) {
		if (activeOpenIdx >= this.idxInRangeCount) {
			throw new IllegalArgumentException("activeOpenIdx: "+activeOpenIdx+" must < "+this.idxInRangeCount);
		}
		int removedCount = this.nextIdx - this.idxInRangeCount;
		boolean skipSomeRemoved = false;
		if (maxRemoved != null && removedCount > maxRemoved) {
			removedCount = maxRemoved;
			skipSomeRemoved = true;
		}
		int[] removed = new int[removedCount];
		int i = removedCount - 1;
		ConveyorBelt cur = this;
		while (cur.prev != null && i>=0) {
			removed[i--] = cur.removedIdx;
			cur = cur.prev;
		}
		StringBuffer sb = new StringBuffer();
		if (skipSomeRemoved) {
			sb.append("... ");
		}
		for (i=0; i<removed.length; i++) {
			if (i>0) { sb.append(' '); }
			sb.append(Integer.toString(removed[i]));
		}
		if (removed.length > 0) { sb.append(' '); }
		sb.append('{');
		for (i=0; i<this.idxInRangeCount; i++) {
			if (i>0) { sb.append(' '); }
			if (activeOpenIdx >= 0 && i == activeOpenIdx) {
				sb.append("("+this.idxInRange[i]+")");
			} else {
				sb.append(Integer.toString(this.idxInRange[i]));
			}
		}
		sb.append('}');
		if (this.maxIdx == null) {
			sb.append(" "+this.nextIdx+" ...");
		} else if (this.nextIdx < this.maxIdx) {
			sb.append(" "+this.nextIdx+" ... "+(this.maxIdx-1)+"");
		}
		return sb.toString();
	}

	public static void main(String[] args) {
		ConveyorBelt cb = new ConveyorBelt(3, 10);
		System.out.println(cb.toString(1,null));
		cb = cb.remove(1);
		System.out.println(cb.toString(2,null));
		cb = cb.remove(2);
		System.out.println(cb.toString(0,null));
		cb = cb.remove(0);
		System.out.println(cb.toString(1,null));
		for (int i=6; i<10; i++) {
			cb = cb.remove(1);
			System.out.println(cb.toString(1,null));
		}
		cb = cb.remove(1);
		System.out.println(cb.toString(1,null));
		cb = cb.remove(1);
		System.out.println(cb.toString(-1,null));
		cb = cb.remove(0);
		System.out.println(cb.toString(-1,null));
		
	}

}

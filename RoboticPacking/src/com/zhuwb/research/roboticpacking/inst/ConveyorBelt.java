package com.zhuwb.research.roboticpacking.inst;

/**
 * ��������ڴ��ʹ������ε�������ַ�Χ
 * �������ִӷ�Χ��ȡ��һ�����ӣ�����ĺ���������ǰ�ƶ�һ��λ�á�
 * Ҳ��������ģ�����̱��ر����ߺ󣬺���Ŀ����̴Ӵ��ʹ���ǰ�����ƶ�һ��λ�á�
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
	 * ����һ�����ʹ�
	 * @param range		�����ֵĲ�����Χ
	 * @param maxIdx	�ܹ����ӵĸ�������0��ʼ��ţ�null ��ʾ�����޶��������ģ�����̣�
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
	 * @param activeOpenIdx	Ҫ��ǵĻ�Ծλ�ã�-1�����ñ�ǻ�Ծλ�� 
	 * @param maxRemoved    null ��ӡ�����Ѿ��Ƴ���; ����ֻ��ӡ��� maxRemoved ��
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

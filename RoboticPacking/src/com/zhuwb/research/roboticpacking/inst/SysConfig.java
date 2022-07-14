package com.zhuwb.research.roboticpacking.inst;

import java.io.PrintStream;

import com.zhuwb.research.roboticpacking.inst.Vacuum.Type;

public class SysConfig implements Cloneable {
	
	public Type vaccumType = Type.TwoByThree; // ��е�����̵ĳ���������
	public double theta = 1.0;                // ��е�ִ�С�����ű���
	public double maxDropHeightRatio = 0.1;   // һ�����ӿ��԰�ȫ���������������H-����10%�ĳ���

	// ���� b �Ժ��� a �ṩ֧�ֵ�������
	//     a.h1 - b.h2 <= maxGapForSupport  ���ǵ�һЩ��װ���Ͽ��ܻ���С�α䣬�߶Ȳ��ʱ���µĺ��ӿ���֧�������
	// ����a�ĵײ��ֳ���ȵ�4��
	//   A1 A2
	//   A3 A4
	// ����b��Ai�� L-���ϵ��ص�����Ҫ�ﵽ minOverlapRatioForSupport * (a.l2-a.l1)
	//        ���� W-���ϵ��ص�����Ҫ�ﵽ minOverlapRatioForSupport * (a.w2-a.w1)
	// ��b��Ai �ṩ�㹻֧��
	// ����a��4���ֵ�����>=3�����֧����a���ȶ���
	public double maxHGapForSupport = 0.5;
	public double minOverlapRatioForSupport = 0.2;  // Zhou��alpha

	// changeable:
	public int knownBoxCount = 5;             // known ϵͳ֪��δ������Ϣ����, N_k
	public int openPalletCount = 3;           // pn װ��ʱͬʱ�򿪵���������, P
	public int boxCountInRange = 3;           //oper ��е�ֿ��Բ����ĸ���, N_b

	public static String getHeader() {
		return "vaccumType,theta,maxDropHeightRatio,maxHGapForSupport,minOverlapRatioForSupport,knownBoxCount,openPalletCount,boxCountInRange";
	}
	public String toString() {
		return vaccumType+","+theta +","+maxDropHeightRatio+","+maxHGapForSupport+","+minOverlapRatioForSupport+","+knownBoxCount+","+openPalletCount+","+boxCountInRange;
	}
	
	public void print(PrintStream ps, String linePrefix) {
		ps.println(linePrefix+"vaccumType: "+vaccumType);
		ps.println(linePrefix+"theta: "+theta);
		ps.println(linePrefix+"maxDropHeightRatio: "+maxDropHeightRatio);
		ps.println(linePrefix+"maxHGapForSupport: "+maxHGapForSupport);
		ps.println(linePrefix+"minOverlapRatioForSupport: "+minOverlapRatioForSupport);
		ps.println(linePrefix+"knownBoxCount: "+knownBoxCount);
		ps.println(linePrefix+"openPalletCount: "+openPalletCount);
		ps.println(linePrefix+"boxCountInRange: "+boxCountInRange);
	}
		
	public static SysConfig defaultConf = new SysConfig();

	@Override
	public SysConfig clone() {
		try {
			return (SysConfig) super.clone();
		} catch (CloneNotSupportedException e) {
			throw new RuntimeException(e);
		}
	}
}

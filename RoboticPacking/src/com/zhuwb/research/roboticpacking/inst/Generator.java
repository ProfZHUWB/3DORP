package com.zhuwb.research.roboticpacking.inst;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import com.google.gson.Gson;
import com.zhuwb.research.roboticpacking.inst.InstConfig.Distribution;
import com.zhuwb.research.roboticpacking.inst.InstConfig.Regularity;

public class Generator {
	
	/**
	 * @param rand �����
	 * @param L			����L
	 * @param W			����W
	 * @param palletCount ��������
	 * @param boxTypeCount box��������
	 * @param boxTypeRegularity     ����Ĺ�����
	 * @param boxTypeDistribution pro[i] �ǵ� i ��box�ĸ���
	 * @param boxCountPerType box����
	 * @param vaccumType ������̹��: 2x3 �� 2x2
	 * @param theta      ������̹�����ű���
	 * @return instance ��������
	 */
	@SuppressWarnings("resource")
	public static InstData generate(Random rand, InstConfig conf) {
		int start = rand.nextInt(boxTypeSF.length-conf.boxTypeCount+1);//�����ѡ������������
		
		InstData inst = new InstData(conf.L, conf.W, conf.H, conf.boxTypeCount, conf.boxCount);		
		inst.name = conf.boxTypeRegularity+"-"+conf.boxTypeCount+"-"+conf.boxCount+"-"+conf.dist;

		
		int[][] boxtype = pickReg(rand,conf.boxTypeRegularity);//��ѡ����Ӧ����box����ά��Ϣ
		inst.boxType = Arrays.copyOfRange(boxtype, start, start+conf.boxTypeCount);
		for (int t=0; t<conf.boxTypeCount; t++) {
			inst.ortPerm[t] = new boolean[] {true,true,true,true,true,true};
		}
		
		// �������ֲ�����ÿ�ֺ��ӵĸ���
		int[] boxCountPerType = genDistribuiton(conf.boxCount, conf.dist, conf.boxTypeCount);
		// ��ÿ���������ɺ��ӵĵ���˳��
		ArrayList<Integer> boxType = new ArrayList<>();
		for (int t=0; t<conf.boxTypeCount; t++) {
			for (int i=0; i<boxCountPerType[t]; i++) {
				boxType.add(t);
			}
		}
		// ������ҵ���˳��
		Collections.shuffle(boxType, rand);
		
		for (int i=0; i<boxType.size(); i++) {
			inst.t[i] = boxType.get(i);
			inst.ort[i] = 0;
		}
		
//		bf.close();
		return inst;
	}
	
	
	/**
	 * �������һ��box������ǰ����ά���� [min, max] ֮�䣬������ӽ��������� V��
	 * ��3��ά�Ȳ����� maxD
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxType(Random rand, int min, int max, long V, int minD, int maxD) {
		int range = max - min + 1;
		while (true) {
			int L = rand.nextInt(range) + min;
			int W = rand.nextInt(range) + min;
			int H = (int)(V / (L * W));
			if (minD < H && H <= maxD) {
				return new int[] {L,W,H};
			}
		}
	}
	
	/**
	 * �ж� L �Ƿ��� dim �е���һά����ͬ
	 * @param L
	 * @param dim
	 * @return
	 */
	private static boolean equalAnySide(int L, int[] dim) {
		for (int i=0; i<dim.length; i++) {
			if (L == dim[i]) { return true; }
		}
		return false;
	}
	
	static final int MAX_ATTEMPT = 10000;

	/**
	 * �������һ��box������һ��ά���� boxType �е�һ����ͬ
	 * ��2��ά���� [min, max] ֮�䣬������ӽ��������� V��
	 * ��3��ά�Ȳ����� maxD
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxTypeSD1(Random rand, 
			int[] boxType, int min, int max, long V, int minD, int maxD) {
		int range = max - min + 1;
		for (int i=0; i<MAX_ATTEMPT; i++) {
			int L = boxType[rand.nextInt(3)]; // L �� boxType �������ѡ��һ���߳���
			int W = rand.nextInt(range) + min;
			if (equalAnySide(W, boxType)) { continue; }
			int H = (int)(V / (L * W));
			if (H < minD) { continue; }
			if (H > maxD) { continue; }
			if (equalAnySide(H, boxType)) { continue; }
			return new int[] {L,W,H};
		}
		return null;
	}

	/**
	 * �������һ��box������ǰ����ά���� boxType �е�����ά����ͬ
	 * ��3��ά�Ȳ����� maxD, ������ӽ��������� V����3��ά�ȿ����� boxType �е�ĳ��ά����ͬ��
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxTypeSD2(Random rand, 
			int[] boxType, long V, int minD, int maxD) {
//		long boxV = ((long) boxType[0]) * boxType[1] * boxType[2];
//		while (true) {
//			int[] r = boxType.clone();
//			int idx = rand.nextInt(3);
//			long area = boxV / boxType[idx];
//			r[idx] = (int) (V / area);
//			if (r[idx] > maxD) { continue; }
//			return r;
//		}
		
		for (int att=0; att<MAX_ATTEMPT; att++) {
			int idx = rand.nextInt(3);
			int i=0;
			while (i == idx) { i++; }
			int L = boxType[i++];
			while (i == idx) { i++; }
			int W = boxType[i];
			
			int H = (int)(V / (L * W));
			if (H < minD) { continue; }
			if (H > maxD) { continue; }
//			if (equalAnySide(H, boxType)) { continue; }
			return new int[] {L,W,H};
		}
		return null;
	}

	/**
	 * �������һ��box��ǰ2��ά���� [min, max] ֮�䣬������ӽ��������� V��
	 * ��3��ά�Ȳ����� maxD
	 * ��û���κ�ά����ǰһ��boxһ��
	 * @param min
	 * @param max
	 * @param V
	 * @param maxD
	 * @return
	 */
	private static int[] genBoxTypeSD0(Random rand, 
			int[] boxType, int min, int max, long V, int minD, int maxD) {
		int range = max - min + 1;
		for (int i=0; i<MAX_ATTEMPT; i++) {
			int L =  rand.nextInt(range) + min;
			if (equalAnySide(L, boxType)) { continue; }
			int W = rand.nextInt(range) + min;
			if (equalAnySide(W, boxType)) { continue; }
			int H = (int)(V / (L * W));
			if (H < minD) { continue; }
			if (H > maxD) { continue; }
			if (equalAnySide(H, boxType)) { continue; }
			return new int[] {L,W,H};
		}
		return null;
	}

	
	// SF �� 7 �� box �Ĵ�С
	public static final int[][] boxTypeSF = {{20,18,10},//1,1,1
			{25,20,18},//2,1,2
			{30,25,20},//4,2,2
			{36,30,25},//4,4,4
			{53,32,23},//6,2,3
			{70,40,32},//6,6,6
			{57,35,57}};//6,6,6
	
	/**
	 * ���� 7 �� box ����״
	 * @param rand  �����������
	 * @param reg	SF: ����SF��box���ࣻSD2: ��������box��2��ά����ͬ��SD1: ��������box��1��ά����ͬ��SD0����������box��0��ά����ͬ 
	 * @return 7 �� box, ÿ��box����ά��С�����ȵ�λcm
	 */
	public static int[][] pickReg(Random rand, Regularity reg){
		// һ�ֳ��õ� pallet �ĳߴ�Ϊ 1.2 m x 1.0 m������߶�Ϊ 1.5 m
		int[] pallet = {120,100,150};
		
		// ������box������°�������Ӵ�С����
		// �� i = 0,1,...,6 ��box����� v_i ԼΪһ�� pallet �� 1/n_i
		//    n_i �� [2^(i+3), 2^(i+3) + 2^(i+1)] ֮���һ������� 
		// ����������Ա�֤ n_i �� 8 - 640 ֮�䣬����box 8����������һ��pallet����С��640����������һ��pallet
		int palletV = pallet[0]*pallet[1]*pallet[2];
		int[] possibleV = new int[boxTypeSF.length];
		for(int i=0;i<possibleV.length;i++){
			int n = i+3;
			possibleV[possibleV.length-1-i] = palletV/(rand.nextInt((int)Math.pow(2, n-2)+1)+(int)Math.pow(2, n)); 
		}
		
//		if (debug) { System.out.println("-----------"); }

		//���� SF ÿ��box������ߵ���С�����ֵ
		int[] sfMin = new int[boxTypeSF.length];
		int[] sfMax = new int[boxTypeSF.length];
		for(int i=0;i<boxTypeSF.length;i++){
			int minD = Integer.MAX_VALUE;
			int maxD = Integer.MIN_VALUE;
			for(int j=0;j<boxTypeSF[i].length;j++){
				if(boxTypeSF[i][j]<minD){
					minD = boxTypeSF[i][j];
				}
				if(boxTypeSF[i][j]>maxD){
					maxD = boxTypeSF[i][j];
				}

			}
			sfMin[i] = minD;
			sfMax[i] = maxD;
		}
		
		//		System.out.println(Arrays.deepToString(sfMinMax));
		int minD = 10; // һ����׼���̵�ֱ��
		int limitD = 100;//�������box�ĳߴ�����
		switch (reg) {
		case SF:
			return boxTypeSF.clone();
		case SD1:
			int[][] sd1 = new int[boxTypeSF.length][3];
			outer:
			while (true) {
				sd1[0] = genBoxType(rand, sfMin[0], sfMax[0], possibleV[0], minD, limitD);
				for(int i = 1;i<sd1.length;i++){//һ��ά����ǰһ�ֵ����һ��ά����ȣ��ڶ���ά��ΪSFbox��������������ɣ�������ά�������Լ�����ұ�֤����box����ֻ��һ��ά�����
					sd1[i] = genBoxTypeSD1(rand,sd1[i-1],sfMin[i],sfMax[i],possibleV[i],minD, limitD);
					if (sd1[i] == null) { continue outer; }
				}
				break;
			}
			return sd1;
		case SD2:
			int[][] sd2 = new int[boxTypeSF.length][3];
			
//			if (debug) { System.out.println("-----1------"); }

			outer:
			while (true) {
				sd2[0] = genBoxType(rand, sfMin[0], sfMax[0], possibleV[0], minD, limitD);
				for(int i = 1;i<sd2.length;i++){
	//				if (debug) { System.out.println("----------- "+i); }
					sd2[i] = genBoxTypeSD2(rand,sd2[i-1],possibleV[i],minD, limitD);
					if(sd2[i] == null) { continue outer; }
				}
				break;
			}
			return sd2;
		case SD0:
			int[][] sd0 = new int[boxTypeSF.length][3];
			outer:
			while (true) {
				sd0[0] = genBoxType(rand, sfMin[0], sfMax[0], possibleV[0], minD, limitD);
				for(int i = 1;i<sd0.length;i++){
					sd0[i] = genBoxTypeSD0(rand,sd0[i-1],sfMin[i],sfMax[i],possibleV[i],minD, limitD);
					if (sd0[i] == null) { continue outer; }
				}
				break;
			}
			return sd0;
		default: throw new RuntimeException("Unsupported Regularity");
		}
	}

	
	/**
	 * �� boxCount �����������ķ��䵽 typeCount �ֺ���
	 * @param boxCount �ܺ�������
	 * @param result   [start, start+typeCount) ������������� result[i] ��i�ֺ��ӵĸ������������ķ������һ��
	 * @param start
	 * @param typeCount ����������
	 */
	public static void equalDistribute(int boxCount, int[] result, int start, int typeCount) {
		if (boxCount < 0 || typeCount <= 0) {
			throw new IllegalArgumentException("boxCount: "+boxCount+" typeCount: "+typeCount);
		}
		int countPerType = boxCount / typeCount;
		for (int i=0; i<typeCount-1; i++) {
			result[start+i] = countPerType;
			boxCount -= countPerType;
		}
		result[start+typeCount-1] = boxCount;
	}
	
	/**
	 * �Ѻ����������ֲ���ʽ���䵽���ֺ��ӡ�
	 * 
	 * ����box���ఴ�������С��������һ���� totalType �֣���0��ʼ��˳���ţ�0��1��2��...
	 * �� start ��ʼ���� typeNum �� box ���õ������࣬
	 * Ϊ�õ������ఴ��ָ���ķֲ�����ÿ��boxռ�ı���
	 * ���Ƕ��������С�� typeNum/3 ��ΪСbox��������� typeNum/3 ��Ϊ��box�������Ϊ��box
	 * ��ͬ��С���С���box������ͬ��
	 * 
	 * @param boxCount  �ܺ�����
	 * @param dist      �������ֲ�����
	 * @param typeCount ����������
	 * @return [t] ��t�ֺ��ӵĸ���������һ����ں�������
	 */
	public static int[] genDistribuiton(int boxCount, Distribution dist, int typeCount) {
//		System.out.println("totalType: "+totalType+"; start: "+start+"; typeNum: "+typeNum+"; dist: "+dist);
		if (typeCount <= 0 || boxCount <=0) { throw new RuntimeException(); }
		
		int lowCount = (int) (Math.round(0.2*boxCount));
		int highCount = boxCount - 2 * lowCount;

		
		switch(typeCount) {
		case 1: return new int[] {boxCount};
		case 2:
			switch(dist) {
			case small: return new int[] {highCount, boxCount-highCount};
			case uniform:
			case medium:
				int half = boxCount / 2;
				return new int[] {half, boxCount-half};
			case large: return new int[] {boxCount-highCount, highCount};
			default: throw new IllegalArgumentException("unsupported dist: "+dist);
			}
		default:
			int smallTypeCount = typeCount / 3;
			int largeTypeCount = typeCount / 3;
			int mediumTypeCount = typeCount - smallTypeCount - largeTypeCount;

			int[] boxCountPerType = new int[typeCount];
			switch(dist){
			case uniform: equalDistribute(boxCount, boxCountPerType, 0, typeCount); break;
			case small:
				equalDistribute(highCount, boxCountPerType, 0, smallTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount, mediumTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount+mediumTypeCount, largeTypeCount);
				break;
			case medium:
				equalDistribute(lowCount, boxCountPerType, 0, smallTypeCount);
				equalDistribute(highCount, boxCountPerType, smallTypeCount, mediumTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount+mediumTypeCount, largeTypeCount);
				break;
			case large:
				equalDistribute(lowCount, boxCountPerType, 0, smallTypeCount);
				equalDistribute(lowCount, boxCountPerType, smallTypeCount, mediumTypeCount);
				equalDistribute(highCount, boxCountPerType, smallTypeCount+mediumTypeCount, largeTypeCount);
				break;
			default:
				throw new IllegalArgumentException("Unsupported distribution: "+dist);
			}
			return boxCountPerType;
		}
		
	}

	public static void genDataSet(File dir, long seed) throws IOException {
		Random rand = new Random(seed);
		
		// Input feature:
		Regularity[] boxTypeRegularityList = Regularity.values();
		int[] boxTypeCountList = new int[]{2,3,4,5,6,7};
		int[] boxCountList = new int[]{200,500,1000,2000};
		int[][] palletLWHList = new int[][] {{120,100,150}};
		Distribution[] boxTypeDistributionList = Distribution.values();
		
		for(Regularity boxTypeRegularity:boxTypeRegularityList){
			for(int boxTypeCount:boxTypeCountList){
				for(int boxCount:boxCountList){
					for(Distribution dist:boxTypeDistributionList){
						InstConfig conf = new InstConfig();
						int[] lwh = palletLWHList[rand.nextInt(palletLWHList.length)];
						conf.instSeed = seed;
						conf.L = lwh[0];
						conf.W = lwh[1];
						conf.H = lwh[2];
						conf.boxTypeRegularity = boxTypeRegularity;
						conf.boxTypeCount = boxTypeCount;
						conf.boxCount = boxCount;
						conf.dist = dist;
						
						InstData inst = Generator.generate(rand,conf);
						File confDir = new File(dir,"conf");
						confDir.mkdirs();
						// ��config���json�ļ�
						Gson gs = new Gson();
						String json = gs.toJson(conf);
						BufferedWriter bw = new BufferedWriter(new FileWriter(new File(confDir, inst.name+".json")));
						bw.write(json);
						bw.close();
							
						File instDataFile = new File(dir, "instData/"+inst.name+".json");
						InstanceLoader.write(inst, instDataFile);
						System.out.println("write inst: "+inst.name);
					}//dist
				}//boxCount
			} //boxType
		} // regularity
	}
	
	public static void main(String[] args) throws IOException {
		genDataSet(new File(InstanceLoader.testsetPrefix+"2"), 2);
//		genDataSet(new File(InstanceLoader.testsetPrefix+"3"), 3);
//		genDataSet(new File(InstanceLoader.testsetPrefix+"4"), 4);
	}
	
//	public static boolean debug = false;
}


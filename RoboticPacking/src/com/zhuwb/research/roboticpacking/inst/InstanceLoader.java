package com.zhuwb.research.roboticpacking.inst;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import com.google.gson.Gson;

public class InstanceLoader {
	public static void write(InstData instData, File file) throws IOException {
		if (!file.getParentFile().exists()) {
			file.getParentFile().mkdirs();
		}
		Gson gs = new Gson();
		String json = gs.toJson(instData);
		BufferedWriter bw = new BufferedWriter(new FileWriter(file));
		bw.write(json);
		bw.close();
	}
	
//	public static Instance toInstance(InstData instData, Vacuum gripper) {
//		Instance inst = new Instance();
//		inst.name = instData.name;
//		inst.robot = gripper;
//		inst.pallet = new ArrayList<Pallet>();
//		for(int i=0;i<instData.getBoxCount();i++) {
//			inst.pallet.add(new Pallet(i,instData.L,instData.W,instData.H));
//		}
//		
//		double diameter = gripper.radius * 2;
//		
//		inst.box = new ArrayList<Box>();
//		double[] typeStat = new double[instData.getBoxTypeCount()];	// typeStat[j], 第 j 种box的数量
//		for(int i=0;i<instData.getBoxCount();i++) {
//			int type = instData.t[i] + 1;
//			Box b = new Box(i,type,instData.boxType);//生成box的序列，同时确定可以旋转的面
//			int tooSmallCount = 0;
//			if (b.l < diameter) {
//				tooSmallCount++;
//			}
//			if (b.w < diameter) {
//				tooSmallCount++;
//			}
//			if (b.h < diameter) {
//				tooSmallCount++;
//			}
//			if (tooSmallCount>1) {
//				throw new RuntimeException("Vaccum sucker diameter: "+diameter+" too big for box: "+b.l+" "+b.w+" "+b.h);
//			}
//			if (instData.ort[i] != 0) {
//				throw new RuntimeException("Only support initial rotation 0, but get: "+instData.ort[i]+" for box "+i);
//			}
//			// 每种 box 的6种旋转方式都可以
//			
////			System.out.println(instData.boxType.length);
////			System.out.println(instData.ortPerm.length);
////			System.out.flush();
////			
//			boolean[] ortPerm = instData.ortPerm[instData.t[i]];
//			for(int j = 0;j<6;j++) {
//				if (ortPerm[j]) {
//					b.rotation[j] = 1;
//				} else {
//					b.rotation[j] = 0;
//				}
//			}
//			Generator.caldm(b, inst.robot);
//			inst.box.add(b);
////			bf.write("\n");
//			typeStat[type-1]++;
//			//System.out.println(inst.box.get(i).v);
//		}
//		inst.pro = new double[instData.getBoxTypeCount()];
//		for(int a=0;a<inst.pro.length;a++){
//			inst.pro[a] = typeStat[a]/(double)instData.getBoxCount();
//		}
////		bf.close();
//		return inst;
//	}
	
	public static InstData load(File file) throws IOException {
		Gson gs = new Gson();
		BufferedReader stdin = new BufferedReader(new FileReader(file));
		InstData instData = gs.fromJson(stdin, InstData.class);
		stdin.close();
		return instData;
	}
	
//	public static Instance loadInstance(robot.Config conf) throws IOException {
//		File file = new File(testsetPrefix+conf.seed+"/instData/"+conf.getInstName()+".json");
//		Vacuum gripper = Vacuum.create(conf.theta, conf.vaccumType);
//		InstData instData = load(file);
//		return toInstance(instData,gripper);
//	}
	
	public static ArrayList<InstData> loadAll(File dir) throws IOException {
		ArrayList<InstData> instList = new ArrayList<>();
		//Vacuum gripper = Vacuum.create(theta, vacuumType);
		for (File f:dir.listFiles()) {
			if (!f.getName().endsWith(".json")) {
				continue;
			}
//			System.out.println("loading inst: "+f.getCanonicalPath());
			
			instList.add(load(f));
		}
		return instList;
	}
	
	public static String testsetPrefix = "data/ZhuFuZhou";
	public static ArrayList<InstData> load(long seed) throws IOException {
		return loadAll(new File(testsetPrefix+seed+"/instData"));
	}
	
	public static InstData load(long seed, String name) throws IOException {
		return load(new File(testsetPrefix+seed+"/instData/"+name+".json"));
	}
	
	public static InstConfig loadConf(File file) throws IOException {
		Gson gs = new Gson();
		BufferedReader stdin = new BufferedReader(new FileReader(file));
		InstConfig conf = gs.fromJson(stdin, InstConfig.class);
		stdin.close();
		return conf;
	}
	
	public static InstConfig loadConf(long seed, String name) throws IOException {
		return loadConf(new File(testsetPrefix+seed+"/conf/"+name+".json"));
	}
	
	public static void main(String[] args) throws IOException {
		ArrayList<InstData> a = load(2);
		for (InstData instData:a) {
			Gson gs = new Gson();
			String json = gs.toJson(instData);
			System.out.println(json);
		}
//		load(3);
//		load(4);
	}
}

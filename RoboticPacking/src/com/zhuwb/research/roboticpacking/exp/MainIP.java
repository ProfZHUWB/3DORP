package com.zhuwb.research.roboticpacking.exp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import com.zhuwb.research.roboticpacking.exp.Main.ExecutionRecord;
import com.zhuwb.research.roboticpacking.exp.Main.ExpConfig;
import com.zhuwb.research.roboticpacking.inst.InstConfig;
import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.inst.InstanceLoader;
import com.zhuwb.research.roboticpacking.inst.SysConfig;
import com.zhuwb.research.roboticpacking.inst.Vacuum;
import com.zhuwb.research.roboticpacking.search.AlgoConfig;


public class MainIP {

	public static void runAll(ArrayList<InstData> instDataList, File outdir) throws IOException {
		outdir.mkdirs();

		File summaryFile = new File(outdir, "summary-"+outdir.getName()+".csv");   	
		boolean exists = summaryFile.exists();	
		PrintWriter pw = new PrintWriter(new FileWriter(summaryFile, true));
		if (!exists) {
			pw.println(ExecutionRecord.getHeader()+","+ExpConfig.getHeader()); pw.flush();
		}

		for (int i=0; i<instDataList.size(); i++) {
			InstData instData = instDataList.get(i);
			ExpConfig conf = new ExpConfig(instData);
			
			conf.algoConf = new AlgoConfig();
			conf.sysConf = new SysConfig(); 
			conf.instConf = new InstConfig();
			
			// Algorithm
			conf.algoConf.seed = 2;
			conf.algoConf.depth = 1;
			conf.algoConf.effort = 16;
			conf.algoConf.hweight = 1;
			conf.algoConf.simulationCount = 0;                 // offline版本需要控制simulationCount=0  
			
			conf.sysConf.vaccumType = Vacuum.Type.TwoByThree;
			conf.sysConf.theta = 1.0;
			conf.sysConf.knownBoxCount = conf.inst.t.length;   // knownBoxCount 只对 online 起作用
			conf.sysConf.openPalletCount = 1;
			conf.sysConf.boxCountInRange = 1;
			
			InstData inst = instDataList.get(i);
			System.out.println("run "+i+":" + inst.name +" ****");
			File runDir = new File(outdir,"run-"+i+"-"+conf.inst.name);
			ExecutionRecord exeRec = Main.run(conf.inst, conf, runDir);
			if (exeRec != null) {
				pw.println(exeRec.toString()+","+conf.toString()); pw.flush();
			}
		}
		pw.close();
	}
	
	
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		File dir = new File("../RBNCode_Op/result/exp-IP-2021-07-07/instData");
		ArrayList<InstData> instDataList = InstanceLoader.loadAll(dir);
		Vacuum gripper = Vacuum.create(1.0, Vacuum.Type.TwoByThree);
		
		runAll(instDataList, new File(dir.getParentFile(),"OfflineHeuristic"));
		
	}

}

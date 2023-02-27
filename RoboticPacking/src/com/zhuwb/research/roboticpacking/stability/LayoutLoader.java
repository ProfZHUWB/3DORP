package com.zhuwb.research.roboticpacking.stability;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.LineNumberReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.zhuwb.research.roboticpacking.exp.Main.ExpConfig;
import com.zhuwb.research.roboticpacking.exp.Main4Online;
import com.zhuwb.research.roboticpacking.inst.InstData;
import com.zhuwb.research.roboticpacking.space.Space;
import com.zhuwb.research.roboticpacking.stability.PalletLayout.SME_Status;
import com.zhuwb.research.roboticpacking.stability.PalletLayout.Stats;

import ilog.concert.IloException;

public class LayoutLoader {
	public static ArrayList<PalletLayout> load(InstData inst, File loadingInstructionFile) throws IOException {
		double mu = 0.2;
		double gravityPerUnitVolume = 1.0;
		
		ArrayList<PalletLayout> pallets = new ArrayList<>();
		
		Pattern pattern = Pattern.compile("\\d+");
		
		try (LineNumberReader stdin = new LineNumberReader(new FileReader(loadingInstructionFile))) {
			String line = stdin.readLine().trim();
			if (!line.startsWith("box types:")) { throw new RuntimeException(); }
			for (int i=0; i<inst.getBoxTypeCount(); i++) {
				line = stdin.readLine();
				if (!line.startsWith("  type ")) { throw new RuntimeException(); }
			}
			line = stdin.readLine();
			if (!line.startsWith("arrival seq: [")) { throw new RuntimeException(); }
			line = line.substring("arrival seq: [".length(),line.length()-1);				
			String[] tokens = line.split(", ");
	
			int stepCount = 0;
			int loadedBoxCount = 0;
			while (true) {
				line = stdin.readLine();
				String stepLabel = "step "+stepCount+";";
				stepCount++;
				
				if (!line.startsWith(stepLabel)) { break; }
				line = stdin.readLine();
				if (!line.startsWith("  arrivalSeq:")) { throw new RuntimeException(); }
				line = stdin.readLine();
				if (!line.startsWith("  box belt:")) { throw new RuntimeException(); }
				line = stdin.readLine();
				if (!line.startsWith("  pallet belt:")) { throw new RuntimeException(); }
				line = stdin.readLine();
				if (line.startsWith("  close pallet")) { continue; }
				if (!line.startsWith("  place ")) { throw new RuntimeException(); }
	
	//			System.out.println(line);
	
				Matcher m = pattern.matcher(line);
				if (!m.find()) { throw new RuntimeException(); } // box idx in arrival sequence
				if (!m.find()) { throw new RuntimeException(); } // box type
	//			int boxType = Integer.parseInt(m.group());
	//			System.out.println("box type: "+boxType);
				
				if (!m.find()) { throw new RuntimeException(); } // box location in operation range
				if (!m.find()) { throw new RuntimeException(); } // pallet idx
				int palletIdx = Integer.parseInt(m.group());
	//			System.out.println("pallet idx: "+palletIdx);
				
				if (!m.find()) { throw new RuntimeException(); } // pallet location in loading area
				if (!m.find()) { throw new RuntimeException(); } // occupied: l1
				int l1 = Integer.parseInt(m.group());
				if (!m.find()) { throw new RuntimeException(); }
				int w1 = Integer.parseInt(m.group());
				if (!m.find()) { throw new RuntimeException(); }
				int h1 = Integer.parseInt(m.group());
				if (!m.find()) { throw new RuntimeException(); }
				int l2 = Integer.parseInt(m.group());
				if (!m.find()) { throw new RuntimeException(); }
				int w2 = Integer.parseInt(m.group());
				if (!m.find()) { throw new RuntimeException(); }
				int h2 = Integer.parseInt(m.group());
				Space boxSpace = new Space(l1, w1, h1, l2, w2, h2);
	//			System.out.println("occupied: "+boxSpace);
	
				while (pallets.size() <= palletIdx) {
					pallets.add(new PalletLayout(inst.L, inst.W, inst.H, mu, gravityPerUnitVolume));
				}
				PalletLayout pallet = pallets.get(palletIdx);
				pallet.placeBox(loadedBoxCount, boxSpace);
				
				loadedBoxCount++;
			}
	
			if (!line.startsWith("final box belt:")) { throw new RuntimeException(); }
			line = stdin.readLine();
			if (!line.startsWith("final pallet belt:")) { throw new RuntimeException(); }
	
			if (loadedBoxCount != tokens.length) { throw new RuntimeException("loaded box count: "+loadedBoxCount+" != arrival sequence length: "+tokens.length); }		
			int boxCount = 0;
			for (PalletLayout pallet:pallets) {
				boxCount += pallet.getBoxCount();
			}
			if (loadedBoxCount != boxCount) { throw new RuntimeException("boxes in pallets: "+boxCount); }
		}
		
		return pallets;
	}

	
	public static void main(String[] args) throws IOException, IloException {
		// If check_SME = true, invoke cplex to check stability of layouts
		// Configure cplex (assume cplex is installed in C:\Program Files\IBM\ILOG\CPLEX_Studio201\cplex\bin\x64_win64)
		//   1. copy C:\Program Files\IBM\ILOG\CPLEX_Studio201\cplex\lib\cplex.jar" to replace lib\cplex.jar
		//   2. Run as dialogue add the following VM arguments:
		//      Djava.library.path="C:\Program Files\IBM\ILOG\CPLEX_Studio201\cplex\bin\x64_win64" 
		boolean check_SME = true;
		long instSeed = 2;
		int algoSeed = 2;
		ArrayList<ExpConfig> confList = Main4Online.genExpConfigOnline(instSeed, true, -1, algoSeed);		
		File outdir = new File("result/2023-01-18-final-"+instSeed); 
		
		File boxFile = new File(outdir, (check_SME?"SME-":"")+"stablility-box.csv");
		File palletFile = new File(outdir, (check_SME?"SME-":"")+"stablility-pallet.csv");
		File summaryFile = new File(outdir, (check_SME?"SME-":"")+"stablility-summary.csv");
		
		PrintWriter boxPW = new PrintWriter(new FileWriter(boxFile));
		boxPW.println("inst,pallet,box,centroid_pos(1:inside;0:outside;-1:boundary),dist_to_boundary,max{L W}/2,relative_dist"
				+(check_SME? ",SME_stable,#box,#contact_region,#var,#constraints,model_built_time(ms),total_time(ms)":""));
		PrintWriter palletPW = new PrintWriter(new FileWriter(palletFile));
		palletPW.println("inst,pallet,#box,#unstable box,min dist,min relative dist,avg relative dist"
				+(check_SME? ",#SME unstable layout,SME time(ms)":""));
		PrintWriter summaryPW = new PrintWriter(new FileWriter(summaryFile));
		summaryPW.println("inst,#pallet,#box,#unstable pallet,#unstable box,min dist,min relative dist,avg relative dist"
				+(check_SME?",#SME unstable pallet,#SME unstable box,SME time(ms)":""));
		
		
		ArrayList<ExpConfig> expList = new ArrayList<>();
		for (ExpConfig confTemplate: confList) {
			if (confTemplate.instConf.boxCount > 1000) { continue; }
			expList.add(confTemplate);
		}


		int unstableSol = 0;
		double overallMinDist = Double.MAX_VALUE;
		double overallMinRelative = Double.MAX_VALUE;
		int SME_unstableSol = 0;
		for (int i=0; i<expList.size(); i++) {
			ExpConfig conf = expList.get(i);
			File runDir = new File(outdir,"run-"+i+"-"+conf.inst.name);
			File loadingInstructionFile = new File(runDir, "loading-operation.txt");
			if (!loadingInstructionFile.exists()) { continue; }
			
			System.out.println("checking "+loadingInstructionFile);
			ArrayList<PalletLayout> pallets = load(conf.inst, loadingInstructionFile);
			System.out.println("  pallets: "+pallets.size());

			int totalBoxCount = 0;
			int unstablePallet = 0;
			int unstableBox = 0;
			double minDist = Double.MAX_VALUE;
			double minRelativeDist = Double.MAX_VALUE;
			double sumRelativeDist = 0;
			double stableBoxCount = 0;
			int SME_unstablePalletCount = 0;
			int SME_unstableBox = 0;
			long inst_time_SME = 0;
			for (int j=0; j<pallets.size(); j++) {
				PalletLayout p = pallets.get(j);
				Stats stats = p.checkConvexHullStability("    ");
				int unstableCount = p.getBoxCount() - stats.stableBoxCount;
				double avg = 0;
				if (stats.stableBoxCount > 0) {
					avg = stats.sumRelativeDist / stats.stableBoxCount;
				}
				
				long start = System.currentTimeMillis();
				int SME_unstableLayoutCount = 0;
				ArrayList<SME_Status> status_SME = null;
				if (check_SME) {
//					File palletFile = new File(runDir, "P"+j);
					status_SME = p.rigidBodySME_all(null);
					for (SME_Status s:status_SME) {
						if (!s.stable) {
							SME_unstableLayoutCount += 1;
						}
					}
				}
				long time_SME = System.currentTimeMillis() - start;
				inst_time_SME += time_SME;
				
				palletPW.println(conf.inst.name+","+j+","+p.getBoxCount()+","+unstableCount+","+stats.minDistToBoundary+","+stats.minRelativeDist+","+avg
						+(check_SME?","+SME_unstableLayoutCount+","+time_SME : ""));
				palletPW.flush();
				
				if (!stats.palletStable) { unstableCount += 1; }
				if (stats.minDistToBoundary < minDist) { minDist = stats.minDistToBoundary; }
				if (stats.minRelativeDist < minRelativeDist) { minRelativeDist = stats.minRelativeDist; }
				sumRelativeDist += stats.sumRelativeDist;
				stableBoxCount += stats.stableBoxCount;
				
				if (SME_unstableLayoutCount > 0) {
					SME_unstablePalletCount += 1;
				}
				SME_unstableBox += SME_unstableLayoutCount;

				for (int k=0; k<p.getBoxCount(); k++) {
					SME_Status sme = check_SME? status_SME.get(k):null;
					boxPW.println(conf.inst.name+","+j+","+k+","+stats.centroidPos[k]+","+stats.dist[k]+","+stats.maxDim[k]+","+(stats.dist[k]/stats.maxDim[k])
							+(check_SME? ","+sme.stable+","+sme.boxCount+","+sme.contactRegionCount+","+sme.countVariable()+","+sme.countConstraints()+","+sme.modelBuildTimeMS+","+sme.totalTimeMS : ""));
				}
				totalBoxCount += p.getBoxCount();
			}

			double avg = 0;
			if (stableBoxCount > 0) {
				avg = sumRelativeDist / stableBoxCount;
			}
			System.out.println("  #unstable pallets: "+unstablePallet+", min dist: "+minDist+", min relative dist: "+minRelativeDist+", avg relative dist: "+avg
					+(check_SME?", #SME unstable pallents: "+SME_unstablePalletCount : ""));
			summaryPW.println(conf.inst.name+","+pallets.size()+","+totalBoxCount+","+unstablePallet+","+unstableBox+","+minDist+","+minRelativeDist+","+avg
					+(check_SME? ","+SME_unstablePalletCount+","+SME_unstableBox+","+inst_time_SME : ""));
			summaryPW.flush();
			
			if (unstablePallet > 0) { unstableSol += 1; }
			if (minDist < overallMinDist) { overallMinDist = minDist; }
			if (minRelativeDist < overallMinRelative) { overallMinRelative = minRelativeDist; }			
			if (SME_unstablePalletCount > 0) {
				SME_unstableSol += 1;
			}
		}
		boxPW.close();
		palletPW.close();
		summaryPW.close();
		
		System.out.println("#unstable solution: "+unstableSol+", min dist: "+overallMinDist+", min relative dist: "+overallMinRelative
				+(check_SME?", #SME unstable solution: "+SME_unstableSol : ""));
	}
}

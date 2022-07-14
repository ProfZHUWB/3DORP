package com.zhuwb.research.roboticpacking.inst;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import com.zhuwb.research.roboticpacking.inst.Vacuum.Align;
import com.zhuwb.research.roboticpacking.inst.Vacuum.PushAxis;
import com.zhuwb.research.roboticpacking.space.Space;

public class Mathematica {
	public static enum Color {
		Orange, Green, Red, Black, Yellow, Purple
	}
	
	// 按照一种pallet的大小确定其它的大小
//	private String[] vars = new String[] {"L","W","H"};
	private Color palletColor = Color.Orange;
	
	private Color boxColor = Color.Green;
	
	private Color suckerColor = Color.Red;
	private Color boardColor = Color.Black;
	
	private Color boxHorizontalPathColor = Color.Yellow;
	private Color boxVirtialPathColor = Color.Purple;
	private Color gripperPathColor = Color.Purple;

	
//	public String toCuboid(double[] start, double[] end) {
//		return "Cuboid[{"+start[0]+vars[0]+","+start[1]+vars[1]+","+start[2]+vars[2]+"},{"
//				+end[0]+vars[0]+","+end[1]+vars[1]+","+end[2]+vars[2]+"}]";
//	}
//	public String toCuboid(Space s, int L, int W, int H) {
//		return toCuboid(new double[] {((double) s.l1)/L, ((double)s.w1)/W, ((double)s.h1)/H},
//				new double[] {((double) s.l2)/L, ((double)s.w2)/W, ((double)s.h2)/H});				
//	}
	public String toCuboid(double[] start, double[] end) {
		return "Cuboid[{"+start[0]+","+start[1]+","+start[2]+"},{"
				+end[0]+","+end[1]+","+end[2]+"}]";
	}
	public String toCuboid(Space s) {
		return toCuboid(new double[] {s.l1, s.w1, s.h1},
				new double[] {s.l2, s.w2, s.h2});				
	}
	
	// 画一种木制 pallet
	public String drawPallet() {
		// 木条尺寸按照以下的标准 pallet 设计
		final double L = 120.0;
		final double W = 100.0;
//		final double H = 150.0;
		
		// 5 L- strip on top
//		int stripCount = 10; double stripGap = 2.0/W;
//		double stripThick = 3.0/H;
		int stripCount = 10; double stripGap = 2.0;
		double stripThick = 3.0;
//		double stripWidth = (1.0-stripGap * (stripCount-1)) / stripCount;
		double stripWidth = (W-stripGap * (stripCount-1)) / stripCount;
		double stripWithGap = stripWidth + stripGap;
		StringBuilder sb = new StringBuilder();
		sb.append(palletColor);
//		double[] start = new double[] {0.0, 0,    -stripThick};
//		double[] end = new double[]   {1.0, 0, 0.0};
		double[] start = new double[] {0.0, 0,    -stripThick};
		double[] end = new double[]   {L, 0, 0.0};
		for (int j=0; j<stripCount; j++) {
			start[1] = j*stripWithGap;
			end[1] = start[1]+stripWidth;
			sb.append(",");
			sb.append(toCuboid(start,end));
		}
//		double boardThick = 2.0/H;
		double boardThick = 2.0;
		sb.append(",");
//		sb.append(toCuboid(new double[] {0.0,0.0,-(stripThick+boardThick)}, new double[] {1.0,1.0,-stripThick}));
		sb.append(toCuboid(new double[] {0.0,0.0,-(stripThick+boardThick)}, new double[] {L,W,-stripThick}));

		// 3 x 3 pillar
		int pillarCount = 3; 
//		double pillarL = 15.0/L;
//		double pillarW = 10.0/W;
//		double pillarH = 15.0/H;
		double pillarL = 15.0;
		double pillarW = 10.0;
		double pillarH = 15.0;
//		double pillarLGap = (1.0 - pillarL*pillarCount) / (pillarCount-1);
//		double pillarWGap = (1.0 - pillarW*pillarCount) / (pillarCount-1);
		double pillarLGap = (L - pillarL*pillarCount) / (pillarCount-1);
		double pillarWGap = (W - pillarW*pillarCount) / (pillarCount-1);
		start = new double[] {0,   0.0, -(stripThick+boardThick+pillarH)};
		end = new double[] {0, 0.0, -(stripThick+boardThick)};
		double pillarLwithGap = pillarL + pillarLGap;
		double pillarWwithGap = pillarW + pillarWGap;
		for (int i=0; i<pillarCount; i++) { // along L-
			start[0] = i * pillarLwithGap;
			end[0] = start[0] + pillarL;
			for (int j=0; j<pillarCount; j++) { // along W-
				start[1] = j * pillarWwithGap;
				end[1] = start[1]+pillarW;
				sb.append(",");
				sb.append(toCuboid(start, end));
			}
		}
		
//		double baseStripThick = 3.0/H;
		double baseStripThick = 3.0;
//		start = new double[] {0.0, 0.0, -(stripThick+boardThick+pillarH+baseStripThick)};
//		end = new double[]   {0.0, 1.0, -(stripThick+boardThick+pillarH)};
		start = new double[] {0.0, 0.0, -(stripThick+boardThick+pillarH+baseStripThick)};
		end = new double[]   {0.0, W, -(stripThick+boardThick+pillarH)};
		for (int i=0; i<pillarCount; i++) {
			start[0] = i * pillarLwithGap;
			end[0] = start[0] + pillarL;
			sb.append(",");
			sb.append(toCuboid(start, end));
		}
//		start = new double[] {0.0, 0.0, -(stripThick+boardThick+pillarH+baseStripThick)};
//		end = new double[]   {1.0, 0.0, -(stripThick+boardThick+pillarH)};
		start = new double[] {0.0, 0.0, -(stripThick+boardThick+pillarH+baseStripThick)};
		end = new double[]   {L, 0.0, -(stripThick+boardThick+pillarH)};
		for (int j=0; j<pillarCount; j++) { // along W-
			start[1] = j * pillarWwithGap;
			end[1] = start[1]+pillarW;
			sb.append(",");
			sb.append(toCuboid(start, end));
		}
		
		return sb.toString();
	}
	
	public String drawGripper(int[] loc, Vacuum gripper, PushAxis pushAxis, Align align) {
		StringBuilder sb = new StringBuilder();

		double[] gripperLen = gripper.getGripperLength()[pushAxis.ordinal()][align.ordinal()];
		int[] suckerCount = gripper.getSuckerCount()[pushAxis.ordinal()][align.ordinal()];
		double[] firstSuckerCenter = gripper.getFirstSuckerCenterRelativePosition()[pushAxis.ordinal()][align.ordinal()];
		double[] suckerDist = gripper.getSuckerCenterDistance()[pushAxis.ordinal()][align.ordinal()];
		double[] suckerThick = gripper.getSuckerThicness()[pushAxis.ordinal()][align.ordinal()];
		
//		double[] start = new double[] {(loc[0]+suckerThick[0])/L,(loc[1]+suckerThick[1])/W,(loc[2]+suckerThick[2])/H};
//		double[] end = new double[] {start[0]+gripperLen[0]/L,start[1]+gripperLen[1]/W,start[2]+gripperLen[2]/H};
		double[] start = new double[] {(loc[0]+suckerThick[0]),(loc[1]+suckerThick[1]),(loc[2]+suckerThick[2])};
		double[] end = new double[] {start[0]+gripperLen[0],start[1]+gripperLen[1],start[2]+gripperLen[2]};
		
		sb.append(boardColor);
		sb.append(",Opacity[0.3],");
		sb.append(toCuboid(start, end));
		
		sb.append(",Opacity[1],");
		sb.append(suckerColor);
		double[] suckerCenter = new double[3];
		double[] suckerCenterEnd = new double[3];
		for (int i=0; i<suckerCount[0]; i++) { // L- 方向的数量
//			suckerCenter[0] = (loc[0] + firstSuckerCenter[0] + suckerDist[0] * i)/L;
//			suckerCenterEnd[0] = suckerCenter[0] + suckerThick[0]/L;
			suckerCenter[0] = loc[0] + firstSuckerCenter[0] + suckerDist[0] * i;
			suckerCenterEnd[0] = suckerCenter[0] + suckerThick[0];
			for (int j=0; j<suckerCount[1]; j++) { // W -
//				suckerCenter[1] = (loc[1] + firstSuckerCenter[1] + suckerDist[1] * j)/W;
//				suckerCenterEnd[1] = suckerCenter[1] + suckerThick[1]/W;
				suckerCenter[1] = loc[1] + firstSuckerCenter[1] + suckerDist[1] * j;
				suckerCenterEnd[1] = suckerCenter[1] + suckerThick[1];
				for (int k=0; k<suckerCount[2]; k++) {
//					suckerCenter[2] = (loc[2] + firstSuckerCenter[2] + suckerDist[2] * k)/H;
//					suckerCenterEnd[2] = suckerCenter[2] + suckerThick[2]/H;
					suckerCenter[2] = loc[2] + firstSuckerCenter[2] + suckerDist[2] * k;
					suckerCenterEnd[2] = suckerCenter[2] + suckerThick[2];
					
					sb.append(",");
//					sb.append("Cylinder[{{"+suckerCenter[0]+vars[0]+","+suckerCenter[1]+vars[1]+","+suckerCenter[2]+vars[2]+"},{"
//							+suckerCenterEnd[0]+vars[0]+","+suckerCenterEnd[1]+vars[1]+","+suckerCenterEnd[2]+vars[2]+"}},"+gripper.r/W+vars[1]+"]");					
					sb.append("Cylinder[{{"+suckerCenter[0]+","+suckerCenter[1]+","+suckerCenter[2]+"},{"
							+suckerCenterEnd[0]+","+suckerCenterEnd[1]+","+suckerCenterEnd[2]+"}},"+gripper.r+"]");					
				}
			}
		}
		return sb.toString();
	}
	
	
	public String drawPlacements(Placement[] placements, Vacuum gripper, Space pallet, boolean drawPush) {
		StringBuilder sb = new StringBuilder();
		sb.append("Opacity[0],");
//		sb.append(toCuboid(new double[]{0, 0, 0}, new double[]{1,1,1}));
//		sb.append(toCuboid(new double[]{0, 0, 0}, new double[]{L,W,H}));
		sb.append(toCuboid(pallet));
		sb.append(","+boxColor);
		sb.append(",Opacity[opac],Take[{");
		for (int i=0; i<placements.length; i++) {
			if (i>0) { sb.append(","); }
			Placement p = placements[i];
			sb.append(toCuboid(p.occupied));
		}
		sb.append("},numboxes]");
		
		if (drawPush) {
			sb.append(","+boxHorizontalPathColor+",Opacity[0.5],{");
			for (int i=0; i<placements.length; i++) {
				 sb.append(",");  // 在最开始插入一个空的元素
				 Placement p = placements[i];
				 Space c = p.boxPath;
				 sb.append(toCuboid(c));
			}
			sb.append("}[[numboxes+1]]");
		
			sb.append(","+boxVirtialPathColor+",Opacity[1],{");
			for (int i=0; i<placements.length; i++) {
				 sb.append(",");  // 在最开始插入一个空的元素
				 Placement p = placements[i];
				 Space c = p.dropPath;
				 sb.append(toCuboid(c));
			}
			sb.append("}[[numboxes+1]]");
		
			sb.append(","+gripperPathColor+",Opacity[0.2],{");
			for (int i=0; i<placements.length; i++) {
				 sb.append(",");  // 在最开始插入一个空的元素
				 Placement p = placements[i];
				 Space c = p.gripperPath;
				 sb.append(toCuboid(c));
			}
			sb.append("}[[numboxes+1]]");

			sb.append(",Opacity[1],{");
			for (int i=0; i<placements.length; i++) {
				 sb.append(",");  // 在最开始插入一个空的元素
				 Placement p = placements[i];
				 int[] loc = gripper.computeGripperLocation(p.gripperPath, p.pushAxis, pallet);
				 String draw = drawGripper(loc, gripper, p.pushAxis, p.align);
				 sb.append("{");
				 sb.append(draw);
				 sb.append("}");
			}
			sb.append("}[[numboxes+1]]");
		}
		
		return sb.toString();
	}
	
	// 如果 space == null, 画每一个盒子及推送方式
	// 如果 space != null, 只画每个盒子,不画推送方式,画一个透明的space
	public String draw(Placement[] placements, Vacuum gripper, Space pallet, Space space) {
		String str = drawPallet();
		String spaceString = "";
		if (space != null) {
			spaceString = ",Red,Opacity[0.1],"+toCuboid(space);
		}
		String placementStr = drawPlacements(placements, gripper, pallet, space==null);
		
		return "Manipulate[\r\n"+
				"Graphics3D[{{"+str+spaceString+"},\r\n "
				+"{"+placementStr+"}},\r\n"
				+"Boxed -> False, SphericalRegion -> True, "
				+ "ImageSize -> {400, 400}, ViewPoint -> {1.2, -1.6, 1.2}, " // {.8, 1.6, 2.}
				+ "ViewVertical -> {0, 0, 1}, Axes -> True, "
				+ "AxesLabel -> {\"L\", \"W\", \"H\"}],\r\n"
				
//				+ "{{L, "+L+", \"L\"}, 1, "+L+", 1, Appearance -> \"Labeled\", ImageSize -> Tiny},\r\n"
//				+ " {{W, "+W+", \"W\"}, 1, "+W+", 1, Appearance -> \"Labeled\", ImageSize -> Tiny},\r\n"
//				+ " {{H, "+H+", \"H\"}, 1, "+H+", 1, Appearance -> \"Labeled\", ImageSize -> Tiny},\r\n"
				+ " Delimiter, \"opacity\",\r\n"
				+ " {{opac, .9, \"\"}, 0, 1, .1, Appearance -> \"Labeled\", ImageSize -> Tiny},\r\n"
				+ " Delimiter, \"number of boxes\",\r\n"
				+ " {{numboxes, "+placements.length+", \"\"}, 0, "+placements.length+", 1, Appearance -> \"Labeled\", ImageSize -> Tiny},\r\n"
				+ " ControlPlacement -> Left]";
	}
	
	public void drawToFile(File file, Placement[] placements, Vacuum gripper, Space pallet, Space space) throws IOException {
		file.getParentFile().mkdirs();
		String str = draw(placements, gripper, pallet, space);
		PrintWriter pw = new PrintWriter(new FileWriter(file));
		pw.println(str);
		pw.close();
	}

	public static final Mathematica inst = new Mathematica();

	
	public static void example_LWH_Push() throws IOException {
		Vacuum gripper = Vacuum.create(1.0, Vacuum.Type.TwoByThree);
		BoxType[] boxTypes = new BoxType[] {
				new BoxType(0, 60, 50, 40, new boolean[] {true, true, true,true,true,true},
						gripper, 0.5),
				new BoxType(1, 30, 20, 25, new boolean[] {true, true, true,true,true,true},
						gripper, 0.5)
		};
		int L=120, W=100,H=150;
		Space pallet = new Space(0,0,0,L,W,H);
		
		Placement[] placements = new Placement[2];
		Placement p = new Placement();
		placements[0] = p;
		p.boxType = 0;
		p.ort = boxTypes[p.boxType].distinctOrt[0];
		p.releaseL = 0;
		p.releaseW = 0;
		p.releaseH = 20;
		p.dh = 20;
		p.pushAxis = PushAxis.L;
		p.align = Align.org;				
		p.computePathsForDebug(gripper, pallet);
		
		p = new Placement();
		placements[1] = p;
		p.boxType = 0;
		p.ort = boxTypes[p.boxType].distinctOrt[1];
		p.releaseL = 0;
		p.releaseW = 0;
		p.releaseH = 60;
		p.dh = 20;
		p.pushAxis = PushAxis.L;
		p.align = Align.org;		
		p.computePathsForDebug(gripper, pallet);
		Mathematica.inst.drawToFile(
				new File("result/L-Push.nb"), placements, gripper, pallet, null);
		
		p.pushAxis = PushAxis.W;
		p.align = Align.rotated;		
		p.computePathsForDebug(gripper, pallet);
		Mathematica.inst.drawToFile(
				new File("result/W-Push.nb"), placements, gripper, pallet, null);
		
		p.pushAxis = PushAxis.H;
		p.align = Align.org;		
		p.computePathsForDebug(gripper, pallet);
		Mathematica.inst.drawToFile(
				new File("result/H-Push.nb"), placements, gripper, pallet, null);
	}
	
	public static void main(String[] args) throws IOException {
		example_LWH_Push();
	}
}

package com.zhuwb.research.roboticpacking.stability;

import java.util.ArrayList;
import java.util.Arrays;

import com.zhuwb.research.roboticpacking.space.Space;
import com.zhuwb.research.roboticpacking.stability.Geometry2D.Point2d;
import com.zhuwb.research.roboticpacking.stability.Geometry2D.Polygon2d;
import com.zhuwb.research.roboticpacking.stability.Geometry3D.Point3d;
import com.zhuwb.research.roboticpacking.stability.Geometry3D.Vector3d;

import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloNumVarType;
import ilog.cplex.IloCplex;

public class PalletLayout {
	boolean open = true;
	Space loadingSpace;
	ArrayList<SpaceWithContactRegion> boxNodes = new ArrayList<>(); // 0 is a dummy box representing floor
	ArrayList<ContactRegion> contactRegions = new ArrayList<>();
	int lastLoadingSequence = -1;
	
	public final double mu;
	public final double gravityPerUnitVolume;
	
	public static enum Axis {
		L,W,H;
	}

	public static class ContactRegion {
		// smaller.x2 == lager.x1
		SpaceWithContactRegion smaller;  // the box that is smaller along X-axis
		SpaceWithContactRegion larger;  // the box that is larger along X-axis
		Space region;
		Vector3d unitNorm;  // unit norm vector of the contact region (pointing from smaller to larger)
		
		int[][] r;       // r[v] the displacement vector from Origin to vertex v
		IloNumVar[][] F; // F[v] the force vector (variable) at vertex v applied to larger
		double[][] f;    // f[v] this force vector at vertex v applied to larger
		
		public ContactRegion(SpaceWithContactRegion smaller, SpaceWithContactRegion larger) {
			this.smaller = smaller;
			this.larger = larger;
			region = smaller.space.intersect(larger.space);

			r = new int[4][];
			if (smaller.space.l2 == larger.space.l1) { // contact axis is L
				r[0] = new int[] {region.l1, region.w1, region.h1};
				r[1] = new int[] {region.l1, region.w1, region.h2};
				r[2] = new int[] {region.l1, region.w2, region.h1};
				r[3] = new int[] {region.l1, region.w2, region.h2};
				unitNorm = Vector3d.i;
			} else if (smaller.space.w2 == larger.space.w1) { // contact axis is W
				r[0] = new int[] {region.l1, region.w1, region.h1};
				r[1] = new int[] {region.l1, region.w1, region.h2};
				r[2] = new int[] {region.l2, region.w1, region.h1};
				r[3] = new int[] {region.l2, region.w1, region.h2};
				unitNorm = Vector3d.j;
			} else {
				assert smaller.space.h2 == larger.space.h1; // contact axis is H
				r[0] = new int[] {region.l1, region.w1, region.h1};
				r[1] = new int[] {region.l1, region.w2, region.h1};
				r[2] = new int[] {region.l2, region.w1, region.h1};
				r[3] = new int[] {region.l2, region.w2, region.h1};				
				unitNorm = Vector3d.k;
			}
			
		}
		
		public Axis getContactAxis() {
			if (smaller.space.l2 == larger.space.l1) { // contact axis is L
				return Axis.L;
			}
			if (smaller.space.w2 == larger.space.w1) { // contact axis is W
				return Axis.W;
			}
			assert smaller.space.h2 == larger.space.h1; // contact axis is H
			return Axis.H;
		}

		/**
		 * @param v		0 to 3
		 * @param s		either this.smaller or this.larger
		 * @return the force applied at vertex v to the box s
		 */
		public Vector3d getForceTo(int v, SpaceWithContactRegion s) {
			if (s == smaller) {
				return new Vector3d(-f[v][0], -f[v][1], -f[v][2]);
			}
			if (s == larger) {
				return new Vector3d(f[v][0], f[v][1], f[v][2]);
			}
			throw new IllegalArgumentException(s+" is neither the smaller box "+this.smaller+" nor the larger box "+larger);
		}

		/**
		 * @param s either this.smaller or this.larger
		 * @return a unit vector norm to this contact region and pointing towards s.
		 */
		public Vector3d getNorm(SpaceWithContactRegion s) {
			if (s == this.smaller) {
				return unitNorm.negate();
			}
			if (s == this.larger) {
				return unitNorm;
			}
			throw new IllegalArgumentException(s+" is neither the smaller box "+this.smaller+" nor the larger box "+larger);
		}

		/**
		 * @param v
		 * @return vertex v as a Point3d object
		 */
		public Vector3d getVertex(int v) {
			return new Point3d(r[v][0], r[v][1], r[v][2]);
		}
		
		public String toString() {
			return String.format("ContactRegion(%d,%d): ", smaller.loadingSequence+1, larger.loadingSequence+1)+region;
		}
	}
	
	public static class SpaceWithContactRegion {
		public final int loadingSequence;
		public final Space space;
		
		ArrayList<ContactRegion> L1 = new ArrayList<>();
		ArrayList<ContactRegion> L2 = new ArrayList<>();
		ArrayList<ContactRegion> W1 = new ArrayList<>();
		ArrayList<ContactRegion> W2 = new ArrayList<>();
		ArrayList<ContactRegion> H1 = new ArrayList<>();
		ArrayList<ContactRegion> H2 = new ArrayList<>();
		
		public SpaceWithContactRegion(int loadingSequence, Space space) {
			this.loadingSequence = loadingSequence;
			this.space = space;
		}
		
		public ContactRegion updateContact(SpaceWithContactRegion other) {
			if (this.space.l1 == other.space.l2) {
				if (!space.projectionIntersectL(other.space)) { return null; }
				ContactRegion cr = new ContactRegion(other, this);
				this.L1.add(cr);
				other.L2.add(cr);
				return cr;
			}
			if (this.space.l2 == other.space.l1) {
				if (!space.projectionIntersectL(other.space)) { return null; }
				ContactRegion cr = new ContactRegion(this, other);
				this.L2.add(cr);
				other.L1.add(cr);
				return cr;
			}
			if (this.space.w1 == other.space.w2) {
				if (!space.projectionIntersectW(other.space)) { return null; }
				ContactRegion cr = new ContactRegion(other, this);
				this.W1.add(cr);
				other.W2.add(cr);
				return cr;
			}
			if (this.space.w2 == other.space.w1) {
				if (!space.projectionIntersectW(other.space)) { return null; }
				ContactRegion cr = new ContactRegion(this, other);
				this.W2.add(cr);
				other.W1.add(cr);
				return cr;
			}
			if (this.space.h1 == other.space.h2) {
				if (!space.projectionIntersectH(other.space)) { return null; }
				ContactRegion cr = new ContactRegion(other, this);
				this.H1.add(cr);
				other.H2.add(cr);
				return cr;
			}
			if (this.space.h2 == other.space.h1) {
				if (!space.projectionIntersectL(other.space)) { return null; }
				ContactRegion cr = new ContactRegion(this, other);
				this.H2.add(cr);
				other.H1.add(cr);
				return cr;
			}
			return null;
		}
		
		public ArrayList<ContactRegion> getAllContactRegions() {
			ArrayList<ContactRegion> all = new ArrayList<>();
			all.addAll(L1);
			all.addAll(L2);
			all.addAll(W1);
			all.addAll(W2);
			all.addAll(H1);
			all.addAll(H2);
			return all;
		}
		
		public Point3d getCenter() {
			return new Point3d(0.5*space.l1+0.5*space.l2, 0.5*space.w1+0.5*space.w2, 0.5*space.h1+0.5*space.h2);
		}
		
		public Vector3d getGravity(double gravityPerUnitVolume) {
			return new Vector3d(0,0,-space.volume*gravityPerUnitVolume);
		}
		
		Vector3d netFaceTorque;
		/**
		 * Print contact force, torque for every vertex of contact regions for a face.
		 * @param linePrefix
		 * @param faceLabel
		 * @param face
		 * @param pivot
		 * @return Total force on the face, set this.netFaceTorque to total torque of all contact forces.
		 */
		public Vector3d validateAndPrintSME(String linePrefix, String faceLabel, ArrayList<ContactRegion> face, Point3d pivot, double mu) {
			Vector3d netFaceForce = Vector3d.zero;
			netFaceTorque = Vector3d.zero;
			
			System.out.println(linePrefix+faceLabel+" face norm: "+face.get(0).unitNorm);
			
			for (ContactRegion cr:face) {
				System.out.println(linePrefix+"  Contact region: "+cr.region);
				Vector3d unitNorm = cr.getNorm(this);
				for (int v=0; v<4; v++) {
					Vector3d F = cr.getForceTo(v, this);
					Vector3d r = cr.getVertex(v).minus(pivot);
					Vector3d torque = r.cross(F);
					double F_n_norm = F.dot(unitNorm);
					Vector3d F_n = unitNorm.scale(F_n_norm); // F_n = <F,n>n
					Vector3d f = F.minus(F_n);				 // F - F_n
					double f_norm = f.norm2();
					double ratio = 0;
					if (F_n_norm != 0) {
						ratio = f_norm / F_n_norm;
					}
					System.out.println(linePrefix+"    v"+v+", r = "+r+", F = "+F+", r x F = "+torque+", <F,n> = "+F_n_norm+", ||f|| = "+f_norm+", ||f||/<F,n> = "+ratio);
					if (ratio > mu) {
						throw new RuntimeException("||f||/<F,n> exceed mu = "+mu);
					}

					netFaceForce = netFaceForce.add(F);
					netFaceTorque = netFaceTorque.add(torque);
				}
			}
			System.out.println(linePrefix+"  face total contact force = "+netFaceForce+", torque = "+netFaceTorque);
			return netFaceForce;
		}
			
		public void validateAndPrintSME(String linePrefix, Point3d pivot, double mu, double gravityPerUnitVolume) {
			Vector3d r = getCenter().minus(pivot);
			Vector3d G = getGravity(gravityPerUnitVolume);
			System.out.println(linePrefix+"box "+(loadingSequence+1)+": "+space+", r = "+r+", G = "+G+", r x G = "+r.cross(G));
			String[] faceLabels = new String[] {"Left (L1)", "Right (L2)", "Back (W1)", "Front (W2)", "Bottom (H1)", "Top (H2)"};
			ArrayList[] faces = new ArrayList[] {L1, L2, W1, W2, H1, H2 };

			Vector3d totalContactForce = Vector3d.zero;
			Vector3d totalContactTorque = Vector3d.zero;
			for (int i = 0; i<6; i++) {
				ArrayList<ContactRegion> face = faces[i];
				if (face.size() > 0) {
					Vector3d netFaceForce = validateAndPrintSME(linePrefix+"  ", faceLabels[i], face, pivot, mu);
					totalContactForce = totalContactForce.add(netFaceForce);
					totalContactTorque = totalContactTorque.add(this.netFaceTorque);
				}
			}
			System.out.println(linePrefix+"  total contact force = "+totalContactForce+", torque = "+totalContactTorque);
			Vector3d netF = totalContactForce.add(G);
			Vector3d netT = totalContactTorque.add(r.cross(G));
			if (netF.norm2() > 0.01) {
				throw new RuntimeException("net force: "+netF+", magnitude: "+netF.norm2()+" not zero");
			}
			if (netT.norm2() > 0.01) {
				throw new RuntimeException("net torque: "+netT+", magnitude: "+netT.norm2()+" not zoer");
			}
		}
	}
	
	public PalletLayout(int L, int W, int H, double mu, double gravityPerUnitVolume) {
		this.loadingSpace = new Space(0, 0, 0, L, W, H);
		SpaceWithContactRegion floor = new SpaceWithContactRegion(lastLoadingSequence, new Space(0,0,-10,L,W,0)); // a box with top face at L = 0
		this.boxNodes.add(floor);
		this.mu = mu;
		this.gravityPerUnitVolume = gravityPerUnitVolume;
	}
	
	public int getBoxCount() {
		return this.boxNodes.size() - 1;
	}
	
	public SpaceWithContactRegion placeBox(int loadingSquence, Space box) {
		if (loadingSquence <= lastLoadingSequence) { throw new RuntimeException("box loadingSequence: "+loadingSquence+" not greater than last box: "+lastLoadingSequence); }
		SpaceWithContactRegion boxNode = new SpaceWithContactRegion(loadingSquence, box);
		
		for (SpaceWithContactRegion existing:boxNodes) {
			ContactRegion cr = boxNode.updateContact(existing);
			if (cr != null) { this.contactRegions.add(cr); }
		}	
		boxNodes.add(boxNode);
		lastLoadingSequence = loadingSquence;
		return boxNode;
	}
	
	public PalletLayout copyFirst(int n) {
		assert n >= 0 && n <= getBoxCount() : "number of boxes: "+n+" not in valid range [0, "+getBoxCount()+"]";
		PalletLayout newP = new PalletLayout(this.loadingSpace.l2, this.loadingSpace.w2, this.loadingSpace.h2, this.mu, this.gravityPerUnitVolume);
		for (int i=1; i<=n; i++) {
			SpaceWithContactRegion node = this.boxNodes.get(i);
			newP.placeBox(node.loadingSequence, node.space);
		}
		return newP;
	}
	
	
	public static class Stats {
		public int[] centroidPos;  // centroidPos[i] = 1 (inside) 0 (outside) -1 (boundary)
		public double[] dist;      // dist[i] = the minimum distance from centeroid of box i to boundary of convex hull of supporting contact regions
		public double[] maxDim;    // maxDim[i] = max(l2-l1, w2-w1)/2 for each box
		public Stats(int boxCount) {
			this.centroidPos = new int[boxCount];
			this.dist = new double[boxCount];
			this.maxDim = new double[boxCount];
		}
		
		public boolean palletStable = true;  // if boxStatus[i] == 1 for each box i
		public int stableBoxCount = 0;       // number of boxes with boxStatus[i] == 1
		public double minDistToBoundary = Double.MAX_VALUE; // min dist[i]
		public double minRelativeDist = Double.MAX_VALUE;   // min dist[i]/maxDim[i]
		public double sumRelativeDist = 0;                  // sum dist[i]/maxDim[i]
	}
	
	/**
	 * Check whether centroid of box i is inside the convex hull of contatct regions in its bottom face.
	 * @param linePrefix   prefix for printing every line of error message
	 * @return check results
	 */
	public Stats checkConvexHullStability(String linePrefix) {
		int boxCount = getBoxCount();
		Stats stats = new Stats(boxCount);
		
		if (boxCount < 1) { return stats; }
		
		PalletLayout newP = new PalletLayout(this.loadingSpace.l2, this.loadingSpace.w2, this.loadingSpace.h2, this.mu, this.gravityPerUnitVolume);
		for (int i=1; i<=boxCount; i++) {
			SpaceWithContactRegion node = this.boxNodes.get(i);
			SpaceWithContactRegion newNode = newP.placeBox(node.loadingSequence, node.space);
			
			double x = 0.5 * (newNode.space.l1 + newNode.space.l2);
			double y = 0.5 * (newNode.space.w1 + newNode.space.w2);
			double maxDim = Math.max(newNode.space.l2-newNode.space.l1, newNode.space.w2-newNode.space.w1) * 0.5;
			stats.maxDim[(i-1)] = maxDim;

			ArrayList<Point2d> points = new ArrayList<>();
			for (ContactRegion cr: newNode.H1) {
				int l1 = cr.region.l1;
				int l2 = cr.region.l2;
				int w1 = cr.region.w1;
				int w2 = cr.region.w2;
				points.add(new Point2d(l1,w1));
				points.add(new Point2d(l1,w2));
				points.add(new Point2d(l2,w1));
				points.add(new Point2d(l2,w2));
			}
			Polygon2d hull = Polygon2d.convexHull(points);
			int status = hull.inInterior(x, y);
			stats.centroidPos[(i-1)] = status;

			if (status < 1) {
				stats.palletStable = false;
				System.out.println(linePrefix+"box "+(i-1)+": "+node.space);
				
				hull.print(linePrefix+"  ");
				System.out.println(linePrefix+"hull.inInterior("+x+","+y+"): "+status);
			} else {
				stats.stableBoxCount += 1;
				double d = hull.closetOnBoundary(new Point2d(x,y));
				stats.dist[(i-1)] = d;
				if (d < stats.minDistToBoundary) {
					stats.minDistToBoundary = d;
				}
				double rd = d / maxDim;
				if (rd < stats.minRelativeDist) {
					stats.minRelativeDist = rd;
				}
				stats.sumRelativeDist += rd;
			}
		}
		
		return stats;
	}
	
	
	public static class SME_Status {
		boolean stable;         // whether model has feasible solution
		int boxCount;			// number of constraints: 6 * boxCount + 4 * contactRegion
		int contactRegionCount; // * 12 ==> number of decision variables
		long modelBuildTimeMS;  // time taken to build QCQP
		long totalTimeMS;       // time taken to build and solve QCQP
		
		public int countVariable() {
			return contactRegionCount * 4 * 3;
		}
		
		public int countConstraints() {
			return boxCount * 3 // net force 
			     + boxCount * 3 // net torque
			     + contactRegionCount * 4  // friction force
			     + contactRegionCount * 4; // f_norm . norm >= 0
		}
	}
	
	/**
	 * Compute the stability of current layout.
	 * 		net force for every box is zero
	 *      net torque (with respect to origin) for every box is zero
	 *      magnitude_of_friction <= this.mu * magnitude_of_norm_force for every contact force F 
	 *      
	 * @param fileName
	 * @return checking result 
	 * @throws IloException
	 */
	public SME_Status rigidBodySME(String fileName) throws IloException {
		SME_Status status = new SME_Status();
		status.boxCount = this.getBoxCount();
		status.contactRegionCount = this.contactRegions.size();
		long startTime = System.currentTimeMillis();
		IloCplex cplex = new IloCplex();// create model and solve it
		//cplex.setParam(IloCplex.Param.Emphasis.MIP, 1);//0:balanced;1:feasibility;2:optimality
		//(x-z)i: represents the left-bottom-behind position of the ith box in one pallet
		
		// Create force F_ijv at vertex v of contact region <i,j>, that is applied to box j 
		// Compressive force only: F_ijvx >= 0, if i and j contact along x-axis.
		for (ContactRegion cr:this.contactRegions) {
			cr.F = new IloNumVar[4][3];
			Axis contactingAxis = cr.getContactAxis();
			String labelPrefix = "F"+(cr.smaller.loadingSequence+1)+"_"+(cr.larger.loadingSequence+1)+"_";
			
//			System.out.println("Region: "+labelPrefix+": "+cr.region);
			
			for (int v=0; v<4; v++) {
				for (int a=0; a<3; a++) {
					String label = labelPrefix+v+Axis.values()[a];
					float lowerBound;
					if (a == contactingAxis.ordinal()) {
						lowerBound = 0;
					} else {
						lowerBound = Float.NEGATIVE_INFINITY;
					}
					// cplex.numVar(0.0,Float.MAX_VALUE,IloNumVarType.Float,"x_"+Axis.values()[a]);					
					cr.F[v][a] = cplex.numVar(lowerBound, Float.POSITIVE_INFINITY, IloNumVarType.Float, label);
					
//					System.out.println(cr.F[v][a].getName()+" lb: "+cr.F[v][a].getLB()+" ub: "+cr.F[v][a].getUB());
				}
			}
		}
		
		// Friction force constraints:
		//    sum_{a != x}  (F[v][a] * F[v][a]) <= mu * mu * F[v][x] * F[v][x], for all v at vertex of <i,j>
		// When mu = 0 =>
		//    F[v][a] = 0, for a != x
		if (mu > 0) {
			for (ContactRegion cr:this.contactRegions) {
				String prefix = "Friction"+(cr.smaller.loadingSequence+1)+"_"+(cr.larger.loadingSequence+1)+"_";
				Axis contactingAxis = cr.getContactAxis();
				for (int v=0; v<4; v++) {
					IloNumExpr friction = cplex.numExpr();
					for (int a=0; a<3; a++) {
						if (a != contactingAxis.ordinal()) {
							IloNumExpr term = cplex.prod(1.0, cr.F[v][a], cr.F[v][a]);
							friction = cplex.sum(friction, term);
						}
					}
					IloNumVar fnorm = cr.F[v][contactingAxis.ordinal()];
					cplex.addLe(friction, cplex.prod(mu*mu, fnorm, fnorm),prefix+v);		
				}
			}
		} else {
			for (ContactRegion cr:this.contactRegions) {
				Axis contactingAxis = cr.getContactAxis();
				for (int v=0; v<4; v++) {
					for (int a=0; a<3; a++) {
						if (a != contactingAxis.ordinal()) {
							cr.F[v][a].setLB(0);
							cr.F[v][a].setUB(0);
						}
					}
				}
			}
		}
		
		// Zero net force for each box i
		for (int i=1; i<this.boxNodes.size(); i++) {
			SpaceWithContactRegion boxNode = boxNodes.get(i);
			// Gravity experienced by box i
			double[] G = new double[] { 0, 0, - boxNode.space.volume * gravityPerUnitVolume};
			
//			System.out.println("box "+i+", G: "+Arrays.toString(G));
//			System.out.println("    ----> space: "+boxNode.space);
			
			ArrayList<ContactRegion> all = boxNode.getAllContactRegions();
			for (int a=0; a<3; a++) {
				String label = "ZNF"+(boxNode.loadingSequence+1)+Axis.values()[a];
				IloNumExpr expr = cplex.numExpr();
				for (ContactRegion cr:all) {
					for (int v=0; v<4; v++) {
						if (boxNode == cr.larger) {
							expr = cplex.sum(expr, cr.F[v][a]);
						} else {
							assert boxNode == cr.smaller;
							expr = cplex.sum(expr, cplex.prod(-1, cr.F[v][a]));
						}
					}
				}
				cplex.addEq(expr, -G[a], label);
			}
		}
		
		// zero net torque (with respective pivot O) for each box i
		for (int i=1; i<this.boxNodes.size(); i++) {
			SpaceWithContactRegion boxNode = boxNodes.get(i);
//			double[] r = new double[] {
//					(0.5 * boxNode.space.l1 + 0.5 * boxNode.space.l2),
//					(0.5 * boxNode.space.w1 + 0.5 * boxNode.space.w2),
//					(0.5 * boxNode.space.h1 + 0.5 * boxNode.space.h2)
//			};
//			System.out.println("r"+i+": "+Arrays.toString(r));
			
			double G_i2 = (- boxNode.space.volume * gravityPerUnitVolume);
			// Torque due to gravity (with respect to O): (r_i1 * G_i2, -r_i0*G_i2, 0)
			double[] T = new double[] {
				(0.5 * boxNode.space.w1 + 0.5 * boxNode.space.w2) * G_i2,
				-(0.5 * boxNode.space.l1 + 0.5 * boxNode.space.l2) * G_i2,
				0
			};
			
			ArrayList<ContactRegion> all = boxNode.getAllContactRegions();
			IloNumExpr expr = cplex.numExpr();
			for (ContactRegion cr:all) {
				for (int v=0; v<4; v++) {
					if (boxNode == cr.larger) {
						expr = cplex.sum(expr, cplex.prod( cr.r[v][1], cr.F[v][2]));
						expr = cplex.sum(expr, cplex.prod(-cr.r[v][2], cr.F[v][1]));
					} else {
						expr = cplex.sum(expr, cplex.prod(-cr.r[v][1], cr.F[v][2]));
						expr = cplex.sum(expr, cplex.prod( cr.r[v][2], cr.F[v][1]));
					}
				}
			}
			cplex.addEq(expr, -T[0], "ZNT"+(boxNode.loadingSequence+1)+"L");
			
			expr = cplex.numExpr();
			for (ContactRegion cr:all) {
				for (int v=0; v<4; v++) {
					if (boxNode == cr.larger) {
						expr = cplex.sum(expr, cplex.prod( cr.r[v][2], cr.F[v][0]));
						expr = cplex.sum(expr, cplex.prod(-cr.r[v][0], cr.F[v][2]));
					} else {
						expr = cplex.sum(expr, cplex.prod(-cr.r[v][2], cr.F[v][0]));
						expr = cplex.sum(expr, cplex.prod( cr.r[v][0], cr.F[v][2]));
					}
				}
			}
			cplex.addEq(expr, -T[1], "ZNT"+(boxNode.loadingSequence+1)+"W");

			expr = cplex.numExpr();
			for (ContactRegion cr:all) {
				for (int v=0; v<4; v++) {
					if (boxNode == cr.larger) {
						expr = cplex.sum(expr, cplex.prod( cr.r[v][0], cr.F[v][1]));
						expr = cplex.sum(expr, cplex.prod(-cr.r[v][1], cr.F[v][0]));
					} else {
						expr = cplex.sum(expr, cplex.prod(-cr.r[v][0], cr.F[v][1]));
						expr = cplex.sum(expr, cplex.prod( cr.r[v][1], cr.F[v][0]));
					}
				}
			}
			cplex.addEq(expr, -T[2], "ZNT"+(boxNode.loadingSequence+1)+"H");			
		}
		
//		IloLinearNumExpr obj = cplex.linearNumExpr();
//		obj.addTerm(1, contactRegions.get(0).F[0][2]);
//		cplex.minimize(obj);
		
		if (fileName != null) {
			System.out.println("export model to: "+fileName);
			cplex.exportModel(fileName);
		}
		status.modelBuildTimeMS = System.currentTimeMillis() - startTime;
		
		status.stable = cplex.solve();
		if (status.stable) {
			for (ContactRegion cr:this.contactRegions) {
				cr.f = new double[4][3];
				for (int v=0; v<4; v++) {
					for (int a=0; a<3; a++) {
						cr.f[v][a] = cplex.getValue(cr.F[v][a]);
//						System.out.println(cr.F[v][a].getName()+" = "+cr.f[v][a]);
					}
				}
			}
		}
		
		cplex.close();
		status.totalTimeMS = System.currentTimeMillis() - startTime;
		return status;
	}
	
	
	/**
	 * Starting from empty pallet, load one box in each step according to loading sequence.
	 * After a box is loaded, check stability of all boxes in the layout.
	 * @param fileNamePrefix
	 * @return the status for each box i
	 * @throws IloException
	 */
	public ArrayList<SME_Status> rigidBodySME_all(String fileNamePrefix) throws IloException {
		ArrayList<SME_Status> status = new ArrayList<>();
		for (int i=1; i<=getBoxCount(); i++) {
//			System.out.println("B"+i+": "+boxNodes.get(i).space);
			
			PalletLayout p = this.copyFirst(i);
			String fileName = fileNamePrefix == null? null : fileNamePrefix+"B"+i+"-model.lp"; 
			status.add(p.rigidBodySME(fileName));
		}
		return status;
	}
	
	public void validateAndPrintSME(String linePrefix, Point3d pivot) {
		for (int i=1; i<=getBoxCount(); i++) {
			SpaceWithContactRegion s = boxNodes.get(i);
			s.validateAndPrintSME(linePrefix, pivot, mu, gravityPerUnitVolume);
		}
	}
	
	public static void test1() throws IloException {
		// https://www.mathcha.io/editor/Q0J0WSM8fqNUEq4zYQty7OjW2slPxENMikO55Pq
		double mu = 0.2;
		double gravityPerUnitVolume = 1.0; 
		PalletLayout p = new PalletLayout(120, 100, 150, mu, gravityPerUnitVolume);
		p.placeBox(0, new Space(0,0,0,10,10,20));     // V1 = 2000 = 10 x 10 x 20
		p.placeBox(1, new Space(21,0,0,31,10,30));    // V2 = 3000 = 10 x 10 x 30
		p.placeBox(2, new Space(0,0,20,21,10,25));    // V3 = 1050 = 21 x 10 x 5
		p.rigidBodySME("result/SME-3box-3.lp");
		
		p.validateAndPrintSME("", Point3d.O);
		
		
//		System.out.println("--------------- manual solution ----------------");
//		
//		for (ContactRegion cr:p.contactRegions) {
//			System.out.println(cr);
//		}
//		ContactRegion cr0_1 = p.contactRegions.get(0);
//		ContactRegion cr0_2 = p.contactRegions.get(1);
//		ContactRegion cr1_3 = p.contactRegions.get(2);
//		ContactRegion cr3_2 = p.contactRegions.get(3);
//		cr0_1.f = new double[][] {
//			new double[] {105*0.5,0,1210*0.5}, // F_P/2, F_P = 105i + 1210k
//			new double[] {105*0.5,0,1210*0.5}, // F_P/2
//			new double[] {0,0,1840*0.5},       // F_Q/2, F_Q = 1840k
//			new double[] {0,0,1840*0.5}        // F_Q/2
//		};
//		cr1_3.f = new double[][] {
//			new double[] {0,0,0}, 
//			new double[] {0,0,0},
//			new double[] {105*0.5,0,1050*0.5}, // F_A/2, F_A = 105i + 1050k
//			new double[] {105*0.5,0,1050*0.5}  // F_A/2
//		};
//		cr0_2.f = new double[][] {
//			new double[] {0,0,1237.5/2},       // F_U/2, F_U = 1237.5k
//			new double[] {0,0,1237.5/2},
//			new double[] {-105*0.5,0,1762.5/2},// F_V/2, F_V = -105i + 1762.5k
//			new double[] {-105*0.5,0,1762.5/2}
//		};
//		cr3_2.f = new double[][] {
//			new double[] {0,0,0}, 
//			new double[] {105*0.5,0,0},        // -F_B/2, F_B = -105i
//			new double[] {0,0,0}, 
//			new double[] {105*0.5,0,0},        // -F_B/2, F_B = -105i
//		};
//				
//		p.validateAndPrintSME("", Point3d.O);
	}
	
	public static void main(String[] args) throws IloException {
		// https://www.mathcha.io/editor/Q0J0WSM8fqNUEq4zYQty7OjW2slPxENMikO55Pq
		double mu = 0.2;
		double gravityPerUnitVolume = 1.0; 
		PalletLayout p = new PalletLayout(120, 100, 150, mu, gravityPerUnitVolume);
		p.placeBox(0, new Space(0,0,0,10,10,20));     // V1 = 2000 = 10 x 10 x 20
		p.placeBox(1, new Space(21,0,0,31,10,30));    // V2 = 3000 = 10 x 10 x 30
		p.placeBox(2, new Space(0,0,20,21,10,25));    // V3 = 1050 = 21 x 10 x 5

		ArrayList<SME_Status> status = p.rigidBodySME_all("result/SME-3box");
	}
}
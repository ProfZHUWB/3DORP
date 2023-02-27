package com.zhuwb.research.roboticpacking.stability;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

public class Geometry2D extends JFrame {
	public static class Vector2d implements Comparable<Vector2d> {
		public final double x;
		public final double y;
		public static final Vector2d i = new Vector2d(1,0);
		public static final Vector2d j = new Vector2d(0,1);
		public static final Vector2d zero = new Vector2d(0,0);

		public Vector2d(double x, double y) {
			this.x = x;
			this.y = y;
		}
		
		public int hashCode() {
			return Objects.hash(x, y);
		}

		public boolean equals(Object obj) {
			if (!(obj instanceof Vector2d)) {
				return false;
			} else {
				Vector2d other = (Vector2d) obj;
				return x == other.x && y == other.y;
			}
		}

		public int compareTo(Vector2d other) {
			if (x != other.x)
				return Double.compare(x, other.x);
			else
				return Double.compare(y, other.y);
		}
		
		public String toString() {
			return String.format("(%g, %g)", x, y);
		}
		
		/**
		 * The cross product of this (x1, y1) and other vector (x2, y2),
		 *    (x1 i + y1 j) x (x2 i + y2 j) = (x1y2 - y1x2) k
		 * Cross product is perpendicular to the plane XY. 
		 *    
		 * @param other
		 * @return (x1y2 - y1x2) the length of cross product.
		 */
		public double cross(Vector2d other) {
			return this.x * other.y - this.y * other.x;
		}
		
		/**
		 * The dot product of this and other vector
		 * @param other
		 * @return this dot other
		 */
		public double dot(Vector2d other) {
			return this.x*other.x + this.y*other.y;
		}

		/**
		 * @return Euclidean norm of of vector: || this ||_2
		 */
		public double norm2() {
			return Math.sqrt(x*x + y*y);
		}
		
		/**
		 * @return Square of Euclidean norm of this vector: (|| this ||_2)^2
		 */
		public double norm2Squared() {
			return x*x + y*y;
		}

		/**
		 * @return the unit vector: this / ||this||.
		 */
		public Vector2d unit() {
			double n = norm2();
			if (n == 0) { throw new RuntimeException("zero vector has no unit vector"); }
			return new Vector2d(x/n, y/n);
		}

		/**
		 * Vector addition.
		 * @param other
		 * @return this + other
		 */
		public Vector2d add(Vector2d other) {
			return new Vector2d(this.x+other.x, this.y+other.y);
		}

		/**
		 * Vector subtraction.
		 * @param other
		 * @return this - other
		 */
		public Vector2d minus(Vector2d other) {
			return new Vector2d(this.x-other.x, this.y-other.y);
		}

		/**
		 * Scalar product.
		 * @param c
		 * @return c * this.
		 */
		public Vector2d scale(double c) {
			return new Vector2d(c*this.x, c*this.y);
		}

		/**
		 * @return Rotate this vector anti-clockwise about the origin by 90 degree (Pi/4).
		 */
		public Vector2d rotate90Degree() {
			return new Vector2d(-this.y, this.x);
		}
		
		/**
		 * @return negate: -this.
		 */
		public Vector2d negate() {
			return new Vector2d(-x, -y);
		}
	}

	/**
	 * (x,y) the coordinate of a point P,
	 * which is also the displacement vector between the origin O and P.
	 * @author iwenc
	 */
	public static class Point2d extends Vector2d {		
		public static final Point2d O = new Point2d(0,0);
		
		public Point2d(double x, double y) {
			super(x,y);
		}
		
		public Point2d(Vector2d displacementFromOrigin) {
			super(displacementFromOrigin.x, displacementFromOrigin.y);
		}
		
		public String toString() {
			return "Point"+super.toString();
		}
		
		public boolean equals(Object obj) {
			if (!(obj instanceof Point2d))
				return false;
			else {
				Point2d other = (Point2d)obj;
				return x == other.x && y == other.y;
			}
		}			
	}
	
	
	public static class Line2d {
		public final Point2d p;  // A point on the line
		public final Vector2d n; // unit norm
		
		public Line2d(Point2d p1, Point2d p2) {
			this(p1, norm(p1, p2));
		}
		
		/**
		 * Construct a line passing through point p with norm vector n
		 * @param p
		 * @param n
		 */
		public Line2d(Point2d p, Vector2d n) {
			this.p = p;
			this.n = n.unit();
		}
		
		/**
		 * @param p1
		 * @param p2
		 * @return the vector norm to the line passing through two distinct point p1 and p2.
		 * @throws IllegalArgumentException if p1 and p2 are the same point
		 */
		public static Vector2d norm(Point2d p1, Point2d p2) {
			if (p1.equals(p2)) { throw new IllegalArgumentException("p1: "+p1+" and p2: "+p2+" are same point, cannot define a unique line"); }
			Vector2d r = p2.minus(p1); // r = p2 - p1
			return r.rotate90Degree();
		}
		
		/**
		 * @param a
		 * @return Find the foot of the perpendicular of a point a on the this line.
		 */
		public Point2d foot(Point2d a) {
			// Let b be foot, ba perpendicular to bp, ba = (pa . n) n
			// where pa . n is the distance from this line to a.
			Vector2d ba = this.n.scale(distance(a)); // ba = (pa . n) n			
			return new Point2d(a.minus(ba));         // ob = oa + ab = oa - ba
		}
		
		/**
		 * The distance from this line to a given point a.
		 * Distance is positive of a is away from this line in the direction of the norm vector.
		 * @param a
		 * @return  pa . n = (this.p - a) . n
		 */
		public double distance(Point2d a) {
			Vector2d pa = a.minus(this.p);   // p - this.p
			return pa.dot(this.n);
		}
		
		public String toString() {
			return "Line("+this.p+", norm"+this.n+")";
		}
	}
	
	public static class LineSegment2d extends Line2d {
		public final Point2d e;
		public LineSegment2d(Point2d p, Point2d e) {
			super(p,e);
			this.e = e;
		}
		
		public Point2d closestPoint;
		/**
		 * @param a
		 * @return a point in this line segment that is closest to a given point a.
		 */
		public double closest(Point2d a) {
			Point2d f = super.foot(a);
			Vector2d pf = f.minus(this.p);
			Vector2d fe = e.minus(f);
			if (pf.dot(fe) >= 0) { // f in the line segment [p, e]
				closestPoint = f;
				double df = distance(a);     // || f - a ||_2
				if (df < 0) { df = -df; }
				return df;
			}

			// f outside line segment
			double ds = this.p.minus(a).norm2(); // || p - a ||_2
			double de = this.e.minus(a).norm2(); // || e - a ||_2
			if (ds <= de) {
				closestPoint = this.p;
				return ds;
			} else {
				closestPoint = this.e;
				return de;
			}
		}
	}
	
	
	
	public static class Polygon2d {
		public ArrayList<Point2d> vertices;
		
		public Polygon2d(ArrayList<Point2d> vertices) {
			this.vertices = vertices;
		}

		/**
		 * Return a new polygon where order of vertices are reversed
		 * @return v[i] = this.v[last-i]
		 */
		public Polygon2d reverseVertexOrder() {
			ArrayList<Point2d> reversed = new ArrayList<>(vertices.size());
			for (int i=vertices.size()-1; i>=0; i--) {
				reversed.add(vertices.get(i));
			}
			return new Polygon2d(reversed);
		}
	
		/**
		 * Compute convex hull of a given set of points using Andrew's monotone chain algorithm with time complexity O(n log n).
		 * @param points
		 * @return The vertices of convex hull in clockwise order (math convention, larger y is on top of smaller y);
		 * or in counter-clockwise order (computer graphics convention, larger y is below smaller y);
		 */
		public static Polygon2d convexHull(List<Point2d> points) {
			List<Point2d> newPoints = new ArrayList<>(points);
			Collections.sort(newPoints);
			return new Polygon2d(convexHullPresorted(newPoints));
		}
			
		/**
		 * Compute convex hull for sorted points using Andrew's monotone chain algorithm with time complexity O(n log n).
		 *   (points[i].x, points[i].y) <= (points[i+1].x, points[i+1].y) 
		 *   
		 * Adapted from: https://www.nayuki.io/res/convex-hull-algorithm/ConvexHull.java
		 * 
		 * @param points
		 * @return The vertices of convex hull in clockwise order (math convention, larger y is on top of smaller y);
		 * or in counter-clockwise order (computer graphics convention, larger y is below smaller y);
		 */
		public static ArrayList<Point2d> convexHullPresorted(List<Point2d> points) {
			if (points.size() <= 1)
				return new ArrayList<>(points);
			
			// Andrew's monotone chain algorithm. Positive y coordinates correspond to "up"
			// as per the mathematical convention, instead of "down" as per the computer
			// graphics convention. This doesn't affect the correctness of the result.
			
			ArrayList<Point2d> upperHull = new ArrayList<>();
			for (Point2d p : points) {
				while (upperHull.size() >= 2) {
					Point2d q = upperHull.get(upperHull.size() - 1);
					Point2d r = upperHull.get(upperHull.size() - 2);
					if (((long)(q.x - r.x)) * (p.y - r.y) >= ((long)(q.y - r.y)) * (p.x - r.x))
						upperHull.remove(upperHull.size() - 1);
					else
						break;
				}
				upperHull.add(p);
			}
			upperHull.remove(upperHull.size() - 1);
			
			ArrayList<Point2d> lowerHull = new ArrayList<>();
			for (int i = points.size() - 1; i >= 0; i--) {
				Point2d p = points.get(i);
				while (lowerHull.size() >= 2) {
					Point2d q = lowerHull.get(lowerHull.size() - 1);
					Point2d r = lowerHull.get(lowerHull.size() - 2);
					if (((long)(q.x - r.x)) * (p.y - r.y) >= ((long)(q.y - r.y)) * (p.x - r.x))
						lowerHull.remove(lowerHull.size() - 1);
					else
						break;
				}
				lowerHull.add(p);
			}
			lowerHull.remove(lowerHull.size() - 1);
			
			if (!(upperHull.size() == 1 && upperHull.equals(lowerHull)))
				upperHull.addAll(lowerHull);
			return upperHull;
		}

	
		/**
		 * Return 1 if point(x,y) is inside polygon;
		 *        0 if point(x,y) is outside polygon;
		 *        -1 if point(x,y) is on boundary of polygon
		 *        
		 * Casting horizontal ray from (x, y) to (+inf, y)
		 * and count the number of times the ray pass through edges
		 * If ray pass through edges odd times, (x,y) is inside polygon
		 * If (x,y) is on edges of polygon or at vertices, return false.
		 * 
		 * @param x
		 * @param y
		 * @return
		 */
		public int inInterior(double x, double y) {
			int intersectionCount = 0;
			
			// shooting horizontal ray from (x, y) to (+inf, y)
			
			for (int i=0; i<vertices.size(); i++) {
				Point2d s = vertices.get(i);
				Point2d e = vertices.get((i+1)%vertices.size());
				
	//			System.out.println("s: "+s+"; e: "+e);
	
				double minY, maxY;
				if (s.y <= e.y) { minY = s.y; maxY = e.y; }
				else { minY = e.y; maxY = s.y; }			
				
				// 1. y not in [minY, maxY], (s,e) cannot intersect with ray
				if (y < minY || y > maxY) {
	//				System.out.println("  ray outside bounding box of edge (s,e)");
					continue;
				}
				
				// 2. y in [minY, maxY]
				// 2.1 (s,e) is horizontal, minY == maxY == y
				if (s.y == e.y) {
					double minX, maxX;
					if (s.x <= e.x) { minX = s.x; maxX = e.x; }
					else { minX = e.x; maxX = s.x; }
					
	//				System.out.println("  minX: "+minX+"; maxX: "+maxX);
					
					//  the ray pass through entire (s,e), treat the edge slightly above the ray
					if (x < minX) {
	//					System.out.println("  ray pass through entire horizontal edge (s,e)");
						continue;
					}
					// (x, y) on the edge (s,e]
					if (x <= maxX) {
	//					System.out.println("  ray starts on edge (s,e)");
						return -1;
					}
					// edge (s,e) is to the left of ray, no intersection
	//				System.out.println("  horizontal edge (s,e) to the left of ray");
					continue;
				}
				
				// 2.2 y in [minY, maxY] and (s, e) not horizontal
				double intersectX = s.x + (((double) e.x) - s.x) * (y - s.y) / (e.y - s.y);
				// 2.2.1 intersection not on ray
				if (x > intersectX) {
	//				System.out.println("  non-horizontal edge (s,e) to the left of ray");
					continue;
				}
				// 2.2.2 intersection on ray 
	//			System.out.println("  intersectX: "+intersectX);
				// intersection is (x,y), i.e., (x,y) on edge [s, e]
				if (x == intersectX) {
	//				System.out.println("  ray starts on non-horizontal edge (s,e)");
					return -1;
				}
				// 2.2.3 ray pass through non-horizontal edge (s,e)
				if (y == s.y) { // ray pass through s
	//				System.out.println("  ray pass through s");
					if (e.y < y) {
						intersectionCount++;
	//					System.out.println("    e is lower than ray, count: "+intersectionCount);
						continue;
					}
					continue;
				}
				if (y == e.y) {// ray pass through e
	//				System.out.println("  ray pass through e");
					if (s.y < y) {
						intersectionCount++;
	//					System.out.println("  s is lower than ray, count: "+intersectionCount);
						continue;
					}
					continue;
				}
				// intersection is in (s,e)
				intersectionCount++;
	//			System.out.println("  ray pass through interior of (s,e), count: "+intersectionCount);
			}
			
			return intersectionCount % 2;
		}
		
		public Point2d closestPoint;
		public double closetOnBoundary(Point2d p) {
			Point2d s = vertices.get(vertices.size()-1);
			Point2d e = vertices.get(0);
			LineSegment2d seg = new LineSegment2d(s, e);
			double minD = seg.closest(p);
			closestPoint = seg.closestPoint;
			
			for (int i=0; i<vertices.size()-1; i++) {
				s = vertices.get(i);
				e = vertices.get(i+1);
				seg = new LineSegment2d(s, e);
				double d = seg.closest(p);
				if (d < minD) { minD = d; closestPoint = seg.closestPoint; }
			}
			return minD;
		}
		
		public void print(String linePrefix) {
			System.out.println(linePrefix+"vertices: "+vertices.size());
			for (Point2d p:vertices) {
				System.out.println(linePrefix+"  "+p);
			}
		}
	}
	
	
	public static void test1() {
		ArrayList<Point2d> points = new ArrayList<>();
		points.add(new Point2d(0,0));
		points.add(new Point2d(0,10));
		points.add(new Point2d(0,20));
		points.add(new Point2d(10,0));
		points.add(new Point2d(10,10));
		points.add(new Point2d(10,20));
		points.add(new Point2d(20,0));
		points.add(new Point2d(20,10));
		Polygon2d polygon = Polygon2d.convexHull(points);
		
		Polygon poly = new Polygon();
		for (Point2d p:polygon.vertices) {
			poly.addPoint((int)p.x, (int)p.y);
			System.out.println(p);
		}

		int[] xs =              new int[] {   0,     0,    10,    20,    20,   10,    20,    11,   11};
		int[] ys =              new int[] {   0,    20,    20,    10,     0,   10,    20,    19,   18};
		boolean[] expected = new boolean[]{true, false, false, false, false, true, false, false, true};
		int[] exp =             new int[] {  -1,     -1,    -1,    -1,   -1,    1,     0,    -1,    1};
		for (int i=0; i<xs.length; i++) {
			int x = xs[i];
			int y = ys[i];
			boolean expPoly = expected[i];
			int expTiny = exp[i];
			System.out.println(x+","+y+": Polygon.contains "+poly.contains(x,y)+" exp: "+expPoly
					+"; inInterior: "+polygon.inInterior(x,y)+"; exp: "+expTiny);
			if (poly.contains(x,y) != expPoly) { throw new RuntimeException(); }
			if (polygon.inInterior(x, y) != expTiny) { throw new RuntimeException(); }
		}		

		
		Line2d L00 = new Line2d(new Point2d(1,1), new Point2d(2,1));
		System.out.println("L00: "+L00);
		Line2d L45 = new Line2d(new Point2d(1,1), new Point2d(2,2));
		System.out.println("L45: "+L45);
		Line2d L90 = new Line2d(new Point2d(1,1), new Point2d(1,2));
		System.out.println("L90: "+L90);
		Line2d L135 = new Line2d(new Point2d(1,1), new Point2d(0,2));
		System.out.println("L135: "+L135);
		Line2d L135_2 = new Line2d(new Point2d(-1,-1), new Point2d(0,-2));
		System.out.println("L135_2: "+L135_2);
		Line2d L179 = new Line2d(new Point2d(100,1), new Point2d(0,1.1));
		System.out.println("L179: "+L179);
		Line2d L179_2 = new Line2d(new Point2d(100,2), new Point2d(0,2.1));
		System.out.println("L179_2: "+L179_2);
		
		LineSegment2d seg1 = new LineSegment2d(new Point2d(0,100), new Point2d(100,0));
		System.out.println("foot of (10, 10): "+seg1.foot(new Point2d(10,10)));
		System.out.println("foot of (0, 10): "+seg1.foot(new Point2d(0,10)));
		
		LineSegment2d seg2 = new LineSegment2d(new Point2d(10,0), new Point2d(20, 0));
		System.out.println("closest of (0,10): "+seg2.closest(new Point2d(0,10))+" "+seg2.closestPoint);   // 10,0
		System.out.println("closest of (10,10): "+seg2.closest(new Point2d(10,10))+" "+seg2.closestPoint); // 10,0
		System.out.println("closest of (11,10): "+seg2.closest(new Point2d(11,10))+" "+seg2.closestPoint); // 11,0
		System.out.println("closest of (19,10): "+seg2.closest(new Point2d(19,10))+" "+seg2.closestPoint); // 19,0
		System.out.println("closest of (20,10): "+seg2.closest(new Point2d(20,10))+" "+seg2.closestPoint); // 20,0
		System.out.println("closest of (21,10): "+seg2.closest(new Point2d(21,10))+" "+seg2.closestPoint); // 20,0
	}
	
	
	///////////////////////////////////////////////////////////

	public void paint(Graphics g) {
		super.paint(g);
		
		Graphics2D g2d = (Graphics2D) g;
//		g2d.drawLine(120, 50, 360, 50);
		
		ArrayList<Point2d> points = new ArrayList<>();
		points.add(new Point2d(0,0));
		points.add(new Point2d(0,100));
		points.add(new Point2d(0,200));
		points.add(new Point2d(100,0));
		points.add(new Point2d(100,100));
		points.add(new Point2d(100,200));
		points.add(new Point2d(200,0));
		points.add(new Point2d(200,100));
		Polygon2d polygon = Polygon2d.convexHull(points);
		
		Polygon poly = new Polygon();
		for (Point2d p:polygon.vertices) {
			poly.addPoint((int)p.x+50, (int)p.y+50);
			System.out.println(p);
		}
		g2d.drawPolygon(poly);
	}
	

	public static void main(String[] args) {
		//test1();
				
		SwingUtilities.invokeLater(new Runnable() {
			
			@Override
			public void run() {
				Geometry2D geo = new Geometry2D();
				geo.setSize(480, 400);
				geo.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
				geo.setLocationRelativeTo(null);
				
				geo.setVisible(true);
			}
		});
	}
}




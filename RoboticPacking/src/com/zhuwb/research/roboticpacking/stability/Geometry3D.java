package com.zhuwb.research.roboticpacking.stability;

public class Geometry3D {
	public static class Vector3d {
		public final double x,y,z;
		public static final Vector3d zero = new Vector3d(0,0,0);
		public static final Vector3d i = new Vector3d(1,0,0);
		public static final Vector3d j = new Vector3d(0,1,0);
		public static final Vector3d k = new Vector3d(0,0,1);

		public Vector3d(double x, double y, double z) {
			this.x = x;
			this.y = y;
			this.z = z;
		}

		/**
		 * @param other
		 * @return this + other
		 */
		public Vector3d add(Vector3d other) {
			return new Vector3d(this.x+other.x,this.y+other.y,this.z+other.z);
		}

		/**
		 * @param other
		 * @return this - other
		 */
		public Vector3d minus(Vector3d other) {
			return new Vector3d(this.x-other.x,this.y-other.y,this.z-other.z);
		}
		
		/**
		 * @param other
		 * @return -this
		 */
		public Vector3d negate() {
			return new Vector3d(-x, -y, -z);
		}

		/**
		 * (x i + y j + z k) x (x2 i + y2 j + z2 k) = (yz2-zy2) i + (zx2 - xz2) j + (xy2 - yx2) k
		 *    i  j  k
		 *    x  y  z
		 *   x2 y2 z2
		 * @param other
		 * @return this X other
		 */
		public Vector3d cross(Vector3d other) {
			return new Vector3d(y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x);
		}

		/**
		 * @param other
		 * @return <this, other>
		 */
		public double dot(Vector3d other) {
			return this.x * other.x + this.y * other.y + this.z * other.z;
		}
		
		/**
		 * Scalar product
		 * @param c
		 * @return c * this
		 */
		public Vector3d scale(double c) {
			return new Vector3d(c*x, c*y, c*z);
		}

		/**
		 * @return  Euclidean norm || this ||_2
		 */
		public double norm2() {
			return Math.sqrt(x*x+y*y+z*z);
		}
		
		public String toString() {
			return String.format("(%g,%g,%g)", x,y,z);
		}
				
		public String toStringIJK() {
			StringBuffer sb = new StringBuffer();
			if (x != 0) {
				sb.append(Double.toString(x)).append("i");
			}
			
			if (y > 0) {
				if (sb.length() > 0) {
					sb.append("+");
				}
				sb.append(Double.toString(y)).append("j");
			} if (y < 0) {
				sb.append("-").append(Double.toString(-y)).append("j");
			}
			
			if (z > 0) {
				if (sb.length() > 0) {
					sb.append("+");
				}
				sb.append(Double.toString(z)).append("k");
			} if (z < 0) {
				sb.append("-").append(Double.toString(-z)).append("k");
			}
			return sb.toString();
		}
		
	}
	
	public static class Point3d extends Vector3d {
		/**
		 * The origin of coordinate system.
		 */
		public static final Point3d O = new Point3d(0,0,0);
		
		public Point3d(double x, double y, double z) {
			super(x,y,z);
		}
		
		public String toString() {
			return "Point"+super.toString();
		}
	}
	
	public static void main(String[] args) {
		System.out.println("("+ Vector3d.i.toStringIJK() + ") x (" + Vector3d.j.toStringIJK()+") = "+ Vector3d.i.cross(Vector3d.j).toStringIJK());
		System.out.println("("+ Vector3d.j.toStringIJK() + ") x (" + Vector3d.i.toStringIJK()+") = "+ Vector3d.j.cross(Vector3d.i).toStringIJK());
		System.out.println("("+ Vector3d.i.toStringIJK() + ") x (" + Vector3d.k.toStringIJK()+") = "+ Vector3d.i.cross(Vector3d.k).toStringIJK());
		System.out.println("("+ Vector3d.k.toStringIJK() + ") x (" + Vector3d.i.toStringIJK()+") = "+ Vector3d.k.cross(Vector3d.i).toStringIJK());
		System.out.println("("+ Vector3d.j.toStringIJK() + ") x (" + Vector3d.k.toStringIJK()+") = "+ Vector3d.j.cross(Vector3d.k).toStringIJK());
		System.out.println("("+ Vector3d.k.toStringIJK() + ") x (" + Vector3d.j.toStringIJK()+") = "+ Vector3d.k.cross(Vector3d.j).toStringIJK());
	}
}

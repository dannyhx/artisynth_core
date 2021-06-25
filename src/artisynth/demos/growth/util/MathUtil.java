package artisynth.demos.growth.util;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import maspack.geometry.Face;
import maspack.geometry.Vertex3d;
import maspack.matrix.DenseMatrixBase;
import maspack.matrix.EigenDecomposition;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point2d;
import maspack.matrix.Point3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorBase;
import maspack.matrix.VectorNd;

public class MathUtil {

   public static final double ELIPSON = 1e-6;
   
   public static boolean isInRange(double x, double min, double max) {
      return (x > min && x < max); 
   }
   
   public static int compare(double a, double b) {
      return compare(a, b, ELIPSON);
   }
   
   public static int compare(double a, double b, double eps) {
      if (Math.abs(a-b) < eps) {
         return 0;
      }
      else if (a-b > 0) {
         return 1;
      }
      else  {
         return -1;
      }
   }
   
   public static boolean isZero(double a) {
      return (Math.abs (a) < ELIPSON); 
   }
   
   /**
    * Force all entries to be negative.
    */
   public static void negAbs(DenseMatrixBase M) {
      int m = M.rowSize();   // Number of rows (i.e. col length)
      int n = M.colSize();   // Number of cols (i.e. row length)
      
      for (int r = 0; r < m; r++) {
         for (int c = 0; c < n; c++) {
            double val = M.get (r, c);
            if (val > 0) {
               M.set(r,c, -val);
            }
         }
      }
   }
   
   /**
    * Force all entries to be position.
    */
   public static void abs(DenseMatrixBase M) {
      int m = M.rowSize();   // Number of rows (i.e. col length)
      int n = M.colSize();   // Number of cols (i.e. row length)
      
      for (int r = 0; r < m; r++) {
         for (int c = 0; c < n; c++) {
            double val = M.get (r, c);
            if (val < 0) {
               M.set(r,c, -val);
            }
         }
      }
   }
   
   public static Matrix2d outerProduct2x2(double[] a, double[] b) {
      Matrix2d op = new Matrix2d();
      outerProduct(a, b, op);
      
      return op;
   }
   
   public static void outerProduct(double[] a, double[] b, DenseMatrixBase mtx) {
      for (int r = 0; r < a.length; r++) {
         for (int c = 0; c < b.length; c++) {
            mtx.set (r,c, a[r]*b[c] );
         }
      }
   }
   
   public static double[] toArray(VectorBase vec) {
      double[] arr = new double[vec.size ()];
      for (int i = 0; i < vec.size (); i++) {
         arr[i] = vec.get (i);
      }
      return arr;
   }
   
   public static Point2d to2d(Point3d pnt) {
      return new Point2d( pnt.x, pnt.y );
   }
   
   public static Point3d to3d(Point2d pnt) {
      return new Point3d( pnt.x, pnt.y, 0 );
   }
   
   public static Point2d getClosestPtToLineSegment2d(Point2d P, Point2d A,
   Point2d B) 
   {
      Vector2d PA = P.sub (A);
      Vector2d BA = B.sub (A);
      
      double BA2 = BA.normSquared ();
      double dot = PA.dot (BA);
      
      double t = dot / BA2;
      
      Point2d C = new Point2d(
         A.x + BA.x*t, 
         A.y + BA.y*t);
      
      Vector2d CA = new Vector2d(C).sub (A);
      
      // Is closest line point beyond the segment head?
      if (BA.dot (CA) > 0 && BA.norm () < CA.norm()) {
         return B;
      }
      
      // Is closest line point below the segment tail?
      if (BA.dot (CA) < 0) {
         return A;
      }
      
      return C;
   }
   
   /**
    * Multiply matrix a and b together.
    */
   public static MatrixNd mul(DenseMatrixBase A, DenseMatrixBase B) {

      int m = A.rowSize ();   // number of rows (i.e. col length)
      
      int p = B.colSize ();   // number of col
      
      MatrixNd rv = new MatrixNd(m,p);
      mul(A, B, rv);
      
      return rv;
   }
   
   public static void mul(DenseMatrixBase A, DenseMatrixBase B, DenseMatrixBase rv) {
      int m = A.rowSize ();
      int n = A.colSize ();
      
      int p = B.colSize ();
      
      rv.setSize (m, p);
      
      // For each rv(m,p) cell...
      for (int i = 0; i < m; i++) {
         for (int j = 0; j < p; j++) {
 
            double cell = 0;
            
            // Compute the dot product of corresponding A's row and 
            // corresponding B's column
            for (int k = 0; k < n; k++) {
               cell += A.get (i,k) * B.get (k,j);
            }
            
            rv.set (i,j, cell);
         }
      }
   }
   
   public static void mul(DenseMatrixBase M, Vector v, Vector rv) {
      int m = M.rowSize ();
      int n = M.colSize ();
      
      if ( ! rv.isFixedSize ()) {
         rv.setSize (m);
      }
      
      // For each rv cell...
      for (int i = 0; i < m; i++) {
         double cell = 0;
         
         // Compute dot product of corresponding row in M and v
         for (int j = 0; j < n; j++) {
            cell += M.get (i, j) * v.get (j);
         }
         
         rv.set (i, cell);
      }
   }
   
   public static MatrixNd transpose(DenseMatrixBase a) {
      MatrixNd rv = new MatrixNd(a.colSize (), a.rowSize ());
      
      for (int i = 0; i < a.rowSize (); i++) {
         for (int j = 0; j < a.colSize (); j++) {
            rv.set (i,j,  a.get (j, i));
         }
      }
      
      return rv;
   }
   
   public static double sum(DenseMatrixBase a) {
      double sum = 0;
      
      for (int i = 0; i < a.rowSize (); i++) {
         for (int j = 0; j < a.colSize (); j++) {
            sum += a.get (i, j);
         }
      }
      
      return sum;
   }
   
   public static MatrixNd perp2x2(DenseMatrixBase a) {
      MatrixNd rv = new MatrixNd(2,2);
      rv.set (new double[] {
          a.get (1, 1), -a.get (0, 1),
         -a.get (1, 0),  a.get (0, 0)});
      return rv;
   }

   public static DenseMatrixBase getPositive(Matrix M) {
      // dynamicremesh.cpp get_positive(Mat<n,n>)
      
      int n = M.colSize ();
      
      // Decompose
      EigenDecomposition eig = new EigenDecomposition();
      eig.factor (M);
      
      // Retrieve eigenvectors and eigenvalues
      VectorNd eigVals = new VectorNd(n);
      MatrixNd eigVecs = new MatrixNd(n,n);
      eig.get (eigVals, null, eigVecs);
      
      double[] eigValsBuf = eigVals.getBuffer ();
      
      // Get the absolute of the eigenvalues. DAN Modified
      for (int i = 0; i < n; i++) {
//         eigValsBuf[i] = Math.max(eigValsBuf[i], 0.);
         eigValsBuf[i] = Math.abs(eigValsBuf[i]);
      }
      
      MatrixNd rv = new MatrixNd(n,n);
      rv.set (eigVecs);
      rv.mulDiagonalRight (eigVals);
      rv.mulTranspose (eigVecs);
      
      return rv;
   }
   
   public static MatrixNd sqrt(MatrixNd M) {
      // dynamicremesh.cpp sqrt(Mat<n,n>)
      
      int n = M.colSize ();
      
      // Decompose
      EigenDecomposition eig = new EigenDecomposition();
      eig.factor (M);
      
      // Retrieve eigenvectors and eigenvalues
      VectorNd eigVals = new VectorNd(n);
      MatrixNd eigVecs = new MatrixNd(n,n);
      eig.get (eigVals, null, eigVecs);
      
      double[] eigValsBuf = eigVals.getBuffer ();
      
      // Get the square root of the eigenvalues
      for (int i = 0; i < n; i++) {
         if (eigValsBuf[i] >= 0) {
            eigValsBuf[i] = Math.sqrt (eigValsBuf[i]);
         }
         else {
            eigValsBuf[i] = -Math.sqrt (-eigValsBuf[i]);
         }
      }
      
      // Reconstruct the original matrix using the square rooted eigenvalues.
      // M' = Q sqrt(A) Q^t
      // where Q is the per-column eigenvector matrix
      MatrixNd rv = new MatrixNd(n,n);
      rv.set (eigVecs);
      rv.mulDiagonalRight (eigVals);
      rv.mulTranspose (eigVecs);
      
      return rv;
   }
   
   public static double clamp(double x, double min, double max) {
      return Math.min ( Math.max (x, min) , max);
   }
   
   public static double normF(Matrix F) {
      return Math.sqrt ( norm2F(F) );
   }
   
   protected static double norm2F(Matrix F) {
      double a = 0;
      
      VectorNd col = new VectorNd(F.colSize ());
      for (int c = 0; c < F.colSize (); c++) {
         F.getColumn (c, col);
         a += col.normSquared ();
      }
      
      return a;
   }
   
   /**
    * Compute the wedge product of 2 one-forms, followed by the Hodge dual 
    * operation. Also known as the perp dot product for two 2D-vectors.
    * 
    * In other words, get the signed area of the parallelogram formed by vectors
    * u and v.
    * 
    * Assuming both vector tails lie on the origin (0,0), if signed area is
    * positive, then u to v to CCW, and if signed area is negative, then u to v
    * is CW.
    */
   public static double wedge(Vector2d u, Vector2d v) {
      return u.x * v.y - u.y * v.x;
   }
   
   public static double area(Point3d a, Point3d b, Point3d c) {
      Vector3d b_a = new Vector3d().sub (b, a);
      Vector3d c_a = new Vector3d().sub (c, a);
      
      return b_a.cross (c_a).norm () * 0.5;
   }
   
   public static double laymenAspect(Point3d a, Point3d b, Point3d c) {
      double a_b = a.distance (b);
      double a_c = a.distance (c);
      double b_c = b.distance (c);
      
      double minLen = min(min(a_b, a_c),b_c);
      double maxLen = max(max(a_b, a_c),b_c);
      
      return maxLen / minLen;
   }
   
   public static Point3d centroid(Face face) {
      return MathUtil.centroid (
         face.getPoint (0),
         face.getPoint (1),
         face.getPoint (2)
      );
   }
   
   public static Point3d centroid(Point3d a, Point3d b, Point3d c) {
      Point3d rv = (Point3d)new Point3d(a).add (b, c);
      rv.scale (1/3.0);
      
      return rv;
   }
   
   public static double minRadius(Point3d a, Point3d b, Point3d c) {
      Point3d centroid = centroid(a,b,c);
      
      double a_centroid = new Vector3d().sub (a, centroid).normSquared ();
      double b_centroid = new Vector3d().sub (b, centroid).normSquared (); 
      double c_centroid = new Vector3d().sub (c, centroid).normSquared ();
      
      return min(min(a_centroid, b_centroid), c_centroid);
   }
   
   public static double distFromPtToLine(Point3d p, Point3d l0, Point3d l1) {
      // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Vector_formulation
      Point3d a = l0;
      Point3d n = (Point3d)new Point3d().sub (l1, l0).normalize ();
      
      Vector3d a_p = new Vector3d().sub (a, p);
      double a_p_dot_n = a_p.dot (n);
      
      Vector3d rv = new Vector3d();
      rv.set (a_p);
      rv.scaledAdd (-a_p_dot_n, n);
      
      return rv.normSquared ();
   }
   
   public static Vector3d getNormal(Point3d a, Point3d b, Point3d c) {
      Vector3d b_a = new Vector3d().sub (b, a);
      Vector3d c_a = new Vector3d().sub (c, a);
      
      Vector3d rv = new Vector3d().cross (b_a, c_a);
      rv.normalize ();
      return rv;
   }
   
   public static double doubleDotProduct(SymmetricMatrix3d a, SymmetricMatrix3d b) {
      double sum = 0;
      for (int i = 0; i < 3; i++) {
         for (int j = 0; j < 3; j++) {
            sum += a.get (i, j) * b.get (i, j);
         }
      }
      return sum;
   }

   public static Matrix3d outerProduct(Vector3d a, Vector3d b) {
      return new Matrix3d( 
         a.x*b.x, a.x*b.y, a.x*b.z,
         a.y*b.x, a.y*b.y, a.y*b.z,
         a.z*b.x, a.z*b.y, a.z*b.z );
   }
   
   public static void setSymmetric2d(Matrix2d M) {
      M.m01 = 0.5*(M.m01 + M.m10);

      // Do not touch diagonal entries.

      M.m10 = M.m01;
   }
   
   public static double dihedralAngle(
      Point3d edgeHead, Point3d edgeTail, Vector3d n0, Vector3d n1) 
   {
      Vector3d edge = new Vector3d(edgeHead).sub (edgeTail).normalize ();

      Vector3d n0xn1 = new Vector3d(n0).cross (n1);
      
      double cosine = n0.dot (n1); 
      double sine = n0xn1.dot (edge);
      
      double theta = Math.atan2 (sine, cosine);
      return theta;
   }
   
   /**
    * Get the shortest distance between a point and an edge.
    * 
    * @param a Tail 
    * @param a Head 
    */
   public static double distanceBetweenPointAndLine(Point3d x, Point3d a, Point3d b) {
      Vector3d e = new Vector3d(b).sub (a);
      Vector3d xa = new Vector3d(x).sub(a);
      
      Vector3d xp = new Vector3d(e).scale (e.dot (xa)/e.dot (e));
      
      Vector3d xa_xp = new Vector3d(xa).sub (xp);
      return Math.max(xa_xp.norm (), 1e-3*e.norm());
   }
   
   /////////////
   // Average
   /////////////
   
   public static Point3d avg(Point3d p0, Point3d p1) {
      return (Point3d) new Point3d().add (p0, p1).scale (0.5);
   }
   
   protected static Vector3d avg(Vector3d p0, Vector3d p1) {
      return new Vector3d().add (p0, p1).scale (0.5);
   }
   
   protected static float[] avg(float[] p0, float[] p1) {
      float[] avg = new float[3];
      
      for (int i = 0; i < 3; i++) {
         avg[i] = (p0[i] + p1[i])*(float)0.5;
      }
         
      return avg;
   }
   
   protected static Vertex3d avg(Vertex3d v0, Vertex3d v1) {
      Point3d p0 = v0.getPosition ();
      Point3d p1 = v1.getPosition (); 
      
      Point3d avgPos = avg(p0, p1);
      
      return new Vertex3d(avgPos);
   }
   
   protected static VectorNd avg(VectorNd v0, VectorNd v1) {
      VectorNd avg = new VectorNd(v0.size());
      for (int i = 0; i < v0.size (); i++) {
         double v0i = v0.get (i);
         double v1i = v1.get (i);
         avg.set (i, (v0i+v1i)*0.5);
      }
      return avg;
   }
   
   public static Matrix3d avg(Matrix3d m0, Matrix3d m1) {
      Matrix3d avg = new Matrix3d();
      avg.add (m0);
      avg.add (m1);
      avg.scale (0.5);
      return avg;
   }
   
   ///
   
   
   public static Vector3d vecFromPtToLine(Point3d p, Point3d l0, Point3d l1) {
      // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Vector_formulation
      Point3d a = l0;
      Point3d n = (Point3d)new Point3d().sub (l1, l0).normalize ();
      
      Vector3d a_p = new Vector3d().sub (a, p);
      double a_p_dot_n = a_p.dot (n);
      
      Vector3d rv = new Vector3d();
      rv.set (a_p);
      rv.scaledAdd (-a_p_dot_n, n);
      
      return rv;
   }
   
   public static Point3d projectPoint2plane(
   Point3d p, Point3d q0, Point3d q1, Point3d q2) {
      // https://math.stackexchange.com/questions/444968/project-a-point-in-3d-on-a-given-plane
      Vector3d q10 = new Vector3d().sub (q1, q0);
      Vector3d q21 = new Vector3d().sub (q2, q1);
      
      Vector3d nrm = q10.cross (q21);
      nrm.normalize ();
      
      double nrmScale = new Point3d(p).sub (q0).dot (nrm);
      
      Point3d j = (Point3d)new Point3d(p).scaledAdd (-nrmScale, nrm);
      
      return j;
   }
   
   /**
    * Given a triangle in 3D space and a 3D vector that's along the plane of
    * the triangle, return the vector in tangent coordinates (x,y).
    * 
    * Origin is tri[0], e1 basis (i.e. tangent x axis) is origin to tri[1] 
    * normalized, and e2 basis (i.e. tangent y axis) is cross product of 
    * triangle normal and e1. 
    */
   public static Vector2d vec3dToTangentCoords(Point3d[] tri, Vector3d v) {
      Vector3d nrm = MathUtil.getNormal (tri[0], tri[1], tri[2]);
      
      Vector3d e1 = new Vector3d(tri[1]).sub(tri[0]).normalize ();
      Vector3d e2 = new Vector3d(nrm).cross (e1);
      
      double t1 = e1.dot (v);
      double t2 = e2.dot (v);
      
      return new Vector2d(t1, t2);
   }
   
   
   /**
    * Project vector A onto vector B.
    */
   public static Vector3d vectorProjection(Vector3d A, Vector3d B) {
      Vector3d normalizedB = new Vector3d();
      normalizedB.normalize (B);
      
      double bScale = A.dot (B);
      
      Vector3d projA = new Vector3d();
      projA.scaledAdd (bScale, B);
      
      return projA;
   }
   
   public static boolean isBaryCoordWithinTriangle(
   Vector2d bary, double elipson) {
      if (bary.x < -elipson || bary.y < -elipson ||
          bary.x + bary.y > 1 + elipson) 
      {
         return false;
      }
      else {
         return true;
      }
   }
   
   /**
    * physics.cpp/barycentric_weights
    * 
    * @param a
    * @param b
    * @param c
    * @return
    */
   public static Vector2d barycentricWeights(Point3d x, Point3d a, Point3d b) {
      Vector3d e = new Point3d(b).sub (a);
      Vector3d xa = new Point3d(x).sub (a);
      
      double t = e.dot (xa) / e.dot (e); 
      
      return new Vector2d(1-t, t);
   }
   
   public static VectorNd matToVec_colMajor(DenseMatrixBase M) {
      int m = M.rowSize();
      int n = M.colSize();
      
      VectorNd V = new VectorNd(m*n);
      
      for (int i = 0; i < m; i++) { 
         for (int j = 0; j < n; j++) {
            V.set(i + j*m, M.get(i,j)); 
         }
      }
      
      return V; 
   }
   
   public static MatrixNd vecToMat_colMajor(VectorNd V, int m, int n) {
      MatrixNd M = new MatrixNd(m, n);
      
      for (int i = 0; i < m; i++) {
         for (int j = 0; j < n; j++) {
            M.set(i, j, V.get (i + j*m));
         }
      }
      
      return M; 
   }

   
   ////////////////
   // Interpolation 
   ////////////////
   
   public static Vector3d linearInterpolation(Vector3d A, Vector3d B,
   double bScale) {
      Vector3d C = new Vector3d();
      C.scaledAdd (1-bScale, A);
      C.scaledAdd (bScale,   B);
      return C;
   }
   
   public static Vector3d linearInterpolation(Vector3d A, Vector3d B, Vector3d C,
   double aScale, double bScale) {
      Vector3d D = new Vector3d();
      D.scaledAdd (aScale, A);
      D.scaledAdd (bScale, B);
      D.scaledAdd (1-aScale-bScale, C);
      return D;
   }
   
   public static Point3d linearInterpolation(Point3d A, Point3d B, Point3d C,
   double aScale, double bScale) {
      Point3d D = new Point3d();
      D.scaledAdd (aScale, A);
      D.scaledAdd (bScale, B);
      D.scaledAdd (1-aScale-bScale, C);
      return D;
   }

   public static void main(String[] args) {
      Vector3d A = new Vector3d(-0.23,-0.88,2);
      Vector3d B = new Vector3d(-1,0,0);
      
      Vector3d C = vectorProjection (A, B);
      
      System.out.println (C);
   }
}

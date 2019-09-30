package _custom.cont;

import maspack.matrix.*;
import maspack.util.*;

public class ContinuousCollisions {

   public CCRV collideVertexTrianglePnts (
      Vector3d p0, Vector3d p1 ,
      Vector3d a0, Vector3d a1, 
      Vector3d b0, Vector3d b1, 
      Vector3d c0, Vector3d c1,
      double epsilon) {
      
      Vector3d pdiff = new Vector3d();
      Vector3d adiff = new Vector3d();
      Vector3d bdiff = new Vector3d();
      Vector3d cdiff = new Vector3d();

      pdiff.sub (p1, p0);
      adiff.sub (a1, a0);
      bdiff.sub (b1, b0);
      cdiff.sub (c1, c0);
      return collideVertexTriangleDiff (
         p0, pdiff, a0, adiff, b0, bdiff, c0, cdiff, epsilon);
   }

   public CCRV collideVertexTriangleDiff (
      Vector3d p0, Vector3d pdiff,
      Vector3d a0, Vector3d adiff,
      Vector3d b0, Vector3d bdiff,
      Vector3d c0, Vector3d cdiff,
      double epsilon) {
      
      Vector3d ap0 = new Vector3d();
      Vector3d ab0 = new Vector3d();
      Vector3d ac0 = new Vector3d();
      Vector3d apdiff = new Vector3d();
      Vector3d abdiff = new Vector3d();
      Vector3d acdiff = new Vector3d();

      ap0.sub (p0, a0);
      apdiff.sub (pdiff, adiff);
      ab0.sub (b0, a0);
      abdiff.sub (bdiff, adiff);
      ac0.sub (c0, a0);
      acdiff.sub (cdiff, adiff);

      Vector3d coeff0 = new Vector3d();
      Vector3d coeff1 = new Vector3d();
      Vector3d coeff2 = new Vector3d();

      /*
       * Quadratic Equation Qt^2 + Rt + S = AB(t)^AC(t)
       * which calculates vector orthogonal to vectors AB and AC at time t.
       */
      makeCrossProductQuadraticFromRays (
         coeff0, coeff1, coeff2, ab0, abdiff, ac0, acdiff);

      /*
       * The moments where the vector AP is orthogonal to the cross-product to
       * AB and AC correspond to moments of coplanarity of vertices p, a, b,
       * c. We take the dot-product of the vector quadratic equation above with
       * the vector AP(t), producing a scalar cubic equation, with roots
       * corresponding to moments of coplanarity
       *
       * Cubic Equation At^3 + Bt^2 + Ct + D = 0
       * for t values that make all points coplanar
       */
      double A = coeff2.dot (apdiff);
      double B = coeff2.dot(ap0) + coeff1.dot(apdiff);
      double C = coeff1.dot(ap0) + coeff0.dot(apdiff);
      double D = coeff0.dot(ap0);

      if (!cubicRootsMayBeInZeroToOne(A, B, C, D)) {
         return new CCRV(); // -1
      }
      double[] roots = new double[3];
      // look for roots on the interval [0,1]
      //int nr = CubicSolver.getRootsAlgebraic (roots, A, B, C, D);
      int nr = CubicSolver.getRoots (roots, A, B, C, D, 0.0, 1.0);

      Vector3d u = new Vector3d();
      Vector3d v = new Vector3d();
      Vector3d w = new Vector3d();
      double tmin = 2.0;
      Vector2d bary = null;
      for (int i=0; i<nr; i++) {
         double t = roots[i];
         if (t >= 0 && t <= 1 && t < tmin) {
            // evaluate point and triangle vectors at AP(t), AB(t) and AC(t)
            u.scaledAdd (t, abdiff, ab0);
            v.scaledAdd (t, acdiff, ac0);
            w.scaledAdd (t, apdiff, ap0);
            
            bary = isPointInsideTriangle (u, v, w, epsilon);
            if (bary != null) {
               tmin = t;
            }
         }
      }
      
      tmin = tmin != 2.0 ? tmin : -1.0;    // DANCOLEDIT: Used to be `tmin : -1.0`
      
      CCRV rv = new CCRV();
      rv.hitTime = tmin;   
      rv.bary = bary;
      return rv;
   }

   public CCRV collideEdgeEdgePnts (
      Vector3d a0, Vector3d a1, 
      Vector3d b0, Vector3d b1, 
      Vector3d c0, Vector3d c1,
      Vector3d d0, Vector3d d1,
      double epsilon) {
      
      Vector3d adiff = new Vector3d();
      Vector3d bdiff = new Vector3d();
      Vector3d cdiff = new Vector3d();
      Vector3d ddiff = new Vector3d();

      adiff.sub (a1, a0);
      bdiff.sub (b1, b0);
      cdiff.sub (c1, c0);
      ddiff.sub (d1, d0);
      return collideEdgeEdgeDiff (
         a0, adiff, b0, bdiff, c0, cdiff, d0, ddiff, epsilon);
   }


   public CCRV collideEdgeEdgeDiff (
      Vector3d a0, Vector3d adiff,
      Vector3d b0, Vector3d bdiff,
      Vector3d c0, Vector3d cdiff,
      Vector3d d0, Vector3d ddiff,
      double epsilon) {

      Vector3d ab0 = new Vector3d();      
      Vector3d abdiff = new Vector3d();
      Vector3d cd0 = new Vector3d();
      Vector3d cddiff = new Vector3d();
      Vector3d ac0 = new Vector3d();
      Vector3d acdiff = new Vector3d();

      ab0.sub (b0, a0);
      abdiff.sub (bdiff, adiff);
      cd0.sub (d0, c0);
      cddiff.sub (ddiff, cdiff);
      ac0.sub (c0, a0);
      acdiff.sub (cdiff, adiff);

      Vector3d coeff0 = new Vector3d();
      Vector3d coeff1 = new Vector3d();
      Vector3d coeff2 = new Vector3d();

      /*
       * Quadratic Equation Qt^2 + Rt + S = AB(t)^CD(t)
       * which calculates vector orthogonal to vectors AB and CD at time t.
       */
      makeCrossProductQuadraticFromRays (
         coeff0, coeff1, coeff2, ab0, abdiff, cd0, cddiff);

      /*
       * The moments where the vector AC is orthogonal to the cross-product to
       * AB and CD correspond to moments of coplanarity of vertices a, b, c,
       * d. We take the dot-product of the vector quadratic equation above with
       * the vector AC(t), producing a scalar cubic equation, with roots
       * corresponding to moments of coplanarity
       *
       * Cubic Equation At^3 + Bt^2 + Ct + D = 0
       * for t values that make all points coplanar
       */
      double A = coeff2.dot (acdiff);
      double B = coeff2.dot(ac0) + coeff1.dot(acdiff);
      double C = coeff1.dot(ac0) + coeff0.dot(acdiff);
      double D = coeff0.dot(ac0);

      //System.out.printf ("coefs: %g %g %g %g\n", A, B, C, D);

      if (!cubicRootsMayBeInZeroToOne(A, B, C, D)) {
         return new CCRV();
      }
      double[] roots = new double[3];
      // look for roots on the interval [0,1]
      //int nr = CubicSolver.getRootsAlgebraic (roots, A, B, C, D);
      int nr = CubicSolver.getRoots (roots, A, B, C, D, 0.0, 1.0);

      // System.out.println ("numr " + nr);
      // for (int i=0; i<nr; i++) {
      //    System.out.println ("  " + roots[i]);
      // }
      

      Vector3d u = new Vector3d();
      Vector3d v = new Vector3d();
      Vector3d w = new Vector3d();
      double tmin = 2.0;
      double dmin = Double.POSITIVE_INFINITY;
      double r = 0;
      double s = 0;
      
      double[] rv = new double[3];
      
      for (int i=0; i<nr; i++) {
         double t = roots[i];
         if (t >= 0 && t <= 1 && t < tmin) {
            // evaluate edge vectors AB(t), CD(t) and AC(t)
            u.scaledAdd (t, abdiff, ab0);
            v.scaledAdd (t, cddiff, cd0);
            w.scaledAdd (t, acdiff, ac0);
            minimumDistanceBetweenSegments(u, v, w, epsilon, rv);
            double curMinDist = rv[0];
            double cur_r = rv[1];
            double cur_s = rv[2];
            
            if (Math.abs(curMinDist) <= epsilon) {
               tmin = t;
               dmin = curMinDist;
               r = cur_r;
               s = cur_s;
            }
         }
      }
      
      tmin = tmin != 2.0 ? tmin : -1.0;
      
      CCRV ccrv = new CCRV();
      ccrv.hitTime = tmin;
      ccrv.bary = null;
      ccrv.minDist = dmin;
      ccrv.r = r;
      ccrv.s = s;
      return ccrv;
   }

   void makeCrossProductQuadraticFromRays (
      Vector3d c0, Vector3d c1, Vector3d c2, 
      Vector3d oa, Vector3d da, Vector3d ob, Vector3d db) {

      c0.set (oa.y*ob.z - oa.z*ob.y,
              oa.z*ob.x - oa.x*ob.z,
              oa.x*ob.y - oa.y*ob.x);

      c1.set (oa.y*db.z + da.y*ob.z - oa.z*db.y - da.z*ob.y,
              oa.z*db.x + da.z*ob.x - oa.x*db.z - da.x*ob.z,
              oa.x*db.y + da.x*ob.y - oa.y*db.x - da.y*ob.x);

      c2.set (da.y*db.z - da.z*db.y,
              da.z*db.x - da.x*db.z,
              da.x*db.y - da.y*db.x);

   }

   boolean computeBarycentricCoords (
      Vector2d bary, Vector3d p,
      Vector3d a, Vector3d b, Vector3d c, double epsilon) {

      Vector3d u = new Vector3d();
      Vector3d v = new Vector3d();
      Vector3d w = new Vector3d();

      u.sub (b, a);
      v.sub (c, a);
      w.sub (p, a);
      return computeBarycentricCoords (bary, u, v, w, epsilon);
   }

   boolean computeBarycentricCoords (
      Vector2d bary, Vector3d u, Vector3d v, Vector3d w, double epsilon) {

      double uv = u.dot(v);
      double vv = v.dot(v);
      double uu = u.dot(u);

      double wv = w.dot(v);
      double wu = w.dot(u);

      double denom = uv*uv - uu*vv;
      if (Math.abs(denom) <= epsilon) {
         return false;
      }
      else {
         bary.x = (uv * wv - vv * wu) / denom; // s 
         bary.y = (uv * wu - uu * wv) / denom; // t
         return true;
      }
   }           

   Vector2d isPointInsideTriangle (
      Vector3d u, Vector3d v, Vector3d w, double epsilon) {

      Vector2d bary = new Vector2d();
      if (!computeBarycentricCoords (bary, u, v, w, epsilon)) {
         return null;
      }
      if (bary.x < -epsilon || bary.y < -epsilon ||
          bary.x + bary.y > 1 + epsilon) {
         return null;
      }
      return bary;
   }

   /**
    * @param rv 
    * Return values will be stored in here:
    * [minDist, r, s] where 
    * minDist = Minimum distance between two edges at the time of collision.
    *       r = Fraction along line segment from tail to head of edge0 where collision occurred.
    *       s = Fraction along line segment from tail to head of edge1 where collision occurred.
    */
   void minimumDistanceBetweenSegments (
      Vector3d u, Vector3d v, Vector3d w, double epsilon, double[] rv) {

      double uv = u.dot(v);
      double vv = v.dot(v);
      double uu = u.dot(u);

      double wv = w.dot(v);
      double wu = w.dot(u);

      double denom = uu * vv - uv * uv;
      double sD = denom;
      double tD = denom;
      double sN = (vv * wu - uv * wv);
      double tN = (uv * wu - uu * wv);

      if (Math.abs(denom) <= epsilon) {
         sN = 0.0;
         sD = 1.0;
         tN = -wv;
         tD = vv;
      }
      else {
         if (sN < 0.0) {
            sN = 0.0;
            tN = -wv;
            tD = vv;
         }
         else if (sN > sD) {
            sN = sD;
            tN = uv - wv;
            tD = vv;
         }
      }

      if (tN < 0.0) {
         tN = 0.0;
         if (wu < 0.0) {
            sN = 0.0;
         }
         else if (wu > uu) {
            sN = sD;
         }
         else {
            sN = wu;
            sD = uu;
         }
      }
      else if (tN > tD) {
         tN = tD;
         if ((wu + uv) < 0.0) {
            sN = 0.0;
         }
         else if ((wu + uv) > uu) {
            sN = sD;
         }
         else {
            sN = (wu + uv);
            sD = uu;
         }
      }

      double s = (Math.abs(sN) < epsilon ? 0.0 : sN / sD);
      double t = (Math.abs(tN) < epsilon ? 0.0 : tN / tD);

      Vector3d r = new Vector3d();
      r.scaledAdd (-s, u, w);
      r.scaledAdd (t, v);
      double minDist = r.norm();
      
      rv[0] = minDist; 
      rv[1] = 1-s;   // Bugfixed
      rv[2] = 1-t;   // Bugfixed
   }

   boolean cubicRootsMayBeInZeroToOne (double a, double b, double c, double d) {
      
      double absA = Math.abs(a);
      double absB = Math.abs(b);
      double sgnA = Math.signum(a);

      // |d| > |b| + |c| + |a|
      if (Math.abs(d) > absB + Math.abs(c) + absA) {
         return false;
      }
      // all coefficients have the same sign
      if (sgnA == Math.signum(b) &&
          sgnA == Math.signum(c) &&
          sgnA == Math.signum(d)) {
         return false;
      }
      // d,c > 0 and d + c > |a| + |b|
      if (d > 0.0 && c > 0.0 && d + c > absA + absB) {
         return false;
      }
      // d,c < 0 and d + c < -|a| - |b|
      if (d < 0.0 && c < 0.0 && d + c < -absA - absB) {
         return false;
      }
      
      return true;
   }

   public static void main (String[] args) {
   }

}

package _custom.cont;

import maspack.geometry.HalfEdge;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

/** 
 * Space-time path of an edge, across a single time step.
 */
public class SweptEdge extends BoundablePointArray {

   // Convenient array indices that represent the starting and ending 
   // head (H) and tail (T) vertex positions of the traversed edge.
   // (H0,T0) -> (H1,T1)
   
   public final int H1 = 0;
   public final int T1 = 1;
   public final int H0 = 2;
   public final int T0 = 3;
   
   /** Traversing edge. */
   HalfEdge myEdge;

   /** Space-time path of an edge, across a single time step. */   
   public SweptEdge (HalfEdge edge, Point3d[] oldPositions) {
      super (4);
      myEdge = edge.getPrimary();
      Vertex3d head = myEdge.getHead();
      Vertex3d tail = myEdge.getTail();

      myPnts[H1] = head.getPosition();
      myPnts[T1] = tail.getPosition();
      myPnts[H0] = oldPositions[head.getIndex()];
      myPnts[T0] = oldPositions[tail.getIndex()];
   }  
   
   /** Is the path length zero? */
   public boolean isSweepIdle(double elipson) {
      return (
         myPnts[H1].distance (myPnts[H0]) < elipson &&
         myPnts[T1].distance (myPnts[T0]) < elipson
      );
   }
   
   /** Get the instanteous location of the edge's head vertex,
    * 
    * @param t 
    * Fraction of the path, ranging from 0 to 1. For example, 0.5 returns the
    * head vertex that's half-way along its traversed path.
    */
   public Point3d computeInstanteousHead(double t) {
      Point3d instHead = new Point3d();
      instHead.sub (myPnts[H1], myPnts[H0]);
      instHead.scale (t);
      instHead.add (myPnts[H0]);
      
      if (X != null)
         instHead.transform (X);
      
      return instHead;
   }
   
   /** Get the instanteous location of the edge's tail vertex.
    * 
    * @param t 
    * Fraction of the path, ranging from 0 to 1. For example, 0.5 returns the
    * tail vertex that's half-way along its traversed path.
    */
   public Point3d computeInstanteousTail(double t) {
      Point3d instTail = new Point3d();
      instTail.sub (myPnts[T1], myPnts[T0]);
      instTail.scale (t);
      instTail.add (myPnts[T0]);
      
      if (X != null)
         instTail.transform (X);
      
      return instTail;
   }
   
   /** Get the instanteous location of the edge within the traversed path.
    * 
    * @param t 
    * Fraction of the path, ranging from 0 to 1. For example, 0.5 returns the
    * edge that's half-way along its traversed path.
    * 
    * @return 
    * Instanteous edge in vector form.
    */
   public Vector3d computeInstanteousEdgeVec(double t) {
      Point3d instHead = computeInstanteousHead (t);
      Point3d instTail = computeInstanteousTail (t);
      
      Vector3d instEdgeVec = new Vector3d();
      instEdgeVec.sub (instHead, instTail);
      
      return instEdgeVec;
   }
   
   /** Get the instanteous location of a point in the edge.
    * 
    * @param t 
    * Fraction of the path, ranging from 0 to 1. For example, 0.5 returns the
    * edge that's half-way along its traversed path.
    * 
    * @param s 
    * Fraction of the edge, ranging from 0 to 1. For example, 0.5 refers to the
    * point that's in the middle of the edge.
    */
   public Point3d computeInstanteousEdgePoint(double t, double s) {
      Vector3d instEdgeVec = computeInstanteousEdgeVec(t);
      Point3d instTail = computeInstanteousTail (t);
      
      Point3d edgePt = new Point3d();
      edgePt.scaledAdd (s, instEdgeVec);
      edgePt.add (instTail); 
      
      return edgePt;
   }
   
   /** Get the instanteous location of the edge within the traversed path.
    * 
    * @param t 
    * Fraction of the path, ranging from 0 to 1. For example, 0.5 returns the
    * edge that's half-way along its traversed path.
    * 
    * @return
    * Head and tail vertex locations of the instanteous edge.
    */
   public Point3d[] computeInstanteousEdgePoints(double t) {
      Point3d[] ePnts = new Point3d[2];
      ePnts[0] = computeInstanteousHead (t);
      ePnts[1] = computeInstanteousTail (t);
      return ePnts;
   }
   
   /**
    * Given this edge and another edge (se1) at time t, calculate the closest
    * points of both edges.
    * 
    * @param se1
    * Another traversed edge.
    * 
    * @param t 
    * Time within the traversal of both edges that the closest points should
    * be determined. [0,1].
    * 
    * @param out_e0 
    * Closest point of this edge. Output parameter.
    * 
    * @param out_e1
    * Closest point of another edge. Output parameter.
    * 
    * @param rs 
    * Closest point of this edge and another edge. Each point is represented as 
    * a fraction along the line segment from tail to head of the corresponding
    * edge. [0,1]. Output parameter.
    * 
    * @param epsilon 
    * 1e-10 is sufficient.
    * 
    * @return 
    * Distance between the two closest points.
    */
   public double computeClosestEdgePairPoints(SweptEdge se1, double t, 
   Point3d out_e0, Point3d out_e1, Vector2d rs, double epsilon) {
      // Borrowed from ContinuousCollider.minimumDistanceBetweenSegments
      
      Point3d[] pts0 = this.createTransformedPoints ();
      Point3d[] pts1 = se1.createTransformedPoints ();
      
      Vector3d ab0 = new Vector3d().sub (pts0[T0], pts0[H0]);
      Vector3d cd0 = new Vector3d().sub (pts1[T0], pts1[H0]);
      Vector3d ac0 = new Vector3d().sub (pts1[H0], pts0[H0]);
      
      Vector3d adiff = new Vector3d().sub (pts0[H1], pts0[H0]);
      Vector3d bdiff = new Vector3d().sub (pts0[T1], pts0[T0]);
      Vector3d cdiff = new Vector3d().sub (pts1[H1], pts1[H0]);
      Vector3d ddiff = new Vector3d().sub (pts1[T1], pts1[T0]);
      
      Vector3d abdiff = new Vector3d().sub (bdiff, adiff);
      Vector3d cddiff = new Vector3d().sub (ddiff, cdiff);
      Vector3d acdiff = new Vector3d().sub (cdiff, adiff);
      
      Vector3d u = new Vector3d().scaledAdd (t, abdiff, ab0);
      Vector3d v = new Vector3d().scaledAdd (t, cddiff, cd0);
      Vector3d w = new Vector3d().scaledAdd (t, acdiff, ac0);
      
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
      double aT = (Math.abs(tN) < epsilon ? 0.0 : tN / tD);

      Vector3d r = new Vector3d();
      r.scaledAdd (-s, u, w);
      r.scaledAdd (aT, v);

      double s0 = 1-s;   // Bugfixed
      double s1 = 1-aT;   // Bugfixed
      
      out_e0.set( this.computeInstanteousEdgePoint (t, s0) );
      out_e1.set( se1.computeInstanteousEdgePoint  (t, s1) );
      
      if (rs != null) {
         rs.x = s0;
         rs.y = s1;
      }
      
      return r.norm();   // mindist
   }
   

   
   public static void main(String[] args) {
      HalfEdge he0 = new HalfEdge();
      he0.head = new Vertex3d(1,1,0);
      he0.tail = new Vertex3d(1,0,0);   
      
      he0.head.setIndex (0);
      he0.tail.setIndex (1);
      
      SweptEdge se0 = new SweptEdge(he0, new Point3d[] {
         new Point3d(0,1,0), 
         new Point3d(0,0,0)
      });
      
      HalfEdge he1 = new HalfEdge();
      he1.head = new Vertex3d(2,0,1);
      he1.tail = new Vertex3d(2,0,0.3);
      
      he1.head.setIndex (0);
      he1.tail.setIndex (1);
      
      SweptEdge se1 = new SweptEdge(he1, new Point3d[] {
         new Point3d(0,0,1), 
         new Point3d(0,0,0.3)
      });
      
      Point3d cp0 = new Point3d();
      Point3d cp1 = new Point3d();
      double minDist = se0.computeClosestEdgePairPoints (se1, 1, cp0, cp1, null, 1e-6);
      
      System.out.printf ("MinDist: %.4f, CP0: %s, CP1: %s", 
         minDist, 
         cp0.toString ("%.2f"), 
         cp1.toString ("%.2f")
      );
      
   }
   
   
   
   /* --- Functions for Continuous Collision Response --- */
   
   public void saveCurrentPositions() {
      myPnts1Backup = new Point3d[myPnts.length/2];
      
      myPnts1Backup[H1] = new Point3d( myPnts[H1] );
      myPnts1Backup[T1] = new Point3d( myPnts[T1] );
   }
   
   public void loadCurrentPositions() {
      myPnts[H1].set( myPnts1Backup[H1] );
      myPnts[T1].set( myPnts1Backup[T1] );
   }
   
   public void copyPrevious2CurrentPositions() {
      myPnts[H1].set ( myPnts[H0] );
      myPnts[T1].set ( myPnts[T0] );
   }
   
   public Vector3d[] computeAvgVelocity(double t) {
      myAvgVels = new Vector3d[myPnts.length/2];
      
      myAvgVels[H1] = new Vector3d();
      myAvgVels[H1].sub (myPnts[H1], myPnts[H0]);
      myAvgVels[H1].scale (1/t);
      
      myAvgVels[T1] = new Vector3d();
      myAvgVels[T1].sub (myPnts[T1], myPnts[T0]);
      myAvgVels[T1].scale (1/t);
      
      return myAvgVels;
   }
   
}

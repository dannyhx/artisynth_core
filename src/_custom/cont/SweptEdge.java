package _custom.cont;

import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import _custom.cont.BoundablePointArray;
import maspack.geometry.*;

public class SweptEdge extends BoundablePointArray {

   public final int H1 = 0;
   public final int T1 = 1;
   public final int H0 = 2;
   public final int T0 = 3;
   
   HalfEdge myEdge;
   protected RigidTransform3d X;

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
   
   public boolean isSweepIdle(double elipson) {
      return (
         myPnts[H1].distance (myPnts[H0]) < elipson &&
         myPnts[T1].distance (myPnts[T0]) < elipson
      );
   }
   
   public Point3d computeInstanteousHead(double t) {
      Point3d instHead = new Point3d();
      instHead.sub (myPnts[H1], myPnts[H0]);
      instHead.scale (t);
      instHead.add (myPnts[H0]);
      
      if (X != null)
         instHead.transform (X);
      
      return instHead;
   }
   
   public Point3d computeInstanteousTail(double t) {
      Point3d instTail = new Point3d();
      instTail.sub (myPnts[T1], myPnts[T0]);
      instTail.scale (t);
      instTail.add (myPnts[T0]);
      
      if (X != null)
         instTail.transform (X);
      
      return instTail;
   }
   
   public Vector3d computeInstanteousEdgeVec(double t) {
      Point3d instHead = computeInstanteousHead (t);
      Point3d instTail = computeInstanteousTail (t);
      
      Vector3d instEdgeVec = new Vector3d();
      instEdgeVec.sub (instHead, instTail);
      
      return instEdgeVec;
   }
   
   public Point3d computeInstanteousEdgePoint(double t, double s) {
      Vector3d instEdgeVec = computeInstanteousEdgeVec(t);
      Point3d instTail = computeInstanteousTail (t);
      
      Point3d edgePt = new Point3d();
      edgePt.scaledAdd (s, instEdgeVec);
      edgePt.add (instTail); 
      
      return edgePt;
   }
   
   public Vector3d computeEdgeVelocity(double s) {
      Point3d srtEdgePt = computeInstanteousEdgePoint (0, s);
      Point3d endEdgePt = computeInstanteousEdgePoint (1, s);
      
      Vector3d velo = new Vector3d();
      velo.sub (endEdgePt, srtEdgePt);
      
      return velo;
   }
   
   public double computeClosestEdgePairPoints(SweptEdge se1, double t, 
   Point3d out_e0, Point3d out_e1, double epsilon) {
      // Borrowed from ContinuousCollider.minimumDistanceBetweenSegments
      
      Point3d[] pts0 = this.myPnts;
      Point3d[] pts1 = se1.myPnts;
      
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
      double minDist = se0.computeClosestEdgePairPoints (se1, 1, cp0, cp1, 1e-6);
      
      System.out.printf ("MinDist: %.4f, CP0: %s, CP1: %s", 
         minDist, 
         cp0.toString ("%.2f"), 
         cp1.toString ("%.2f")
      );
      
   }
}

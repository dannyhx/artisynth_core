package _custom.cont;

import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import _custom.cont.BoundablePointArray;
import maspack.geometry.*;

public class SweptEdge extends BoundablePointArray {

   HalfEdge myEdge;
   protected RigidTransform3d X;

   public SweptEdge (HalfEdge edge, Point3d[] oldPositions) {
      super (4);
      myEdge = edge.getPrimary();
      Vertex3d head = myEdge.getHead();
      Vertex3d tail = myEdge.getTail();

      myPnts[0] = head.getPosition();
      myPnts[1] = tail.getPosition();
      myPnts[2] = oldPositions[head.getIndex()];
      myPnts[3] = oldPositions[tail.getIndex()];
   }  
   
   public boolean isSweepIdle(double elipson) {
      return (
         myPnts[0].distance (myPnts[2]) < elipson &&
         myPnts[1].distance (myPnts[3]) < elipson
      );
   }
   
   public Point3d computeInstanteousHead(double t) {
      Point3d instHead = new Point3d();
      instHead.sub (myPnts[0], myPnts[2]);
      instHead.scale (t);
      instHead.add (myPnts[2]);
      
      if (X != null)
         instHead.transform (X);
      
      return instHead;
   }
   
   public Point3d computeInstanteousTail(double t) {
      Point3d instTail = new Point3d();
      instTail.sub (myPnts[1], myPnts[3]);
      instTail.scale (t);
      instTail.add (myPnts[3]);
      
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
}

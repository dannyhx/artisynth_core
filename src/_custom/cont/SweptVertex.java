package _custom.cont;

import _custom.cont.BoundablePointArray;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;

public class SweptVertex extends BoundablePointArray {

   public final int V0 = 1;
   public final int V1 = 0;
   
   Vertex3d myVertex;
   public RigidTransform3d X;

   public SweptVertex (Vertex3d vertex, Point3d[] oldPositions) {
      super(2);
      myVertex = vertex;
      
      myPnts[V1] = myVertex.getPosition();
      myPnts[V0] = oldPositions[myVertex.getIndex()];
   }
   
   public boolean isSweepIdle(double elipson) {
      return (myPnts[V1].distance (myPnts[V0]) < elipson);
   }
   
   public Point3d computeInstanteousPoint(double t) {
      Point3d instPt = new Point3d();
      instPt.sub (myPnts[V1], myPnts[V0]);
      instPt.scale (t);
      instPt.add (myPnts[V0]);
      
      if (X != null)
         instPt.transform (X);
      
      return instPt;
   }
}

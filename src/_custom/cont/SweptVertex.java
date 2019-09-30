package _custom.cont;

import _custom.cont.BoundablePointArray;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class SweptVertex extends BoundablePointArray {

   Vertex3d myVertex;

   public SweptVertex (Vertex3d vertex, Point3d[] oldPositions) {
      super(2);
      myVertex = vertex;
      
      myPnts[0] = myVertex.getPosition();
      myPnts[1] = oldPositions[myVertex.getIndex()];
   }
   
   public boolean isSweepIdle(double elipson) {
      return (myPnts[0].distance (myPnts[1]) < elipson);
   }
   
   public Point3d computeInstanteousPoint(double t) {
      Point3d instPt = new Point3d();
      instPt.sub (myPnts[0], myPnts[1]);
      instPt.scale (t);
      instPt.add (myPnts[1]);
      
      return instPt;
   }
}

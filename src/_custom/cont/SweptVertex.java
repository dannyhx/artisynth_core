package _custom.cont;

import _custom.cont.BoundablePointArray;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.util.DataBuffer;

public class SweptVertex extends BoundablePointArray {

   public final int V1 = 0;
   public final int V0 = 1;
   
   Vertex3d myVertex;
   

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
 
   
   
   
   
   
   
   
   /* --- Functions for Continuous Collision Response --- */
   
   public void saveCurrentPositions() {
      myPnts1Backup = new Point3d[myPnts.length/2];
      myPnts1Backup[V1] = new Point3d( myPnts[V1] );
   }
   
   public void loadCurrentPositions() {
      myPnts[V1].set( myPnts1Backup[V1] );
   }
   
   public void copyPrevious2CurrentPositions() {
      myPnts[V1].set ( myPnts[V0] );
   }
   
//   public Vector3d[] computeAvgVelocity(double t) {
//      myAvgVels = new Vector3d[myPnts.length/2];
//      myAvgVels[V1] = new Vector3d();
//      
//      myAvgVels[V1].sub (myPnts[V1], myPnts[V0]);
//      myAvgVels[V1].scale (1/t);
//      
//      return myAvgVels;
//   }
}

package _custom.cont;

import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Matrix3d;
import maspack.geometry.*;

public class BoundablePointArray implements Boundable {

   protected Point3d[] myPnts;

   public BoundablePointArray (int size) {
      myPnts = new Point3d[size];
   }
   
   public int numPoints() {
      return myPnts.length;
   }

   public Point3d getPoint (int idx) {
      return myPnts[idx];
   }
   
   public Point3d[] getPoints () {
      return myPnts;
   }

   public void computeCentroid (Vector3d centroid) {
      centroid.set (myPnts[0]);
      for (int i=1; i<myPnts.length; i++) {
         centroid.add (myPnts[i]);
      }
      centroid.scale (1/myPnts.length);
   }

   public void updateBounds (Vector3d min, Vector3d max) {
      for (int i=0; i<myPnts.length; i++) {
         myPnts[i].updateBounds (min, max);
      }
   }
   
   public double computeCovariance (Matrix3d C) {
      return -1;
   }
   
   public Point3d[] createTransformedPoints(RigidTransform3d X) {
      Point3d[] xPts = new Point3d[ numPoints() ];
      
      for (int p = 0; p < numPoints(); p++) {
         xPts[p] = new Point3d( myPnts[p] );
         xPts[p].transform (X);
      }
      
      return xPts;
   }
}

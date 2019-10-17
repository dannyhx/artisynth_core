package _custom.cont;

import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.util.DataBuffer;
import maspack.matrix.Matrix3d;
import maspack.geometry.*;

public class BoundablePointArray implements Boundable {

   protected Point3d[] myPnts;
   public RigidTransform3d X;
   
   /* --- Stored Meta Data for Continuous Collision --- */

   protected Point3d[] myPnts1Backup;
   protected Vector3d[] myAvgVels;
   
   
   
   
   
   

   public BoundablePointArray (int size) {
      myPnts = new Point3d[size];
   }
   
   public int numPoints() {
      return myPnts.length;
   }

   public Point3d getPoint (int idx) {
      return myPnts[idx];
   }
   
   public Point3d createTransformedPoint(int idx) {
      Point3d xPnt = new Point3d(myPnts[idx]);
      xPnt.transform (this.X);
      return xPnt;
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
   
   public Point3d[] createTransformedPoints() {
      Point3d[] xPts = new Point3d[ numPoints() ];
      
      for (int p = 0; p < numPoints(); p++) {
         xPts[p] = new Point3d( myPnts[p] );
         xPts[p].transform (this.X);
      }
      
      return xPts;
   }
   
   public Vector3d computeDisplacement(int p) {
      int numMovingPts = myPnts.length/2;
      
      Vector3d disp = new Vector3d();
      disp.sub (myPnts[p+numMovingPts], myPnts[p]);
      
      disp.transform (this.X);
      
      return disp;
   }

   
   
   
   
   /* --- Functions for Continuous Collision Response --- */
   
   public void saveCurrentPositions() {
      throw new RuntimeException("Function needs to be overriden.");
   }
   
   public void loadCurrentPositions() {
      throw new RuntimeException("Function needs to be overriden.");
   }
   
   public void copyPrevious2CurrentPositions() {
      throw new RuntimeException("Function needs to be overriden.");
   }
   
   public Vector3d[] computeAvgVelocity(double t) {
      throw new RuntimeException("Function needs to be overriden.");
   }
}

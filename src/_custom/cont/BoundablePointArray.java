package _custom.cont;

import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.util.DataBuffer;
import maspack.matrix.Matrix3d;
import maspack.geometry.*;

/** Represents a single AABB that encloses the space-time path of a mesh 
 *  feature, across a single time step. */
public class BoundablePointArray implements Boundable {

   /** Boundary points that comprise the space-time path. AABB will be big
    *  enough to enclose all the points. 
    *
    *  By convention, the first half of the array is comprised of points 
    *  that represents the mesh feature at the END of the path, while the 
    *  second half of the array corresponds to the mesh feature at the START of 
    *  the path.
    */
   protected Point3d[] myPnts;
   
   /** What transformation matrix should be used when calling the transformation
    *  functions (e.g. {@link #createTransformedPoints()})?*/
   public RigidTransform3d X;
   
   /* --- Stored Meta Data for Continuous Collision --- */

   protected Point3d[] myPnts1Backup;
   protected Vector3d[] myAvgVels;
   
   
   
   /** 
    * Represents a single AABB that encloses the space-time path of a mesh 
    * feature. 
    *
    * @param size
    * Number of boundary points that comprise the path. 
    */
   public BoundablePointArray (int size) {
      myPnts = new Point3d[size];
   }
   
   public int numPoints() {
      return myPnts.length;
   }

   public Point3d getPoint (int idx) {
      return myPnts[idx];
   }
   
   /** Retrieve a transformed point. Transformation matrix used is according 
    *  to {@link #X}.
    */
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
   
   /** Retrieve all the points with transformation applied. Transformation 
    *  matrix used is according to {@link #X}.
    */
   public Point3d[] createTransformedPoints() {
      Point3d[] xPts = new Point3d[ numPoints() ];
      
      for (int p = 0; p < numPoints(); p++) {
         xPts[p] = new Point3d( myPnts[p] );
         xPts[p].transform (this.X);
      }
      
      return xPts;
   }
   
   /** Compute the displacement of a point across the space-time path. 
    *  Transformation matrix used is according to {@link #X}.
    *
    * @param p 
    * Identifies the moving point. Must be an index within the first half of the 
    * {@link #myPnts} array. 
    */
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

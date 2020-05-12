package artisynth.demos.growth.collision;

import artisynth.demos.growth.util.MathUtil;
import maspack.geometry.Face;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/** Space-time path of a triangle, across a single time step. */
public class SweptTriangle extends BoundablePointArray {

   // Convenient array indices that represent the starting and ending 
   // vertex positions of the traversed triangle.
   // (A0,B0,C0) -> (A1,B1,C1)
   
   public final int A1 = 0;
   public final int B1 = 1;
   public final int C1 = 2;
   
   public final int A0 = 3;
   public final int B0 = 4;
   public final int C0 = 5;

   /** Traversing triangle. */
   Face myFace;
   
   /** Space-time path of a triangle, across a single time step. */
   public SweptTriangle (Face face, Point3d[] oldPositions) {
      super (6);
      myFace = face;
      Vertex3d[] vtxs = face.getVertices();
      if (vtxs.length != 3) {
         throw new IllegalArgumentException (
            "face is not a triangle (does not have three vertices)");
      }
      myPnts[A1] = vtxs[0].getPosition();
      myPnts[B1] = vtxs[1].getPosition();
      myPnts[C1] = vtxs[2].getPosition();

      myPnts[A0] = oldPositions[vtxs[0].getIndex()];
      myPnts[B0] = oldPositions[vtxs[1].getIndex()];
      myPnts[C0] = oldPositions[vtxs[2].getIndex()];
   }  

   /** Get the instanteous location of the triangle.
    * 
    * @param t 
    * Fraction of the path, ranging from 0 to 1. For example, 0.5 returns the
    * triangle that's half-way along its traversed path.
    * 
    * @return 
    * Vertex positions of the instanteous triangle.
    */
   public Point3d[] computeInstanteousTriangle(double t) {
      Point3d[] tri = new Point3d[3];
      
      for (int i = 0; i < 3; i++) {
         tri[i] = new Point3d();
         tri[i].sub (myPnts[i], myPnts[i+3]);
         tri[i].scale (t);
         
         tri[i].add (myPnts[i+3]);
         
         if (X != null) 
            tri[i].transform (X);
      }
      
      return tri;
   }
   
   /** Get the instanteous normal of a triangle.
    * 
    * @param t 
    * Fraction of the path, ranging from 0 to 1. For example, 0.5 returns the
    * normal of the triangle when the triangle is half-way along its traversed
    * path.
    */
   public Vector3d computeInstanteousNormal(double t) {
      Point3d[] instTri = computeInstanteousTriangle (t);
      return MathUtil.getNormal (instTri[0], instTri[1], instTri[2]);
   }
   

   
   
   
   
   
   
   
   
   
   /* --- Functions for Continuous Collision Response --- */
   
   public void saveCurrentPositions() {
      myPnts1Backup = new Point3d[myPnts.length/2];
      
      myPnts1Backup[A1] = new Point3d( myPnts[A1] );
      myPnts1Backup[B1] = new Point3d( myPnts[B1] );
      myPnts1Backup[C1] = new Point3d( myPnts[C1] );
   }
   
   public void loadCurrentPositions() {
      myPnts[A1].set( myPnts1Backup[A1] );
      myPnts[B1].set( myPnts1Backup[B1] );
      myPnts[C1].set( myPnts1Backup[C1] );
   }
   
   public void copyPrevious2CurrentPositions() {
      myPnts[A1].set ( myPnts[A0] );
      myPnts[B1].set ( myPnts[B0] );
      myPnts[C1].set ( myPnts[C0] );
   }
   
   public Vector3d[] computeAvgVelocity(double t) {
      myAvgVels = new Vector3d[myPnts.length/2];
      
      myAvgVels[A1] = new Vector3d();
      myAvgVels[A1].sub (myPnts[A1], myPnts[A0]);
      myAvgVels[A1].scale (1/t);
      
      myAvgVels[B1] = new Vector3d();
      myAvgVels[B1].sub (myPnts[B1], myPnts[B0]);
      myAvgVels[B1].scale (1/t);
      
      myAvgVels[C1] = new Vector3d();
      myAvgVels[C1].sub (myPnts[C1], myPnts[C0]);
      myAvgVels[C1].scale (1/t);
      
      return myAvgVels;
   }
   
}

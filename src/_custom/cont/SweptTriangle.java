package _custom.cont;

import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import _custom.cont.BoundablePointArray;
import maspack.geometry.*;

public class SweptTriangle extends BoundablePointArray {

   Face myFace;
   protected RigidTransform3d X;

   public SweptTriangle (Face face, Point3d[] oldPositions) {
      super (6);
      myFace = face;
      Vertex3d[] vtxs = face.getVertices();
      if (vtxs.length != 3) {
         throw new IllegalArgumentException (
            "face is not a triangle (does not have three vertices)");
      }
      myPnts[0] = vtxs[0].getPosition();
      myPnts[1] = vtxs[1].getPosition();
      myPnts[2] = vtxs[2].getPosition();

      myPnts[3] = oldPositions[vtxs[0].getIndex()];
      myPnts[4] = oldPositions[vtxs[1].getIndex()];
      myPnts[5] = oldPositions[vtxs[2].getIndex()];
   }  

   public Point3d[] computeInstanteousTriangle(double s) {
      Point3d[] tri = new Point3d[3];
      
      for (int i = 0; i < 3; i++) {
         tri[i] = new Point3d();
         tri[i].sub (myPnts[i], myPnts[i+3]);
         tri[i].scale (s);
         
         tri[i].add (myPnts[i+3]);
         
         if (X != null) 
            tri[i].transform (X);
      }
      
      return tri;
   }
   
   public Vector3d computeInstanteousNormal(double s) {
      Point3d[] instTri = computeInstanteousTriangle (s);
      return MathUtil.getNormal (instTri[0], instTri[1], instTri[2]);
   }
}

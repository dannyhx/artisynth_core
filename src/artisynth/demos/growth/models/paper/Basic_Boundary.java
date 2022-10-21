package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/* 
 * -model artisynth.demos.growth.models.paper.Basic_Boundary
 * 
 * Adjustable parameters:
 * 
 * - isHigherStiffness
 */
public class Basic_Boundary extends Basic_Base {

   protected void build_pre () {
      super.build_pre ();

      boolean isHigherStiffness = true;

      if (isHigherStiffness) {
         m_shellThickness = 0.1;
         m_youngsModulus = 1e7;
         mPauseEveryInterval = 2.50;
         mCameraEye = new Point3d (2.35248, -1.37514, -0.0954112);
         mCameraCenter = new Point3d (0, 0, 0);
      }
      else {
         m_shellThickness = 0.0001;
         m_youngsModulus = 1e5;
         mPauseEveryInterval = 1.50;
         mCameraEye = new Point3d (1.63678, -1.90427, 1.07648);
         mCameraCenter = new Point3d (0, 0, 0);
      }

   }

   public boolean isMorphogenSrcNode (int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return MeshUtil.isBoundaryVtx (vtx);
   }

}

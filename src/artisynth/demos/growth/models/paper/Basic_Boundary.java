package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/* 
 * -model artisynth.demos.growth.models.paper.Basic_Boundary
 * 
 * High resistance camera center:
 *     2.35248 -1.37514 -0.0954112
 * 
 * Low resistance camera center:
 *     1.63678 -1.90427 1.07648
 */
public class Basic_Boundary extends Basic_Base {

   protected void build_pre () {
      super.build_pre ();

      // High Resistance
      m_shellThickness = 0.1;
      m_youngsModulus = 1e7;
      mPauseEveryInterval = 2.50;

      // Low Resistance
      // m_shellThickness = 0.0001;
      // m_youngsModulus = 1e5;
      // mPauseEveryInterval = 1.50;
   }

   public boolean isMorphogenSrcNode (int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return MeshUtil.isBoundaryVtx (vtx);
   }

}

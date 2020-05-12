package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class Basic_Boundary extends Basic_Base {
   
   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.1; 
      m_youngsModulus = 1e8;  
      
//      m_shellThickness = 0.0001; 
//      m_youngsModulus = 1e5;  
      
      // Post bug fix: Young modulus was previous capped to 1e5
      m_youngsModulus = 1e5;
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return MeshUtil.isBoundaryVtx (vtx);
   }
    
}

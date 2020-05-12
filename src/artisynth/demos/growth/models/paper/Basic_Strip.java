package artisynth.demos.growth.models.paper;

import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class Basic_Strip extends Basic_Base {

   
   protected void build_pre() {
      super.build_pre();
      
      mMeshY = 4;
      mMeshYDiv = 80;
      
//      mIsActivatePAR = false;
      
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

      return (pnt.y > -0.101 && pnt.y < 0.101);  
   }
    
}

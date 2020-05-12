package artisynth.demos.growth.models.paper;

import java.awt.Color;

import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class Basic_Center extends Basic_Base {

   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.1; 
      m_youngsModulus = 1e8;  
      
      m_shellThickness = 0.0001; 
      m_youngsModulus = 1e5;  
      
      // Post bug fix: Young modulus was previous capped to 1e5
      m_youngsModulus = 1e5;
   }
   
   // REMESH FIGURE ONLY
   protected void build_renderConfig() {
      super.build_renderConfig ();
      mRendCfgPresets.get (RenderMode.DEFAULT).mNodeRadius *= 0;
      this.setSurfaceColor (SurfaceColor.DEFAULT);
      mRendCfgPresets.get (RenderMode.DEFAULT).mFrontMeshColor = Color.orange;
      
      this.setRenderMode (RenderMode.DEFAULT);
   }

   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return (pnt.norm () < 0.201);
   }
    
}

package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.GrowNode3d;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/**
 * Notes:
 * 
 *  h0,h1 = 0.9 in ThinShellAux.java
 *  
 *  middle nodes cannot bend up and down -> have to wait for edge to bend first.
 *  
 */
public class Basic_Curl_ThinShell extends Basic_Curl {

   protected void build_pre() {
      super.build_pre();
      
      m_isMembrane = true;
      
      mEnableDiffusion = true;
      mEnableGrowth = true; 
      mEnableRemesh = false; 
      mEnablePlasticEmbedding = true; 
      mEnableCollisionHandling = false;
      
      this.mPauseEveryInterval = 999;
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.01;
   }
   

}

package artisynth.demos.growth.models.paper;

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
      
      mEnableDiffusion = false;
      mEnableGrowth = false; 
      mEnableRemesh = true; 
      mEnablePlasticEmbedding = false; 
      mEnableCollisionHandling = false;
      
      this.mPauseEveryInterval = 999;
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.01;
   }
   

}

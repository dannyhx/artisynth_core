package artisynth.demos.growth.models.ts;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.paper.Basic_Curl;

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
      mEnableRemesh = true; 
      mEnablePlasticEmbedding = true; 
      mEnableCollisionHandling = false;
            
      this.mPauseEveryInterval = 999;
      
//      mIsActivatePAR = true;
//      mIsActivatePER = true;
//      
//      
//      
//      ///
//      
      m_shellThickness = 1e-3;
      m_youngsModulus = 1e4; 
//      
//      mEnableDiffusion = false;
//      mEnableGrowth = false; 
//      mEnableRemesh = false; 
//      mEnablePlasticEmbedding = false; 
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.01;
   }
   
   protected void build_post() {
      super.build_post ();
//      mMorphogen2GrowthTensor.isBendingMorphogenHack = false;
   }

   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, 0.75);
         }
      }
   }
}

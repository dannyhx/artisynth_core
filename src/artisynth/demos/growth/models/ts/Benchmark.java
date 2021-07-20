package artisynth.demos.growth.models.ts;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.paper.Basic_Base;

/**
 * How to use:
 * 
 *      this.m_isMembrane = true|false
 *      
 * Set Artisynth command-line flags to include:-play
 * Note that GUI runs in separate thread; doesn't impact solver performance.
 * 
 * As set FemModel3d.profileStressAndStiffness to true;
 * 
 * Benchmark will run for 1 seconds. See the "Time Elapsed" in the console
 * when run finishes.
 * 
 * 1012 elements (22x23):
 *      Solid-shell: 
 *      Thin-shell: 
 *      
 * 5000 elements (50x50):
 *      Solid-shell:
 *      Thin-shell:  
 *      
 * 20000 elements (100x100):
 *      
 */
public class Benchmark extends Basic_Base {
   
   protected void build_pre() {
      super.build_pre();
      
      mEnableRemesh = false;
      mEnableCollisionHandling = false;
      
      m_shellThickness = 1e-2; 
      m_youngsModulus = 1e6;  
      
      mMeshXDiv = 100;
      mMeshYDiv = 100;
      
      mDiffusionTimestepScale *= 0.1;
      
      mShowColorBar = false;
      mPauseEveryInterval = 1;
      
      //
      
      this.m_isMembrane = false;
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      mRendCfg.mNodeRadius = 0.01;
   }
   
   protected void build_post() {
      super.build_post ();
      mMorphogen2GrowthTensor.isBendingMorphogenHack = false;
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      super.advanceCustom (t0, t1, flags);
      System.out.printf ("Number of elements: %d\n", mFemModel[0].numShellElements ());;
   }
   
   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, (this.m_isMembrane) ? 0.1 : 1);
         }
      }
   }
}

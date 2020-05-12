package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/**
 * Automatically stops at 4.97 seconds.
 */
public class Basic_Base extends GrowDemo {

   protected double morphogenSrcDuration = 1;
   
   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.1; 
      m_youngsModulus = 1e8;  
      
      mSizeMin = 0.01;    // 0.1
      mSizeMax = mSizeMin*10;
      
      mMeshXDiv = 20;
      mMeshYDiv = 20;
      
      mDiffusionTimestepScale *= 0.1;
      
      mRenderMode = RenderMode.MORPHOLOGY;
      mSurfaceColor = SurfaceColor.MORPHOGEN;
      mPauseEveryInterval = 4.97;
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      mRendCfg.mNodeRadius = 0.01;
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      advanceCustom_applyMorphogen (t0);
      super.advanceCustom (t0, t1, flags);
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return (pnt.norm () < 0.101);
   }
   
   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, 1);
         }
      }
   }
   
}

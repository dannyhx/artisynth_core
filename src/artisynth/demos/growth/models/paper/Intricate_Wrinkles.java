package artisynth.demos.growth.models.paper;

import artisynth.core.driver.Main;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.PolarityElementAux;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/*
 * -model artisynth.demos.growth.models.paper.Intricate_Wrinkles
 * 
 * Zoom-in of mesh corner: 
 * 
 * Eye: -0.226606 -0.261661 0.999019 
 * 
 * Center: -0.226606 -0.261661 1.91022e-18 
 * 
 * Latex Crop: 732 134 688 166
 * 
 */
public class Intricate_Wrinkles extends GrowDemo {

   protected double morphogenSrcDuration = 2;

   protected void build_pre () {
      super.build_pre ();

      mSizeMin = 0.01;
      mSizeMax = mSizeMin * 10;
      mMeshXDiv = 10;
      mMeshYDiv = 10;
      m_shellThickness = 0.1 / (1e3);
      m_youngsModulus = 100000.0 / (1e3); // 5e5
      mRemeshFreq = 0.25;
      mEnableCollisionHandling = false;

      mRenderMode = RenderMode.MORPHOLOGY;
      mPauseEveryInterval = 1; // 1
      mDiffusionTimestepScale *= 0.5;
      mShowColorBar = false;

      mPauseEveryInterval = 3.00;
   }

   protected void build_renderConfig () {
      super.build_renderConfig ();

      mRendCfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      mRendCfg.mDrawEdges = false;
      mRendCfg.mNodeRadius = 0;

      // mRendCfg = mRendCfgPresets.get (RenderMode.TOPOLOGY);
      // mRendCfg.mDrawEdges = true;
      // mRendCfg.mNodeRadius = 0;
      // mCameraEye = new Point3d (-0.226606, -0.261661, 0.999019);
      // mCameraCenter = new Point3d (-0.226606, -0.261661, 1.91022e-18);
   }

   protected void build_post () {
      super.build_post ();
      PolarityElementAux.createPolGradientAgainstAxis (mFemModel[0], 1, +1);
   }

   public void advanceCustom (double t0, double t1, int flags) {
      if (t0 < 0.01) {
         Main.getMain ().getViewer ().setEye (new Point3d (0, 0, 2.01888));
         Main.getMain ().getViewer ().setCenter (new Point3d (0, 0, 0));
      }

      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if (isMorphogenSrcNode (v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, 1.5);
         }
      }
      super.advanceCustom (t0, t1, flags);
   }

   public boolean isMorphogenSrcNode (int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return (pnt.norm () < 0.201);
   }
}

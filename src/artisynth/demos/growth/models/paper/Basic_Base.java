package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.PolarityElementAux;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/*
 * -model artisynth.demos.growth.models.paper.Basic_Base
 * 
 * Adjustable parameters:
 * 
 * - isHigherStiffness
 */
public class Basic_Base extends GrowDemo {

   protected double morphogenSrcDuration = 1;

   protected void build_pre () {
      super.build_pre ();

      boolean isHigherStiffness = false;

      mSizeMin = 0.01;
      mSizeMax = mSizeMin * 10;

      mMeshXDiv = 20;
      mMeshYDiv = 20;

      mDiffusionTimestepScale *= 0.1;

      mRenderMode = RenderMode.MORPHOLOGY;
      mSurfaceColor = SurfaceColor.MORPHOGEN;
      mPauseEveryInterval = 4.97;
      mShowColorBar = false;
      mEnableCollisionHandling = false;

      if (isHigherStiffness) {
         m_shellThickness = 0.1;
         m_youngsModulus = 1e7;
         mPauseEveryInterval = 4.00;
         mCameraEye = new Point3d (1.96632, 1.86592, 0.340617);
         mCameraCenter = new Point3d (0, 0, 0);
      }
      else {
         m_shellThickness = 0.0001;
         m_youngsModulus = 1e5;
         mPauseEveryInterval = 3.00;
         mCameraEye = new Point3d (1.63678, -1.90427, 1.07648);
         mCameraCenter = new Point3d (0, 0, 0);
      }
   }

   protected void build_renderConfig () {
      super.build_renderConfig ();

      mRendCfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      mRendCfg.mNodeRadius = 0.00;
   }

   protected void build_post () {
      super.build_post ();
      PolarityElementAux.createPolGradientAgainstAxis (mFemModel[0], 1, +1);
   }

   public void advanceCustom (double t0, double t1, int flags) {
      advanceCustom_applyMorphogen (t0);
      super.advanceCustom (t0, t1, flags);
   }

   public boolean isMorphogenSrcNode (int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return (pnt.norm () < 0.101);
   }

   public void advanceCustom_applyMorphogen (double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if (isMorphogenSrcNode (v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, 2);
         }
      }
   }

}

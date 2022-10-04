package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.PolarityElementAux;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/* -model artisynth.demos.growth.models.paper.Intricate_RippleCascade
 * 
 * Grow up to 20 seconds in 6.66 intervals.
 */
public class Intricate_RippleCascade extends GrowDemo {

   protected double morphogenSrcDuration = 999;

   protected void build_pre () {
      super.build_pre ();

      mMeshX = 2;
      mMeshY = 1;
      mMeshXDiv = 16;
      mMeshYDiv = 8;

      mRemeshFreq = 0.25;
      mSizeMin = 0.01;
      mSizeMax = mSizeMin * 100;

      m_shellThickness = 0.001;
      m_youngsModulus = 1e4;

      mDiffusionTimestepScale = 0.00001;
      mIsActivatePAR = false;
      mIsActivatePER = true;

      mShowColorBar = false;

      // Collision test
      mEnableCollisionHandling = false;
      mEnableProximityDetection = true;
      mEnableContinuousDetection = false;
      mEnableImpactZoneDetection = true;
      mPenetrationTol = -1e-3;

      mPauseEveryInterval = 18;

      // Plastic Embed figure - All
      // mPauseEveryInterval = 13.5;
      // mEnablePlasticEmbedding = true;
      // mEnableRemesh = true;

      // Plastic Embed figure - Remesh
      // mPauseEveryInterval = 13.5;
      // mEnablePlasticEmbedding = false;
      // mEnableRemesh = true;

      // Plastic Embed figure - Plastic
      // mPauseEveryInterval = 13.5;
      // mEnablePlasticEmbedding = true;
      // mEnableRemesh = false;

      // Plastic Embed figure - None
      // mPauseEveryInterval = 7.00;
      // mEnablePlasticEmbedding = false;
      // mEnableRemesh = false;
   }

   protected void build_renderConfig () {
      super.build_renderConfig ();

      RenderConfig cfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      cfg.mFrontMeshColor = Color.RED;
      cfg.mBackgroundColor = Color.WHITE;

      // Front-View
      // mCameraEye = new Point3d (0.00221373, -3.18407, 0.00573919);
      // mCameraCenter = new Point3d (0.00221373, -0.0263626, 0.00573919);

      // Perspective
      // mCameraEye = new Point3d (1.981, -1.84654, 0.301408);
      // mCameraCenter = new Point3d (0.302009, -0.301062, -0.0870515);

      // Close-up
      mCameraEye = new Point3d (0.214431, -0.687118, 0.08121);
      mCameraCenter = new Point3d (-0.0160858, -0.521149, 0.0225617);

      // Top-Down for ablation topology
      // mCameraEye = new Point3d (-0.0103429, -0.00136385, 2.64955);
      // mCameraCenter = new Point3d (-0.0103429, -0.00136385, -0.00338145);
      cfg = mRendCfgPresets.get (RenderMode.TOPOLOGY);
      // cfg.mFrontMeshColor = new Color (204, 204, 204);
      // cfg.mRearMeshColor = new Color (204, 204, 240);
      // cfg.mBackgroundColor = Color.WHITE;
      // cfg.mMeshEdgeColor = Color.BLACK;

      // Ablation rest figure color
      // cfg.mFrontMeshColor = cfg.mRearMeshColor;

      mRendCfg = cfg;
   }

   protected void build_post () {
      super.build_post ();

      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if (isMorphogenSrcNode (v)) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mIsMorphogenSrc = true;
         }
      }

      PolarityElementAux.createPolGradientAgainstAxis (mFemModel[0], 1, -1);
   }

   public void advanceCustom (double t0, double t1, int flags) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
         if (gNode.mIsMorphogenSrc) {
            gNode.mChems.set (3, 1.0);
         }
      }

      super.advanceCustom (t0, t1, flags);
   }

   public boolean isMorphogenSrcNode (int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.pnt;

      return (pnt.y < -0.499);
   }
}

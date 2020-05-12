package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.core.driver.Main;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/**
 * Grow up to 20 seconds in 6.66 intervals.
 * 
 * Camera:
 *   Front view:
 *     0.00221373 -3.18407 0.00573919
 *     0.00221373 -0.0263626 0.00573919
 *     
 *   Front view (alt for video): 
 *     
 *     
      mCameraEye = new Point3d(0, -3.18586, 0);
      mCameraCenter = new Point3d(0, 0, 0);
 *    
 *   Perspective:
 *     1.981 -1.84654 0.301408
 *     0.302009 -0.301062 -0.0870515
 *  
 *   Close-up:
      mCameraEye = new Point3d(0.119382, -0.985104, 0.098211);
      mCameraCenter = new Point3d(-0.138341, -0.747875, 0.038583);
 */
// artisynth.models.plants.growth.paperdemos.Intricate_RippleCascade
public class Intricate_RippleCascade extends GrowDemo {
   
   protected double morphogenSrcDuration = 999;
   
   protected void build_pre() {
      super.build_pre();
      
      mMeshX = 2;
      mMeshY = 1;
      
      mSizeMin = 0.01;
      mSizeMax = mSizeMin*100;
      mMeshXDiv = 16;
      mMeshYDiv = 8;
      m_shellThickness = 0.001;
      m_youngsModulus = 1e6;  
      
      mDiffusionTimestepScale = 0.00001;
      
      mIsActivatePAR = true;
      mIsActivatePER = true;
      
      // Benchmark:
      mRenderMode = RenderMode.MORPHOLOGY;
      mPauseEveryInterval = 20;
      mEnableCollisionHandling = false;
      mRemeshFreq = 0.25;
      
      // Post bug fix: Young modulus was previous capped to 1e5
      m_youngsModulus = 1e5;
      
      mShowColorBar = false;
      mCameraEye = new Point3d(0, -3.18586, 0);
      mCameraCenter = new Point3d(0, 0, 0);
      
      // Collision test
      mPauseEveryInterval = 20;
      mEnableCollisionHandling = false;
      mEnableProximityDetection = true;
      mEnableContinuousDetection = false;
      mEnableImpactZoneDetection = true;
      mPenetrationTol = -1e-3;
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      RenderConfig cfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      cfg.mNodeRadius = 0.01;
      
      cfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      cfg.mFrontMeshColor = Color.RED;
      cfg.mBackgroundColor = Color.white;
      
//      cfg = mRendCfgPresets.get (RenderMode.TOPOLOGY);
//      cfg.mFrontMeshColor = new Color(250,250,250);
      
//      mRendCfg = mRendCfgPresets.get (RenderMode.TOPOLOGY);
//      mRendCfg.mFrontMeshColor = new Color(242,242, 242);
//      mRendCfg.mDrawEdges = true;
   }
   
   protected void build_post() {
      super.build_post ();
      
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) ) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mIsMorphogenSrc = true;
         }
      }
   }
   
   public void advanceCustom(double t0, double t1, int flags) {       
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
         if (gNode.mIsMorphogenSrc) {
            gNode.mChems.set (3, 1);
         }
      }
      
      super.advanceCustom (t0, t1, flags);
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.pnt;
      
      return (pnt.y < -0.499);
   }
}

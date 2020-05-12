package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.demos.growth.GrowNode3d;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/*
1.25093 -1.1616 1.03499
0.0225034 -0.0624607 -0.0503307
 */

public class Basic_Curl extends Basic_Base {
   protected void build_pre() {
      super.build_pre();
      
//      mIsActivatePAR = false;
      
      m_shellThickness = 1e-2;
      m_youngsModulus = 1e5; 
      
      // Paper
//      m_shellThickness = 1e-4;
//      m_youngsModulus = 1e8;
      
//      m_shellThickness = 0.0001; 
//      m_youngsModulus = 1e5;  
      
      mEnableRemesh = true;
      mSizeMin = 0.05;
      mSizeMax = mSizeMax * 100;
      
      mEnableCollisionHandling = true;
      
      mShowColorBar = false;
      mPauseEveryInterval = 15;
      
      mIsActivatePAR = true;
      mIsActivatePER = false;
      
      mCameraEye = new Point3d(1.25093, -1.1616, 1.03499);
      mCameraCenter = new Point3d(0.0225034, -0.0624607, -0.0503307);
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mSurfaceColor = SurfaceColor.MORPHOGEN;
      mRendCfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
//      mRendCfg.mNodeRadius = 0;
      mFemModel[0].setDirectorRenderLen (100);
      mRendCfg.mNodeRadius = 0.00;
   }
   
   protected void build_post() {
      super.build_post ();
      mMorphogen2GrowthTensor.isBendingMorphogenHack = true;
      
      mGrowColorer.zeroMorphogenRGB = new Vector3d(250., 250., 255.);
      mGrowColorer.zeroMorphogenRGB.scale (1.0/255);
      
      mGrowColorer.maxMorphogenRGB = new Vector3d(199., 199., 255.);
      mGrowColorer.maxMorphogenRGB.scale (1.0/255);
      
      // Clamp
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
         if (gNode.getPosition ().y < -0.49) {
            gNode.setDynamic (false);
         }
      }
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      super.advanceCustom (t0, t1, flags);
      if (t0 < 10) {
//         mMorphogen2GrowthTensor.applyGrowthTensorsCustom ();
      }
   }

   
   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, 0.4);
         }
      }
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();

      return (pnt.y > 0);
   }
}

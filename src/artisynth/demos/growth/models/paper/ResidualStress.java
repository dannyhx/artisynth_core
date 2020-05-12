package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.core.driver.Main;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;

// artisynth.models.plants.growth.paperdemos.ResidualStress
public class ResidualStress extends GrowDemo {
   
   protected double mMorphogenSrcConc = 1;
   
   protected void build_pre() {
      super.build_pre ();
      
      m_shellThickness = 0.1;
      m_youngsModulus = 1e8;
      
      mMeshX = 0.5;
      mMeshY = 0.5;
      mMeshXDiv = 2;
      mMeshYDiv = 5;
      
      mSizeMin = 0.25;  
      mSizeMax = mSizeMin*100;
      
      mEnableCollisionHandling = false;
      mEnableRemesh = true;
      
      // Post bug fix: Young modulus was previous capped to 1e5
      m_youngsModulus = 1e5;
      
      mPauseEveryInterval = 10;
      
      mSepScaleForCutNodes = 0.20;
   }
   
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[M];
      mMesh[0] = MeshFactory.createOpenCylinder (1, 5, 16, 16);
      
      RigidTransform3d X = new RigidTransform3d();
      X.mulRotY (Math.toRadians (90));
      mMesh[0].transform (X);
   }
   
   protected void build_renderConfig() {   
      super.build_renderConfig ();
      
      mRendCfg.mNodeRadius = 0.02;
      
      RenderConfig topolCfg = mRendCfgPresets.get (RenderMode.TOPOLOGY);
      topolCfg.mFrontMeshColor = new Color(51, 255, 204);
      
      mSurfaceColor = SurfaceColor.PLASTIC_STRAIN;
   }
   
   protected void build_post() {      
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) ) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
//            gNode.mIsMorphogenSrc = true;
         }
      }

      mFemModel[0].setDirectorRenderLen (0);
      
      super.build_post ();
   }

   public void advanceCustom(double t0, double t1, int flags) {
      
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
         if (gNode.mIsMorphogenSrc) {
            gNode.mChems.set (3, mMorphogenSrcConc);
         }
      }
      
      super.advanceCustom (t0, t1, flags);
   }
   
   public void customLogic() {

   }
   

   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = mFemModel[0].getNode (v).getPosition ();
      
      return (pnt.x > -0.3126 && pnt.x < 0.3126);
   }
   
   
}

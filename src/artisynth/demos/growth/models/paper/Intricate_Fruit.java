package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.models.base.ShellPatch.RenderConfig;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;

/* Top-Camera:

      mCameraEye = new Point3d(0, -4.9771, 0);
      mCameraCenter = new Point3d(0,0,0);
 */
// artisynth.models.plants.growth.paperdemos.Intricate_Fruit
public class Intricate_Fruit extends GrowDemo {

   protected void build_modelSkeleton() {
    RigidTransform3d tns = new RigidTransform3d();
    tns.mulRotX (45);
      
    mMesh = new PolygonalMesh[M];
    mMesh[0] = MeshFactory.createIcosahedralSphere (0.5, 4);
//    mMesh.transform ( tns );
//    for (Vertex3d vtx : mMesh.getVertices ()) {
//       vtx.setPosition (new Point3d(
//          vtx.pnt.x, vtx.pnt.y, vtx.pnt.z*0.1
//       ));
//    }
 }
   
   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.1;
      m_youngsModulus = 1e6;
      
      mSizeMin = 0.05;    // 0.05
      mSizeMax = mSizeMin*100;
      mRefineCompression = 0.01;           // Bigger wavelength 
      
      // Benchmark:
      mRemeshFreq = 0.25;
      mPauseEveryInterval = 20;
      mEnableCollisionHandling = false;
      mRenderMode = RenderMode.MORPHOLOGY;
      
      // Post bug fix: Young modulus was previous capped to 1e5
      m_youngsModulus = 1e5;
      mPauseEveryInterval = 20; // 6.66
      mShowColorBar = false;
      
      // Perspective
//      mCameraEye = new Point3d(1.70087, -3.15077, -3.87599);
//      mCameraCenter = new Point3d(0,0,0);
      
      // Top-Down 
      mCameraEye = new Point3d(0, -4.9771, 0);
      mCameraCenter = new Point3d(0,0,0);
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      RenderConfig cfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      cfg.mNodeRadius = 0.01;
      
      cfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      cfg.mFrontMeshColor = new Color(255,102,0);
      cfg.mBackgroundColor = Color.white;
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
//      mMechModel.setCollisionBehavior (mFemModel, mFemModel, true);
      
//      mRemesher.isDebug = true;

//      mRemeshFreq = 0.01;
      
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) ) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, 0.5);
         }
      }
//      mMorphogen2GrowthTensor.unapplyGrowthTensors ();
      
      super.advanceCustom (t0, t1, flags);
   }
   
   public boolean isMorphogenSrcNode(int v) {
//      Vertex3d vtx = mMesh.getVertex (v);
//      Point3d pnt = vtx.getPosition ();
//      
//      return (pnt.y > -0.01 && pnt.y < +0.01);
      return true;
   }
    
}

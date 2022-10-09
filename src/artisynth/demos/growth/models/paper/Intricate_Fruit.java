package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

/* 
 * -model artisynth.demos.growth.models.paper.Intricate_Fruit
 */
public class Intricate_Fruit extends GrowDemo {

   protected void build_modelSkeleton () {
      RigidTransform3d tns = new RigidTransform3d ();
      tns.mulRotX (45);

      mMesh = new PolygonalMesh[M];
      mMesh[0] = MeshFactory.createIcosahedralSphere (0.5, 4);
      // mMesh.transform ( tns );
      // for (Vertex3d vtx : mMesh.getVertices ()) {
      // vtx.setPosition (new Point3d(
      // vtx.pnt.x, vtx.pnt.y, vtx.pnt.z*0.1
      // ));
      // }
   }

   protected void build_pre () {
      super.build_pre ();

      m_shellThickness = 0.1;
      m_youngsModulus = 1e6;

      mSizeMin = 0.05; // 0.05
      mSizeMax = mSizeMin * 100;
      mRefineCompression = 0.01; // Bigger wavelength

      // Benchmark:
      mRemeshFreq = 0.25;
      mPauseEveryInterval = 25;
      mEnableCollisionHandling = false;
      mRenderMode = RenderMode.MORPHOLOGY;

      // Post bug fix: Young modulus was previous capped to 1e5
      // m_youngsModulus = 1e5;
      mPauseEveryInterval = 20; // 6.66
      mShowColorBar = false;

      // Perspective
      mCameraEye = new Point3d (1.70087, -3.15077, -3.87599);
      mCameraCenter = new Point3d (0, 0, 0);

      // Top-Down
      // mCameraEye = new Point3d (0, -4.9771, 0);
      // mCameraCenter = new Point3d (0, 0, 0);

      this.mIsActivatePAR = true;
      this.mIsActivatePER = true;
      this.mIsActivateNOR = false;
   }

   protected void build_renderConfig () {
      super.build_renderConfig ();

      RenderConfig cfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      cfg.mNodeRadius = 0.01;

      cfg = mRendCfgPresets.get (RenderMode.MORPHOLOGY);
      cfg.mFrontMeshColor = new Color (255, 102, 0);
      cfg.mBackgroundColor = Color.WHITE;
   }

   protected void build_post () {
      super.build_post ();

      // PolarityElementAux.createPolGradientAgainstAxis (mFemModel[0], 1, +1);
      this.mFixedParDir = new Vector3d (0, 1, 0);
      this.mFixedPerDir = null;
   }

   public void advanceCustom (double t0, double t1, int flags) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if (isMorphogenSrcNode (v)) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (3, 0.5);
         }
      }
      super.advanceCustom (t0, t1, flags);
   }

   public boolean isMorphogenSrcNode (int v) {
      return true;
   }

}

package artisynth.demos.growth.models.paper;

import artisynth.core.femmodels.FemNode3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

// TODO
// Uncomment ContinuousCollider.mySpaceElipson = 1e-12
// -model artisynth.models.plants.growth.paperdemos.Intricate_Cylinder
public class Intricate_Cylinder extends GrowDemo {

   protected enum Type {
      LOWRES_TOP_CYLINDER,
      HIGHRES_TOP_CYLINDER,
      PILLOW,
      BUD
   }
   
   protected Type mType = Type.PILLOW;
   

   
   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.001; // 0.01
      m_youngsModulus = 1e4;  // 1e7
      
      // EXP
      mDiffusionTimestepScale = 0.002;
      mRefMeshOffset = new Vector3d(5,0,0);
      
      mSizeMin = 
         (mType == Type.PILLOW) ?
            0.1 :
         (mType == Type.HIGHRES_TOP_CYLINDER) ?
            0.1 :
         (mType == Type.LOWRES_TOP_CYLINDER) ?
            0.2 :  // 0.2
         (mType == Type.BUD) ?
            0.1 : 
         0.02;
      
      mSizeMax = mSizeMin*100;
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      RenderConfig defaultCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      defaultCfg.mNodeRadius = 0.01;
   }
   
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[M];
      
      mMesh[0] = 
         (mType == Type.PILLOW) ?
            MeshFactory.createOpenCylinder (0.5, 0.5, 36, 8) :
         (mType == Type.HIGHRES_TOP_CYLINDER) ?  
            MeshFactory.createOpenCylinder (0.5, 0.5, 36, 8) :
         (mType == Type.LOWRES_TOP_CYLINDER) ?
            MeshFactory.createOpenCylinder (0.5, 0.5, 16, 3) :
         (mType == Type.BUD) ?
            MeshFactory.createIcosahedralSphere (0.5, 4) :
            null;
   }
   
   protected void build_post() {

      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) ) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mIsMorphogenSrc = true;
         }
      }
 
//      ContinuousCollider.myTimeElipson = 1e-4;
//      ContinuousCollider.mySpaceElipson = 1e-12;
      
      // Freeze the nodes first
      for (FemNode3d node : mFemModel[0].getNodes ()) {
         node.setDynamic (false);
         node.getBackNode ().setDynamic (false);
      }
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      // Need to delay simulating the physics until remesher
      // makes the mesh symmetrical.
      double warmupEnd = 1;
      if (t0 < 1.1) {
         mEnableDiffusion = (t0 > warmupEnd);
         mEnableGrowth = (t0 > warmupEnd);
         mEnableRemesh = true;
         mEnablePlasticEmbedding = (t0 > warmupEnd);
         mEnableCollisionHandling = (t0 > warmupEnd);
      }
      
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
         if (gNode.mIsMorphogenSrc)
            gNode.mChems.set (3, 0.5);
      }

      super.advanceCustom (t0, t1, flags);
      
      // Unfreeze the nodes; allow physics simulation
      for (FemNode3d node : mFemModel[0].getNodes ()) {
         node.setDynamic (t0 > 0.05);
      }
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = mFemModel[0].getNode (v).getPosition ();
      
      return 
         (mType == Type.LOWRES_TOP_CYLINDER || mType == Type.HIGHRES_TOP_CYLINDER) ? 
            (MeshUtil.isBoundaryVtx (vtx)) :
         (mType == Type.PILLOW) ? 
            true :
         (mType == Type.BUD) ? 
            (pnt.z > 0.45) :
         false;
   }
    
}

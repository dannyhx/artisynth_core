package artisynth.demos.growth.models.paper;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo;
import maspack.geometry.MeshFactory;
import maspack.matrix.Matrix3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

// -model artisynth.demos.growth.models.paper._Debug_Growth

public class _Debug_Growth extends GrowDemo {
   
   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 1e-2;
      m_youngsModulus = 1e4;
      m_density = 100;

      mMeshX = 1;
      mMeshY = 1;
      mMeshXDiv = 10;
      mMeshYDiv = 10;
      
      mEleClass = ElementClass.SHELL;
      
      this.mIsShowRefMesh = true;
   }
   
   protected void build_post() {
      mIsShowRefMesh = true;
      mEnableDiffusion = false;
      mEnablePlasticEmbedding = true;
      mEnableRemesh = false;
      mEnableCollisionHandling = false;
      
      super.build_post ();
      
//      this.mMorphogen2GrowthTensor.fixedBendingStrain = new Matrix3d(
//         .25, 0, 0, 
//         0, .25, 0,
//         0, 0, 0 
//      );
   }
   
   public void advanceCustom(double t0, double t1, int flags) {
      super.advanceCustom (t0, t1, flags);
      
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
         gNode.mChems.set (0, 0.10);
      }
   }

   // Overridden Methods
   
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      mRendCfg.mDirectorLen = 1;
      mRendCfg.mNodeRadius = 0.01;
   }


   protected void build_modelSkeleton() {
      super.build_modelSkeleton ();
//      mMesh[0] = MeshUtil.createUniformDisc (10, 20);
      
      // Box
//      mMesh[0] = MeshFactory.createBox (1, 1, 1);
//      RigidTransform3d X = new RigidTransform3d();
//      double rotAng = 1*45 % 360;
//      X.setIdentity ();
//      X.mulRotZ (Math.toRadians (rotAng));
//      mMesh[0].transform(X);
//      mMesh[0].translate (new Vector3d(1,0,0));
      
//      mMesh[0] = MeshUtil.createCylinderFromPlane_YAxisCurved (
//       mMeshX, mMeshY, mMeshXDiv, mMeshYDiv, 1);
      
//    RigidTransform3d X = new RigidTransform3d();
//    double rotAng = 1*90 % 360;
//    X.setIdentity ();
//    X.mulRotX (Math.toRadians (rotAng));
//    mMesh[0].transform(X);

//      mMesh[0] = MeshFactory.createIcosahedralSphere (0.5, 4);
   }
   
   
   protected void build_modelStructure() {
      super.build_modelStructure ();
        
      for (ShellElement3d ele : mFemModel[0].getShellElements ()) {
         FemNode3d[] nodes = ele.getNodes ();
         System.out.printf ("Ele: %s: [%d,%d,%d]\n", 
            ele.getName (), 
            nodes[0].getIndex (),
            nodes[1].getIndex (),
            nodes[2].getIndex ()
         );
      }
      
//      PolygonalMesh restMeshFront =  MeshUtil.createCylinderFromPlane_YAxisCurved (
//          mMeshX, mMeshY, mMeshXDiv, mMeshYDiv, 1);
////      mMesh[0] = restMeshFront;
//       
//      double c1 = mMeshY;
//      double r1 =  c1 / (2*PI);
//      double c2 = 2*PI * (r1 + m_shellThickness);
//       
//      PolygonalMesh restMeshBack =  MeshUtil.createCylinderFromPlane_YAxisCurved (
//          mMeshX, c2, mMeshXDiv, mMeshYDiv, 1);
//       
//      ArrayList<Vertex3d> restFrontVtxs = restMeshFront.getVertices ();
//      ArrayList<Vertex3d> restBackVtxs = restMeshBack.getVertices ();
//       
//      // Create a node for each mesh vertex
//       
//      for (int v = 0; v < restFrontVtxs.size(); v++) {
//         FemNode3d node = null;
//          
//         node = mFemModel[0].getNode (v);
//         node.setPosition(restFrontVtxs.get (v).pnt);
//         node.setRestPosition (restFrontVtxs.get (v).pnt);
//          
//         // Backnode
//         if (this.mEleClass == ElementClass.VOLUMETRIC) {
//            node = mFemModel[0].getNode (restFrontVtxs.size() + v);
//            node.setPosition(restBackVtxs.get (v).pnt);
//            node.setRestPosition (restBackVtxs.get (v).pnt);
//         } else {
//            node.setBackPosition(restBackVtxs.get (v).pnt);
//            node.setBackRestPosition (restBackVtxs.get (v).pnt);
//         }
//      }
   }
}

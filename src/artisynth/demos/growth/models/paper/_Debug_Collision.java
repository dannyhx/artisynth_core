package artisynth.demos.growth.models.paper;

import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import java.awt.Color;

import artisynth.core.femmodels.FemNode3d;
import artisynth.demos.growth.collision.CollisionDetector;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

// artisynth.models.plants.growth.paperdemos._GrowDemoCollisionTest
public class _Debug_Collision extends GrowDemo {
   
   protected void build_pre() {
      super.build_pre();
      
      mIsShowRefMesh = false;
      
      m_shellThickness = 0.001;
      m_youngsModulus = 1e5; 
      
      mPenetrationTol = -5e-2;
      mEnableContinuousDetection = true;
      mEnableCollisionHandling = true;
      
      mSizeMin = 0.20;
      mSizeMax = 100*mSizeMin;
      
      mEnableDiffusion = false;
      mEnableGrowth = false;
      mEnableRemesh = false;
      mEnablePlasticEmbedding = false;
      mEnableSelfCollision = true;
      
      // DEBUG
      mEnableRemesh = true;
      mSizeMin = 0.15;
      mSizeMax = mSizeMin*100;
      mRemeshFreq = 0.25;
      mEnablePlasticEmbedding = true;
      
      // Bilateral test
      mEnablePlasticEmbedding = false;
      mEnableRemesh = false;
      mEnableSelfCollision = true;
      
      // Impact zone test
      mEnableProximityDetection = true;
      mEnableContinuousDetection = false;
      mEnableImpactZoneDetection = true;
     
      mMaxContinuousIters = 5;
      CollisionDetector.myImpactZonePenetrationTol = -0.001;
      
      this.mGravity = new Vector3d(0,0,-9.81);
      
//      mPauseEveryInterval = 1.17;
      
      mCameraEye = new Point3d(0.837961, -5.56226, 1.28312);
      mCameraCenter = new Point3d(-0.108776, 0.12338, 0.174606);  
   }
   
   
   protected Face equilateralTriangle(double sideLen) {
      // Left
      Vertex3d vtx0 = new Vertex3d(0,0,0);
      
      // Right
      Vertex3d vtx1 = new Vertex3d(sideLen,0,0);
      
      // Top
      Vertex3d vtx2 = new Vertex3d(sideLen/2,sin (toRadians (60)) * sideLen, 0);
      
      Point3d centroid = new Point3d();
      centroid.add (vtx0.pnt);
      centroid.add (vtx1.pnt);
      centroid.add (vtx2.pnt);
      centroid.scale (1/3.0);
      
      Vector3d cent2origin = new Vector3d().sub(centroid);
      vtx0.pnt.add (cent2origin);
      vtx1.pnt.add (cent2origin);
      vtx2.pnt.add (cent2origin);

      return MeshUtil.createFace (vtx0, vtx1, vtx2, Vector3d.Z_UNIT);
   }
   
   protected void build_modelSkeleton() {
//      mEnableGrowth = true;
//      mEnableDiffusion = true;
//      mPauseEveryInterval = 1;
      
//       M = 1;
//       mMesh = new PolygonalMesh[M];
//       mMesh[0] = new PolygonalMesh();
//       
//       mMesh[0].addVertex (new Vertex3d(0,0,0));
//       mMesh[0].addVertex (new Vertex3d(1,0,0));
//       mMesh[0].addVertex (new Vertex3d(0,1,0));
//       mMesh[0].addFace (new int[] {0,1,2});
//       
//       mMesh[0].addVertex (new Vertex3d(0,0,1));
//       mMesh[0].addVertex (new Vertex3d(1,0,1));
//       mMesh[0].addVertex (new Vertex3d(0,1,1));
//       mMesh[0].addFace (new int[] {3,4,5});
       
       ///
       
//       M = 2;
//       mMesh = new PolygonalMesh[M];
//       mMesh[0] = new PolygonalMesh();
//       mMesh[1] = new PolygonalMesh();
//       
//       mMesh[0].addVertex (new Vertex3d(0,0,0));
//       mMesh[0].addVertex (new Vertex3d(1,0,0));
//       mMesh[0].addVertex (new Vertex3d(0,1,0));
//       mMesh[0].addFace (new int[] {0,1,2});
//       
//       mMesh[1].addVertex (new Vertex3d(0,0,0.2));
//       mMesh[1].addVertex (new Vertex3d(1,0,0.2));
//       mMesh[1].addVertex (new Vertex3d(0,1,0.2));
//       mMesh[1].addFace (new int[] {0,1,2});
      
       
      
      
//    M = 1;
//    mMesh = new PolygonalMesh[M];
//    mMesh[0] = new PolygonalMesh();
//    mMesh[0].addMesh (MeshFactory.createPlane (1, 1, 10, 10));
//    PolygonalMesh top = MeshFactory.createPlane (1, 1, 10, 10);
//    top.translate (new Vector3d(0,0,1));
//    mMesh[0].addMesh (top);
      
      
      // Two square meshes
      
      RigidTransform3d X = new RigidTransform3d();
      X.mulRotY (Math.toRadians (-90));
      X.addTranslation (new Vector3d(0,0,5));
      
      M = 2;
      mMesh = new PolygonalMesh[M];
      mMesh[0] = MeshFactory.createPlane (2, 2, 10, 10);
      mMesh[1] = MeshFactory.createPlane (6, 1, 30, 5);

      mMesh[1].transform (X);
      

      

      // Triangle colliding with another
      
//      M = 2;
//      mMesh = new PolygonalMesh[M];
//      
//      RigidTransform3d X = new RigidTransform3d();
//      
//      mMesh[0] = new PolygonalMesh();
//      mMesh[0].addVertex (-0.5, 0, 1);
//      mMesh[0].addVertex (+0.5, 0, 1);
//      mMesh[0].addVertex (0, 0, 0);
//      mMesh[0].addFace (new int[] {0,1,2});
//
//      mMesh[1] = new PolygonalMesh();
//      mMesh[1].addVertex (-0.5, 0, 0);
//      mMesh[1].addVertex (+0.5, 0, 0);
//      mMesh[1].addVertex (0, 0, 1);
//      mMesh[1].addFace (new int[] {0,1,2});
//      
//      X.mulRotX (Math.toRadians (-90));
//      mMesh[1].transform (X);
//      mMesh[1].translate(new Vector3d(0,-0.5,-0.25));
   }
   
   protected void build_post() {

      super.build_post ();
//      for (int n=0; n<3; n++)
//         mFemModel[0].getNode (n).setExternalForce (new Vector3d(0,0,-0.01));
      
//      for (int n=3; n<6; n++)
//         mFemModel[0].getNode (n).setExternalForce (new Vector3d(0,0,-0.01));
      
//      for (int n=0; n<3; n++)
//         mFemModel[0].getNode (n).setExternalForce (new Vector3d(0,0,+0.01));
//      for (int n=0; n<3; n++)
//         mFemModel[1].getNode (n).setExternalForce (new Vector3d(0,0,-0.01));

      // Reproduce "block out of bound" bug
//      for (int n : new int[] {20, 14}) {
//         mFemModel[0].getNode (n).setExternalForce (new Vector3d(0,0,+0.5));
//      }
      
       for (FemNode3d node :  mFemModel[0].getNodes ()) {
          if (MeshUtil.getOneRingFaces (mMesh[0].getVertex (node.getNumber ())).size () == 1) {
             node.setDynamic (false);
          }
       }
       
      mFemModel[0].getRenderProps ().setFaceColor (Color.LIGHT_GRAY);
      mFemModel[1].getRenderProps ().setFaceColor (Color.GREEN);
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      mRendCfgPresets.get (RenderMode.DEFAULT).mNodeRadius *= 0;
   }

}

package artisynth.demos.growth.models.paper;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.util.ShellUtil;
import maspack.matrix.Point3d;

public class _Debug_Diffusion_Main extends _Debug_Diffusion_Base {

   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.01;
      m_youngsModulus = 1000;
      
      mSizeMin = 0.05;   
      mSizeMax = mSizeMin*10;
      
      size = 1;  // Affects apply morphogen
      mMeshX = size;
      mMeshY = size;
      
      int div = 25;
      mMeshXDiv = div;
      mMeshYDiv = div;
      
      mAspectMin = 1;
      
//      mChemCvtRate = 0.001;// 0.001
   }
   
//   protected void build_modelSkeleton() {
////      mMesh[0] = MeshFactory.createIcosahedralSphere (0.5, 2);
////      mMesh[0] = MeshUtil.createHexagonalTriRectangle (1, 1, 18, 18, false);
//      
//      AffineTransform3d tns = new AffineTransform3d();
//      tns.addTranslation (-0.5, -0.5, 0);
//      mMesh[0].transform (tns);
//   }
   

   
   public void advanceCustom(double t0, double t1, int flags) {
      // Remesh according to strain
      if (t0 == 0) {
         // Apply morphogen on areas to remesh
         for (int v = 0; v < mMesh[0].numVertices (); v++) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            Point3d pnt = gNode.getPosition ();
            
            if (pnt.y < 0.5)
               gNode.mChems.set (3, 1);
         }
         
         // Convert morphogen to strain
         mMorphogen2GrowthTensor.activateFractionOfMorphogen (0.1, 
            mIsActivatePAR, mIsActivatePER, mIsActivateNOR);
         mMorphogen2GrowthTensor.computeIntegrationGrowthTensors ();
         mMorphogen2GrowthTensor.applyGrowthTensors (); 
         
         // Remesh 
         mRemesher.mSizingField.updateParameters (mSizeMin, mSizeMax, mAspectMin,
            mRefineAngle, mRefineCompression, mRefineVelocity);
//         mRemesher.remesh ();
         
         // Set morphogen back to zero
         for (int v = 0; v < mMesh[0].numVertices (); v++) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            Point3d pnt = gNode.getPosition ();
            gNode.mChems.set (3, 0);
         }
         
         ShellUtil.invalidateFem (mFemModel[0]);
         System.out.printf ("Remesh occurred. Eles: %d, Nodes: %d \n", 
            mFemModel[0].numShellElements (), mFemModel[0].numNodes ());
         
         mMorphogen2GrowthTensor.unapplyGrowthTensors ();
      }

      mMorphogen2GrowthTensor.activateFractionOfMorphogen (this.mChemCvtRate, 
         mIsActivatePAR, mIsActivatePER, mIsActivateNOR);
//      mMorphogen2GrowthTensor.computeGrowthTensors ();
//      mMorphogen2GrowthTensor.applyGrowthTensors (); 
//      mMorphogen2GrowthTensor.unapplyGrowthTensors ();       

      super.advanceCustom (t0, t1, flags);
   }
}

package artisynth.demos.growth.models.paper;

import java.util.Random;

import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.util.ShellUtil;
import maspack.matrix.Point3d;

public class _Debug_Diffusion_HalfRefined extends _Debug_Diffusion_Base {

   protected void build_pre() {
      super.build_pre();
      
      m_shellThickness = 0.01;
      m_youngsModulus = 1000;
      
      mSizeMin = 0.01;   
      mSizeMax = mSizeMin*50;
      
      int div = 16;
      mMeshXDiv = div;
      mMeshYDiv = div;
      
      mAspectMin = 0.1;
   }
   
   
   public void advanceCustom(double t0, double t1, int flags) {
      int seed = 0;
      Random rnd = new Random(seed);
      
      if (t0 == 0) {
         // Remeshing logic
         for (int i=0; i<5; i++) {
            mRemesher.mSizingField.updateParameters (mSizeMin, mSizeMax, mAspectMin,
               mRefineAngle, mRefineCompression, mRefineVelocity);
            
            // Apply morphogen on areas to remesh
            for (int v = 0; v < mMesh[0].numVertices (); v++) {
               GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
               Point3d pnt = gNode.getPosition ();
               
               if (rnd.nextDouble () > 0.4)
                  gNode.mChems.set (3, rnd.nextDouble ());
            }
            
            // Convert morphogen to strain
            mMorphogen2GrowthTensor.activateFractionOfMorphogen (0.0001, 
               mIsActivatePAR, mIsActivatePER, mIsActivateNOR);
            mMorphogen2GrowthTensor.computeIntegrationGrowthTensors ();
            mMorphogen2GrowthTensor.applyGrowthTensors (); 
            
            // Remesh according to strain
            mRemesher.remesh ();     
            
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
         
         // Apply actual morphogen that will diffuse
         advanceCustom_applyMorphogen (t0);
      }
      
//      mMorphogen2GrowthTensor.activateFractionOfMorphogen (this.mChemCvtRate, 
//         mIsActivatePAR, mIsActivatePER);
//      mMorphogen2GrowthTensor.computeGrowthTensors ();
//      mMorphogen2GrowthTensor.applyGrowthTensors (); 
//      mMorphogen2GrowthTensor.unapplyGrowthTensors (); 
//      
//      

      
      super.advanceCustom (t0, t1, flags);
   }
}

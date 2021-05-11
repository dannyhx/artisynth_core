package artisynth.demos.growth.models.paper;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.demos.growth.GrowColorer;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.GrowRemesher;
import artisynth.demos.growth.GrowTriElement;
import artisynth.demos.growth.Morphogen2GrowthTensor;
import artisynth.demos.growth.PlasticEmbedder;
import artisynth.demos.growth.collision.CollisionDetector;
import artisynth.demos.growth.diffusion.Diffusion;
import artisynth.demos.growth.diffusion.MeshChemicals;
import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.models.base.GrowDemo.SurfaceColor;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class _Debug_ThinShell extends GrowDemo {
   
   protected void build_pre() {
      super.build_pre();
      
      m_isMembrane = true;
      
      mEnableDiffusion = true;
      mEnableGrowth = true; 
      mEnableRemesh = false; 
      mEnablePlasticEmbedding = true; 
      
      mEnableCollisionHandling = true;
   }
   
   protected FemModel3d createFemModel() {
      return new GrowModel3d();
   }
   
   protected GrowNode3d createNode(Point3d pt) {
      return new GrowNode3d(pt, new VectorNd(mNumChemTypes), m_isMembrane);
   }
   
   protected GrowTriElement createElement(FemNode3d n0, FemNode3d n1,
   FemNode3d n2, double thickness)
   {
      return new GrowTriElement(
         (GrowNode3d)n0, 
         (GrowNode3d)n1,
         (GrowNode3d)n2, thickness, this.m_isMembrane);
   }
   
   public boolean shouldBeFrozen(int idx) {
      return false;
   }
   
   
   /** -- Engine Loop -- **/

   public void advanceCustom(double t0, double t1, int flags) {
      if (t0 < 0.01) {
         if (mCameraEye != null) {
            Main.getMain ().getViewer ().setEye (mCameraEye);
            Main.getMain ().getViewer ().setCenter (mCameraCenter);
         }
         
         Main.getMain ().getViewer ().setBackgroundColor (mRendCfg.mBackgroundColor);
      }
      
      // Pause the simulation if so.
      if (MathUtil.compare (t0 % mPauseEveryInterval, 0) == 0 && t0 != 0) {
         setStopRequest (true);
      }
      
      // Setup the collision behavior for each pair of models.
      toggleCollisionBehavior(mEnableCollisionHandling);
      
      if (mIsShowRefMesh) 
         updateReferenceMesh();
      
      if (mCustom) {
         customLogic();
         mCustom = false;
      }
      
      // Used to measure the time to simulate each component (e.g. diffusion).
      double start = 0;
      double end = 0;
      
      // Handle diffusion.
      if (mEnableDiffusion) {
         start = System.currentTimeMillis ();
         for (int m=0; m<M; m++) {
            if (! mFemModel[m].getDynamicsEnabled ()) 
               continue;
            
            mDiffusion.setTarget (mMesh[m], mMeshChems[m]);
            mDiffusion.advance ((t1-t0)*mDiffusionTimestepScale);
         }
         customPostDiffusion();
         end = System.currentTimeMillis ();
         System.out.println ("Diffusion solve (ms): " + (end-start));
      }
      
//      // Handle remeshing.
//      if (MathUtil.compare (t0 % mRemeshFreq, 0) == 0 && mEnableRemesh &&
//          mNumRemeshRemaining > 0 && t0 > 0.01) 
//      {
//         start = System.currentTimeMillis ();
//         
//         mRemesher.mSizingField.updateParameters (mSizeMin, mSizeMax, mAspectMin,
//            mRefineAngle, mRefineCompression, mRefineVelocity);
//         
//         for (int m=0; m<M; m++) {
//            if (! mFemModel[m].getDynamicsEnabled ()) 
//               continue;
//            
//            mRemesher.setTarget (mMesh[m], mFemModel[m], mMeshChems[m]);
//            
//            // Ensure constraints are interpolated during remeshing.
//            // CSNCMT
//            mRemesher.setConstraints ( CollisionDetector.myCCAgg );
//            
//            mRemesher.remesh ();   
//            ShellUtil.invalidateFem (mFemModel[m]);
//            
//            // Make sure collision detection uses updated surface mesh.
//            // CSNCMT
//            CollisionManager colMgr = mMechModel.getCollisionManager ();
//            CollisionDetector contCldr = colMgr.getContinuousCollider ();
//            if (contCldr != null && mRemesher.isEleModified)
//               contCldr.rebuildSweptMeshInfo (mFemModel[m].getMeshComp (0));
//         }
//
//         mNumRemeshRemaining--;
//         System.out.printf ("Remesh occurred. Eles: %d, Nodes: %d \n", 
//            getNumShellElements(), getNumNodes());
//
//         end = System.currentTimeMillis ();
//         System.out.println ("Remesh solve (ms): " + (end-start));
//      }
//      
      // Convert fraction of morphogen into plastic strain.
      if (mEnableGrowth) {
         start = System.currentTimeMillis ();
         
         for (int m=0; m<M; m++) {
            if (! mFemModel[m].getDynamicsEnabled ()) 
               continue;
            
            mMorphogen2GrowthTensor.setTarget (mFemModel[m]);
            
            if (! mMaintainResidualStrain)
               mMorphogen2GrowthTensor.unapplyGrowthTensors ();
            
            mMorphogen2GrowthTensor.updatePolarityDirection(mPolDir);
            mMorphogen2GrowthTensor.activateFractionOfMorphogen (mChemCvtRate, 
               mIsActivatePAR, mIsActivatePER, mIsActivateNOR);
            mMorphogen2GrowthTensor.computeIntegrationGrowthTensors ();
            mMorphogen2GrowthTensor.applyGrowthTensors (); 
            mMorphogen2GrowthTensor.clearActivatedMorphogen ();   
         }
         
         end = System.currentTimeMillis ();
         System.out.println ("Tensor solve (ms): " + (end-start));
      }
      
      // Plastic Embedding
      if (mEnablePlasticEmbedding) {
         start = System.currentTimeMillis ();
         
         for (int m=0; m<M; m++) {
            if (! mFemModel[m].getDynamicsEnabled ()) 
               continue;
            
            // Use this instead if simulating both reference and world-space.
   //         mPlasticEmbedder.setTarget (mFemModel);
   //         mPlasticEmbedder.advance (t0, t1);
   //         mPlasticEmbedder.preadvance (); 
            
            mPlasticEmbedder.setTarget (mFemModel[m]);
            mPlasticEmbedder.copyWorldSpaceToReferenceSpace ();
         }
         
         end = System.currentTimeMillis ();
         System.out.println ("PlasticEmbedding solve (ms): " + (end-start));
      }
      else {
//         mMorphogen2GrowthTensor.unapplyGrowthTensors ();
//         mPlasticEmbedder.backupWorldSpace ();
//         mPlasticEmbedder.worldSpaceMode ();
      }
//      
//      updateNodesColor();
//      
//      if (mSurfaceColor == SurfaceColor.PLASTIC_STRAIN) 
//         mGrowColorer.computePlasticStrainColors ();
//      else if (mSurfaceColor == SurfaceColor.MORPHOGEN) 
//         mGrowColorer.computeMorphogenColors ();
//      else if (mGrowColorer != null)
//         mGrowColorer.toggleOff ();
//      
//      mCurElapsedTime = System.currentTimeMillis () - mStartTime;
//      System.out.println ("Time Elapsed (ms): " + mCurElapsedTime);
//      System.out.println ("Time step solve (ms): " + 
//         (mCurElapsedTime - mPrvElapsedTime));
//      mPrvElapsedTime = mCurElapsedTime;
   }
}

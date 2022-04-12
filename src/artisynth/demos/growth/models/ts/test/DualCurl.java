package artisynth.demos.growth.models.ts.test;

import static java.lang.Math.PI;

import java.awt.Color;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.demos.growth.models.paper.Basic_Base;
import artisynth.demos.growth.models.ts.ThinShellType;
import artisynth.demos.growth.models.ts.evouga.DiscreteShell;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

//-model artisynth.demos.growth.models.ts.test.DualCurl
//-noTimeline     
//-disableHybridSolves
//-playFor 3

/**
 * Simulation to make a square patch curl into a cylinder.
 * 
 * Adjustable parameters:
 * 
 *     // Element type
 *     mEleClass = VOLUMETRIC
 *                 SHELL
 *                 MEMBRANE
 *     
 *     // Membrane type
 *     mTsType = NARAIN (Bending Energy based on Angular Strain)  
 *               EVOUGA (Bending Energy based on 2nd Fundamental Form) 
 *     
 *     // Thickness variations. Can leave at 1 for 0.01 (1e-2) thickness. 
 *     t = 0 (1e-3)
 *         1 (1e-2)
 *         2 (1e-1)
 *         3 (1e-2 with 7.5x stress)
 *     
 *         Use t=1 for the 2PI cylinder curl experiment.
 *         Use t=3 for the amplified curl experiment. 
 *         
 *     meshDiv = 25 (1250 Elements) | 
 *               50 (5000 Elements) | 
 *               100 (20,000 Elements)
 */
public class DualCurl extends Basic_Base {

   protected final double AMPLIFIED_STRESS_MULTIPLIER = 5.0; // 7.5
   
   /** Thickness option to simulate */
   protected int t;
   
   /** Constant prestrain that will be applied to all elements. */
   protected Matrix3d mFixedBendingStrainMtx = null;
   
   static {
      ShellTriElement.myDefaultIntegrationCoords = ShellTriElement.INTEGRATION_COORDS_GAUSS_6;
   }

   // Discrete Shell Variables
   
   protected String mDiscreteShellCacheDir = "C:\\Users\\dan\\pj\\libshell\\example";
   protected boolean mIsBuildingRestState = false;
   
   /** 
    * Inverse timestep size. 
    */
   protected double mReg = 0.20;

   protected void build_pre() {
      super.build_pre();
            
      // --- Adjustable parameters --- //
      
      t = 3;
      
//      mEleClass = ElementClass.VOLUMETRIC;
//      mEleClass = ElementClass.SHELL;
      mEleClass = ElementClass.MEMBRANE;
      
//      this.mTsType = ThinShellType.NARAIN;
      this.mTsType = ThinShellType.EVOUGA;
    
      int meshDiv = 25;
      
//      mMinEnergyBeforePausing = 1e-6;
//    mMinEnergyBeforePausing = -1;
      mMinEnergyBeforePausing = 1e-5;
      mPauseEveryInterval = 999;
      
      // --- Setup --- //
      
      mEnableDiffusion = false;
      mEnableGrowth = true; 
      mEnableRemesh = false; 
      mEnablePlasticEmbedding = false; 
      
      mEnableCollisionHandling = false;
      
      mMeshX = 1;
      mMeshY = 1;
       
      mMeshXDiv = meshDiv;
      mMeshYDiv = meshDiv;
      
      morphogenSrcDuration = 0.01;
      
      //
      
      mRenderMode = RenderMode.DEFAULT;
      mSurfaceColor = SurfaceColor.DEFAULT;

      if (t < 3) {
         mCameraCenter = new Point3d(0,0,0);
         mCameraEye = new Point3d(1.8275, -0.536991, 0.506706);
      } else {
         mCameraCenter = new Point3d(0, 0, 0);
         mCameraEye = new Point3d(1.8275, -0.536991, 0.506706);
      }
      
      double[] thicknesses = new double[] {1e-3, 1e-2, 1e-1, 1e-2};
      double[] youngModuluses = new double[] {1e4, 1e4, 1e4, 1e4};
      double angScale = (t == 3) ? AMPLIFIED_STRESS_MULTIPLIER : 1;   // 15 : 1
      double width = (mMeshX/(float)mMeshXDiv);  
      double a = thicknesses[t];
      double theta = angScale*2*PI / mMeshXDiv;  // Rotationa angle.
      double o = a * Math.tan (theta);           // Top-layer stretched length for given side, absolute units.
      o *= 2;                                    // NEW: 2x to account for both sides.
      double strain = (o/width);                 // Length relative to unstretched length.
      System.out.printf ("Strain: %.2f \n", strain);
      double[] sidedStrains_vol = new double[] {strain, strain, strain, strain}; 
      double[] sidedStrains_shell = new double[] {strain, strain, strain, strain};
      double[] pauses_vol = new double[] {999, 999, 999, 3};
      double[] pauses_shell = new double[] {999, 999, 999, 3};
      
      double[] thicknesses_ts = new double[] {     1e-3,  1e-2, 1e-1, 1e-2 };
      double[] youngModuluses_ts = new double[] {  1e4,   1e4,  1e4,  1e4 };
      double[] angularStrainsQual = new double[] { 2*PI,  2*PI, 2*PI, 2*PI*AMPLIFIED_STRESS_MULTIPLIER};       
      double[] pauses_ts = new double[] {          999,    999,   999,   3};

      if (mEleClass == ElementClass.MEMBRANE && mTsType == ThinShellType.NARAIN) {
         m_shellThickness = thicknesses_ts[t];  // 1
         m_youngsModulus = youngModuluses_ts[t]; // 1e8
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,angularStrainsQual[t],0},  // -5 * PI 
            new double[] {0,0,0}
         });
         
         mPauseEveryInterval = pauses_ts[t];
      } else if (mEleClass == ElementClass.MEMBRANE && mTsType == ThinShellType.EVOUGA) {
         m_shellThickness = thicknesses_ts[t];  
         m_youngsModulus = youngModuluses_ts[t]; 
//         m_poissonsRatio = 0.5;
//         m_youngsModulus = 0.5;
//         m_youngsModulus = 0.25;
//         m_youngsModulus = 0.1;
      } else if (mEleClass == ElementClass.SHELL) {
         m_shellThickness = thicknesses[t];
         m_youngsModulus = youngModuluses[t];
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,sidedStrains_shell[t],0},
            new double[] {0,0,0}
         });
         
         mPauseEveryInterval = pauses_shell[t]; 

      } else {
         m_shellThickness = thicknesses[t];
         m_youngsModulus = youngModuluses[t];
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,sidedStrains_vol[t],0},
            new double[] {0,0,0}
         });

         mPauseEveryInterval = pauses_vol[t];
      }
      
      mSizeMin = 0.05;
      mSizeMax = mSizeMin*5;
      mDiffusionTimestepScale = 0.01;
      
      mShowColorBar = false;
      
      morphogenSrcDuration = 3;
      this.mIsActivatePAR = false;
      this.mIsActivatePER = true;
      
      // Residual display
      if (false) {
         mShowColorBar = false; 
         mSurfaceColor = SurfaceColor.RESIDUAL_PLASTIC_BENDING_STRAIN;
//         mPauseEveryInterval = 4;
//         mCameraEye = new Point3d(0.0, 0, 2.1991);
//         mAxisAlignedRotation = AxisAlignedRotation.NY_X;
      }
      
      mPauseEveryInterval = 999;
   }
   
   protected void build_modelSkeleton() {
      super.build_modelSkeleton ();
      
      try {
         if (mEleClass == ElementClass.MEMBRANE && mTsType == ThinShellType.EVOUGA) {
            // Ensure directory exists.
            Path discreteShellCacheDirPath = Paths.get (mDiscreteShellCacheDir);
            if (!Files.exists (discreteShellCacheDirPath)) {
               Files.createDirectory (discreteShellCacheDirPath);
            }
            
            // Use rest state mesh if doesn't exists.
            if (!Files.exists (getRestStatePath())) {
               mIsBuildingRestState = true;
               mMesh[0] =  MeshUtil.createCylinderFromPlane_YAxisCurved (
                  mMeshX, mMeshY, mMeshXDiv, mMeshYDiv, (t == 3) ? AMPLIFIED_STRESS_MULTIPLIER : 1);
            }
         }
      } catch (Exception ex) {
         throw new RuntimeException(ex);
      }
   }
   
   protected void build_modelStructure() {
      if (mEleClass == ElementClass.MEMBRANE && mTsType == ThinShellType.EVOUGA) {
         if (mIsBuildingRestState) {
            super.build_modelStructure ();
            DiscreteShell ds = ((DiscreteShell)mFemModel[0].myThinShellAux.getThinShellBase ());
            ds.saveState (getRestStatePath().toString ());
            
            // Restore original mesh.
            super.build_modelSkeleton ();
         } 
         
         // Restore original model.
         myModels.clear ();
         super.build_modelStructure ();
         
         // Set rest state.
         DiscreteShell ds = ((DiscreteShell)mFemModel[0].myThinShellAux.getThinShellBase ());
         ds.loadRestState (getRestStatePath().toString());
         
         // Set other DS parameters.
         ds.mReg = mReg;
      } else {
         super.build_modelStructure ();
      }
   }
   
   protected void build_modelProperties() {
      super.build_modelProperties();
      
      FemMaterial mat = null;
      mat = new LinearMaterial(m_youngsModulus, m_poissonsRatio);
      
      for (int m=0; m<M; m++) {
        mFemModel[m].setMaterial (mat);
      }
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.0005;
      
      mRendCfg.mDirectorLen = 0; 
      mRendCfg.mFrontMeshColor = Color.LIGHT_GRAY; 
      mRendCfg.mRearMeshColor = Color.GREEN;
      
      if (mEleClass == ElementClass.VOLUMETRIC) {
         mRendCfg.mFrontMeshColor = Color.RED; 
//         mRendCfg.mRearMeshColor = Color.RED;
      } else if (mEleClass == ElementClass.SHELL) {
         mRendCfg.mFrontMeshColor = new Color(204, 204, 204); 
         mRendCfg.mRearMeshColor = new Color(0, 102, 153);
      } else if (this.mTsType == ThinShellType.NARAIN) {
         mRendCfg.mFrontMeshColor = Color.CYAN; 
//         mRendCfg.mRearMeshColor = Color.CYAN;
      } else {
         mRendCfg.mFrontMeshColor = Color.CYAN; 
         mRendCfg.mRearMeshColor = Color.YELLOW;
      }
   }
   
   protected void build_post() {
      super.build_post ();
      mMorphogen2GrowthTensor.isBendingMorphogenHack = true;
      mMorphogen2GrowthTensor.fixedBendingStrain = mFixedBendingStrainMtx;
      mMorphogen2GrowthTensor.zeroStrainAtBottom = true;
      
      System.out.printf ("Element count: %d\n", this.mFemModel[0].numAllElements ());
      
      if (mEleClass == ElementClass.MEMBRANE && mTsType == ThinShellType.EVOUGA) {
         mMechModel.setDynamicsEnabled (false);      
      }
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();
      return (pnt.distance (Vector3d.ZERO) < 0.3);
   }
   
   /** -- Engine Loop -- **/
   
   public void advanceCustom(double t0, double t1, int flags) {
      if (mEleClass == ElementClass.MEMBRANE) {
         mFemModel[0].myThinShellAux.advance ();
      }
      super.advanceCustom (t0, t1, flags);
   }
   
   /** -- IO -- **/
   
   protected Path getRestStatePath() {
      return Paths.get (
         mDiscreteShellCacheDir, 
         String.format ("cylinder_t=%d_meshDiv=%d_amp=%.1f.rest", 
            t, mMeshXDiv, AMPLIFIED_STRESS_MULTIPLIER));
   }
}

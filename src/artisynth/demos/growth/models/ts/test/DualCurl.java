package artisynth.demos.growth.models.ts.test;

import java.awt.Color;
import static java.lang.Math.PI;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.materials.OgdenMaterial;
import artisynth.demos.growth.GrowChemical;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.base.GrowDemo.SurfaceColor;
import artisynth.demos.growth.models.base.ShellPatch.RenderConfig;
import artisynth.demos.growth.models.paper.Basic_Base;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

//  artisynth.demos.growth.models.ts._Debug_ThinShell

/**
 * Simulation to make a square patch curl into a cylinder.
 * 
 * Paramters to adjust:
 *     // Element type
 *     mEleClass = VOLUMETRIC | SHELL | MEMBRANE
 *     
 *     // Thickness variations. Can leave at 1 for 0.01 (1e-2) thickness. 
 *     t = 0 | 1 | 2 | 3
 *     
 *         Use t=1 for the 2PI cylinder curl experiment.
 *         Use t=3 for the amplified curl experiment. 
 *         
 *     Use mMeshXDiv and mMeshYDiv == 25 to observe less exaggerated
 *     opposite axis bending.
 */
public class DualCurl extends Basic_Base {
   
   protected Matrix3d mFixedBendingStrainMtx = null;
   protected boolean zeroStrainAtBottom = true;
   

   protected void build_pre() {
      super.build_pre();
            
      mEnableDiffusion = false;
      mEnableGrowth = true; 
      mEnableRemesh = false; 
      mEnablePlasticEmbedding = false; 
      
      mEnableCollisionHandling = false;
      
      mMeshX = 1;
      mMeshY = 1;
       
      mMeshXDiv = 50;
      mMeshYDiv = 50;
      
      morphogenSrcDuration = 0.01;
      
      //
      
      mRenderMode = RenderMode.DEFAULT;
      mSurfaceColor = SurfaceColor.DEFAULT;
      
      // Configuration
      
//      mEleClass = ElementClass.VOLUMETRIC;
      mEleClass = ElementClass.SHELL;
//      mEleClass = ElementClass.MEMBRANE;
      
      zeroStrainAtBottom = true;
      ShellTriElement.myDefaultIntegrationCoords = ShellTriElement.INTEGRATION_COORDS_GAUSS_6;
      
//      mMinEnergyBeforePausing = 1e-6;
//      mMinEnergyBeforePausing = -1;
      
      int t = 3;

      if (t < 3) {
         mCameraCenter = new Point3d(0,0,0);
         mCameraEye = new Point3d(1.8275, -0.536991, 0.506706);
      } else {
         mCameraCenter = new Point3d(0, 0, 0);
         mCameraEye = new Point3d(1.8275, -0.536991, 0.506706);
      }
      
      double[] thicknesses = new double[] {1e-3, 1e-2, 1e-1, 1e-2};
      double[] youngModuluses = new double[] {1e4, 1e4, 1e4, 1e4};
      double angScale = (t == 3) ? 7.5 : 1;   // 15 : 1
      double width = (mMeshX/(float)mMeshXDiv);  
      double a = thicknesses[t];
      double theta = angScale*2*PI / mMeshXDiv;  // /2
      double o = a * Math.tan (theta);   // absolute units
      double strain = (o/width);
      System.out.printf ("Strain: %.2f \n", strain);
      double[] sidedStrains_vol = new double[] {strain, strain, strain, strain};  //  Math.atan (20*PI/mMeshYDiv/2)
      double[] sidedStrains_shell = new double[] {strain, strain, strain, strain};
      double[] pauses_vol = new double[] {999, 999, 999, 3};
      double[] pauses_shell = new double[] {999, 999, 999, 3};
      
      /*
      12*PI = -76 radians 
      */
      
      double[] thicknesses_ts = new double[] {     1e-3,  1e-2, 1e-1, 1e-2 };
      double[] youngModuluses_ts = new double[] {  1e4,   1e4,  1e4,  1e4 };
      double[] angularStrainsQual = new double[] { 2*PI,  2*PI, 2*PI, 2*PI*7.5};       
      double[] pauses_ts = new double[] {          999,    999,   999,   3};

      if (mEleClass == ElementClass.MEMBRANE) {
         m_shellThickness = thicknesses_ts[t];  // 1
         m_youngsModulus = youngModuluses_ts[t]; // 1e8
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,angularStrainsQual[t],0},  // -5 * PI 
            new double[] {0,0,0}
         });
         
//         if (t == 0) {
//            ThinShellAux.mBendForceScaling = 100;
//         }
         
         mPauseEveryInterval = pauses_ts[t];
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
//      ShellPatch.m_particleDamping = 10;
      
      morphogenSrcDuration = 3;
      this.mIsActivatePAR = false;
      this.mIsActivatePER = true;
      
      if (false) {
         this.mIsActivatePAR = true;
         this.mFixedBendingStrainMtx = null;
         m_youngsModulus = 1e2;
         mPauseEveryInterval = 999;
      }
      
      // Residual display
      if (true) {
         mShowColorBar = false; 
         mSurfaceColor = SurfaceColor.RESIDUAL_PLASTIC_BENDING_STRAIN;
//         mPauseEveryInterval = 4;
//         mCameraEye = new Point3d(0.0, 0, 2.1991);
//         mAxisAlignedRotation = AxisAlignedRotation.NY_X;
      }
      
      // Shearing
      if (false) {
         m_shellThickness = 1e-1;
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,0.5,0},
            new double[] {0,0,0}
         });
      }
   }
   
   
   
//   protected void build_modelSkeleton() {
//      mMesh = new PolygonalMesh[M];
//      PolygonalMesh m = new PolygonalMesh();
//      
//      m.addVertex (0, 0, 0); 
//      m.addVertex (1, 0, 0); 
//      m.addVertex (1, 1, 0);
//      m.addFace (0, 1, 2);
//      
//      m.scale (0.1);
//      
//      mMesh[0] = m;
//   }
   
   protected void build_modelProperties() {
      super.build_modelProperties();
      
      FemMaterial mat = null;
      mat = new LinearMaterial(m_youngsModulus, m_poissonsRatio);
//      mat = new NeoHookeanMaterial(m_youngsModulus, m_poissonsRatio);  // Corset shape. Very similar to LinearMaterial
//      mat = new MooneyRivlinMaterial(1500, 0, 0, 0, 0, 150000);
//      mat = new OgdenMaterial();   // Pill shape 

      
      for (int m=0; m<M; m++) {
        mFemModel[m].setMaterial (mat);
      }
   }
   
//   protected void build_addUI() {
//   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.0005;
      
      mRendCfg.mDirectorLen = 1; 
      mRendCfg.mFrontMeshColor = Color.LIGHT_GRAY; 
      mRendCfg.mRearMeshColor = Color.GREEN;
      
      if (mEleClass == ElementClass.VOLUMETRIC) {
         mRendCfg.mFrontMeshColor = Color.RED; 
//         mRendCfg.mRearMeshColor = Color.RED;
      } else if (mEleClass == ElementClass.SHELL) {
         mRendCfg.mFrontMeshColor = new Color(204, 204, 204); 
         mRendCfg.mRearMeshColor = new Color(0, 102, 153);
      } else {
         mRendCfg.mFrontMeshColor = Color.CYAN; 
//         mRendCfg.mRearMeshColor = Color.CYAN;
      }
   }
   
   protected void build_post() {
      super.build_post ();
      mMorphogen2GrowthTensor.isBendingMorphogenHack = true;
      mMorphogen2GrowthTensor.fixedBendingStrain = mFixedBendingStrainMtx;
      mMorphogen2GrowthTensor.zeroStrainAtBottom = zeroStrainAtBottom;
      
      System.out.printf ("Element count: %d\n", this.mFemModel[0].numAllElements ());
   }
   
   public boolean isMorphogenSrcNode(int v) {
      Vertex3d vtx = mMesh[0].getVertex (v);
      Point3d pnt = vtx.getPosition ();
      return (pnt.distance (Vector3d.ZERO) < 0.3);
//      return true;
   }
   
   /**
    * Create a mesh, which will be used as a template for building 
    * the FEM model.
    */
   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
//            gNode.mChems.set (3, 1);
            
//            gNode.setVelocity (0, 0, 1);
//            break;
         }
      }
   }
   
   
   /** -- Engine Loop -- **/

   
   public void advanceCustom(double t0, double t1, int flags) {
      super.advanceCustom (t0, t1, flags);
//      if (t0 == 0.00) {
//         PointList<FemNode3d> nodes = this.mFemModel[0].getNodes ();
//         nodes.get (0).setRestPosition (new Point3d(0,0,0));
//         nodes.get (1).setRestPosition (new Point3d(-1,0,0));
//         nodes.get (2).setRestPosition (new Point3d(-1,1,0));
//         System.out.println ("Switched");
//      }
//      if (t0 > 0) {
//         mMorphogen2GrowthTensor.fixedBendingStrain = null;
//      }
      
      this.mFemModel[0].getEnergy ();
   }
}

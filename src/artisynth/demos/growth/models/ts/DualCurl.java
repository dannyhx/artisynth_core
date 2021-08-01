package artisynth.demos.growth.models.ts;

import java.awt.Color;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.demos.growth.GrowChemical;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.paper.Basic_Base;
import artisynth.demos.growth.thinshell.ThinShellAux;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix3d;

//  artisynth.demos.growth.models.ts._Debug_ThinShell

/*
     Solid-Shell:
        Straight:
        
        Arc:
           1e-2 thickness
           1e4 young
           0.5 strain
     
     Thin-Shell:
        Straight:
           1e-2 thickness
           1e4 young
           20*PI
        
        Arc:
           5e-2 thickness
           1e4 young
           40*PI
            
 */

public class DualCurl extends Basic_Base {
   
   protected Matrix3d mFixedBendingStrainMtx = null;
   
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
      
      mEleClass = ElementClass.VOLUMETRIC;
//      mEleClass = ElementClass.SHELL;
//      mEleClass = ElementClass.MEMBRANE;
      
      int t = 1;
      
      double[] thicknesses = new double[] {1e-3, 1e-2};
      double[] youngModuluses = new double[] {1e4, 1e4};
      double[] sidedStrains_vol = new double[] {0.85, 0.85};
      double[] sidedStrains_shell = new double[] {0.6, 0.6};
      double[] pauses_vol = new double[] {3.52,7.96};
      double[] pauses_shell = new double[] {3.43,8.46};
      
      double[] thicknesses_ts = new double[] {1e-3, 1e-2,    1e-2};
      double[] youngModuluses_ts = new double[] {1e4, 1e4,   1e4};
      double[] angularStrainsQual = new double[] {4*Math.PI, 40*Math.PI,    50*Math.PI};
      double[] pauses_ts = new double[] {10.59,5.33, 999};
//    double[] angularStrains = new double[] {0.12435, 0.12435, 0.12435, 0.12435}; 

      
      //
      
      if (mEleClass == ElementClass.MEMBRANE) {
         m_shellThickness = thicknesses_ts[t];  // 1
         m_youngsModulus = youngModuluses_ts[t]; // 1e8
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,angularStrainsQual[t],0},  // -5 * PI 
            new double[] {0,0,0}
         });
         
         if (t == 0) {
            ThinShellAux.mBendForceScaling = 100;
         }
         
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
   
   protected void build_addUI() {
   }
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.00;
      
      mRendCfg.mDirectorLen = 0; 
      mRendCfg.mFrontMeshColor = Color.LIGHT_GRAY; 
      mRendCfg.mRearMeshColor = Color.GREEN;
      
      if (mEleClass == ElementClass.VOLUMETRIC) {
         mRendCfg.mFrontMeshColor = Color.GREEN; 
         mRendCfg.mRearMeshColor = Color.GREEN;
      }
   }
   
   protected void build_post() {
      super.build_post ();
      mMorphogen2GrowthTensor.isBendingMorphogenHack = true;
      mMorphogen2GrowthTensor.fixedBendingStrain = mFixedBendingStrainMtx;
   }
   
   public boolean isMorphogenSrcNode(int v) {
//      Vertex3d vtx = mMesh[0].getVertex (v);
//      Point3d pnt = vtx.getPosition ();

      return true;
   }
   
   /**
    * Create a mesh, which will be used as a template for building 
    * the FEM model.
    */
   public void advanceCustom_applyMorphogen(double t0) {
      for (int v = 0; v < mMesh[0].numVertices (); v++) {
         if ( isMorphogenSrcNode(v) && t0 < morphogenSrcDuration) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[0].getNode (v);
            gNode.mChems.set (GrowChemical.PAR.mIdx, 0.1);
            
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
   }
}

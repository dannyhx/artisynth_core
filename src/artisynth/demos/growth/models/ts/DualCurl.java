package artisynth.demos.growth.models.ts;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.demos.growth.GrowChemical;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.paper.Basic_Base;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix3d;

//  artisynth.demos.growth.models.ts._Debug_ThinShell

/*
      if (this.m_isMembrane) {
         m_shellThickness = 1;
         m_youngsModulus = 1e8;
      } else {
         m_shellThickness = 1e-2;
         m_youngsModulus = 1e6;
      }
   
                strainMtx.set (new double[][] {
                  new double[] {0,0,0}, 
                  new double[] {0,0.5,0},
                  new double[] {0,0,0}
               });
               
            avgStrain.set (new double[][] {
               new double[] {0,0,0}, 
               new double[] {0,-5*Math.PI,0},
               new double[] {0,0,0}
            });
            
 */

public class DualCurl extends Basic_Base {
   
   protected Matrix3d mFixedBendingStrainMtx = null;
   
   protected void build_pre() {
      super.build_pre();
            
      mEnableDiffusion = true;
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
      double[] thicknesses = new double[] {1e-3, 1e-2, 1e-2, 1e-1};
      double[] youngModuluses = new double[] {1e6, 1e6, 1e6, 1e6};
      double[] bottomStrains = new double[] {0.25, 0.25, 0.25, 0.25};
      double[] angularStrain = new double[] {0.12435, 0.12435, 0.12435, 0.12435}; 
      int t = 0;
      
      //
      
      if (mEleClass == ElementClass.MEMBRANE) {
         m_shellThickness = 1;
         m_youngsModulus = youngModuluses[t];
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,-5*Math.PI,0},
            new double[] {0,0,0}
         });
      } else if (mEleClass == ElementClass.SHELL) {
         m_shellThickness = 1e-2;
         m_youngsModulus = youngModuluses[t];
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,0.25,0},
            new double[] {0,0,0}
         });
      } else {
         m_shellThickness = 1e-2;
         m_youngsModulus = youngModuluses[t];
         
         mFixedBendingStrainMtx = new Matrix3d();
         mFixedBendingStrainMtx.set (new double[][] {
            new double[] {0,0,0}, 
            new double[] {0,0.25,0},
            new double[] {0,0,0}
         });
      }
      
      mSizeMin = 0.05;
      mSizeMax = mSizeMin*5;
      mDiffusionTimestepScale = 0.01;
      mPauseEveryInterval = 999.00; 
      
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
//      mat = new LinearMaterial(m_youngsModulus, m_poissonsRatio);
      mat = new NeoHookeanMaterial(m_youngsModulus, m_poissonsRatio);  // Corset shape. Very similar to LinearMaterial
//      mat = new MooneyRivlinMaterial(1500, 0, 0, 0, 0, 150000);
//      mat = new OgdenMaterial();   // Pill shape 

      
      for (int m=0; m<M; m++) {
        mFemModel[m].setMaterial (mat);
      }
   }
   
   
   protected void build_renderConfig() {
      super.build_renderConfig ();
      
      mRendCfg = mRendCfgPresets.get (RenderMode.DEFAULT);
      mRendCfg.mNodeRadius = 0.00;
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

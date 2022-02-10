package artisynth.demos.growth.models.ts.chen;

import java.util.ArrayList;
import java.util.HashMap;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.models.ts.ThinShellBase;
import artisynth.demos.growth.models.ts.chen.GeometryDerivative.FirstFundamentalFormRv;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix2d;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public class DiscreteShell extends ThinShellBase {
   
   protected MonolayerRestState mRestState;
   protected DiscreteShellMaterial mMat;
   
   protected final int mNumExtraDOFs = 1;
   
   // FE[f,e] = 'global edge index'
   protected int[][] mFE = null;
   
   public DiscreteShell(FemModel3d model, PolygonalMesh mesh) {
     super(model, mesh);
     this.setMaterialProperties (0, 0, 0);
     
     // main.cpp::runSimulation()
     
     // Initial rest geometry of shell.
     double thickness = model.getShellElement (0).getDefaultThickness ();
     mRestState = new MonolayerRestState(model, thickness);
     
     // Initialize first fundamental forms of mesh.
     firstFundamentalForms(mRestState.abars);
     
     // Initialize second fundamental forms to rest flat.
     for (int i = 0; i < mRestState.bbars.size (); i++) {
        mRestState.bbars.get (0).setZero ();
     }
     
     mFE = MeshUtil.createGlobalEdgeIndices (mesh);
   }
   
   @Override
   public void setMaterialProperties (
      double youngsModulus, double poissonsRatio, double thickness) {
      this.mMat = new StVKMaterial();
   }

   /**
    * ElasticShell.cpp :: elasticEnergy
    */
   @Override
   public void addStretchingForceAndStiffness () {
      // Start of StaticSolve.h :: takeOneStep()
      VectorNd derivative = new VectorNd(this.mModel.numNodes ());
      ArrayList<double[]> hessian = new ArrayList<double[]>();
      
      // double energy = LibShell::ElasticShell<SFF>::elasticEnergy(
      // mesh, curPos, curEdgeDOFs, mat, restState, &derivative, &hessian);
      int nNodes = this.mModel.numNodes ();
      int nEdges = MeshUtil.numEdges (mMesh);
      
      derivative.adjustSize (3 * nNodes + mNumExtraDOFs * nEdges);
      derivative.setZero ();
      
      double result = 0;
      
      // Stretching
      
      for (int f = 0; f < mMesh.numFaces (); f++) {
         ShellElement3d ele = mModel.getShellElement (f);
         
         VectorNd deriv = new VectorNd(9);
         MatrixNd hess = new MatrixNd(9, 9);
         
         result += mMat.stretchingEnergy (
            ele, mRestState, f, 
            (derivative != null) ? deriv : null, 
            (hessian != null) ? hess : null
         );
         
         if (derivative != null) {
            VectorNd deriv_seg = new VectorNd(3);
            VectorNd derivative_seg = new VectorNd(3);
            VectorNd seg_sum = new VectorNd(3);
            
            for (int j = 0; j < 3; j++) {
               FemNode3d node = ele.getNodes ()[j];
               int n = node.getIndex ();
               
               deriv.getSubVector (3*j, deriv_seg);
               derivative.getSubVector (3*n, derivative_seg);
               
               seg_sum.add (deriv_seg, derivative_seg);
               derivative.setSubVector (3*n, seg_sum);
            }
         }
         
         if (hessian != null) {
            for (int j = 0; j < 3; j++) {
               FemNode3d node = ele.getNodes ()[j];
               int n = node.getIndex ();
               
                for (int k = 0; k < 3; k++) {
                    for (int l = 0; l < 3; l++) {
                        for (int m = 0; m < 3; m++) {
                           hessian.add (new double[] {
                              3*n + l, 
                              3*n + m,
                              hess.get (3*j+l, 3*k+m)
                           });
                        }
                    }
                }
            }
         }
      }
      
      // Bending terms
      
      int nedgedofs = this.mNumExtraDOFs;
      
      for (int f = 0; f < mMesh.numFaces (); f++) {
         Face face = mMesh.getFace (f);
         ShellElement3d ele = mModel.getShellElement (f);
         
         VectorNd deriv = new VectorNd(18 + 3 * nedgedofs);
         MatrixNd hess = new MatrixNd(18 + 3 * nedgedofs, 18 + 3 * nedgedofs);

         this.mMat.bendingEnergy (mModel, ele, mRestState, face, f, nedgedofs, 
            (derivative != null) ? deriv : null, 
            (hessian != null) ? hess : null);
         
         if (derivative != null) {
            VectorNd deriv_seg = new VectorNd(3);
            VectorNd derivative_seg = new VectorNd(3);
            VectorNd seg_sum = new VectorNd(3);
            
            for (int j = 0; j < 3; j++) {
               FemNode3d node = ele.getNodes ()[j];
               int n = node.getIndex ();
               
               deriv.getSubVector (3*j, deriv_seg);
               derivative.getSubVector (3*n, derivative_seg);
               
               seg_sum.add (deriv_seg, derivative_seg);
               derivative.setSubVector (3*n, seg_sum);
               
               // Get opposite node index.
               Vertex3d oppVtx = MeshUtil.getOppositeVtx (mMesh.getVertex (n), face);
               int on = (oppVtx != null) ? oppVtx.getIndex () : -1;
               
               if (oppVtx != null) {
                  deriv.getSubVector(9+3*j, deriv_seg);
                  derivative.getSubVector (3*on, derivative_seg);
                  
                  seg_sum.add (deriv_seg, derivative_seg);
                  derivative.setSubVector (3*on, seg_sum);
               }
               
               for (int k = 0; k < nedgedofs; k++) {
                  double s = derivative.get (3 * nNodes + nedgedofs * n + k);
                  s += deriv.get (18 + nedgedofs * j + k);
                  derivative.set (3 * nNodes + nedgedofs * mFE[f][j] + k, s);
               }
            }
         }
         
         if (hessian != null) {
            for (int j = 0; j < 3; j++) {
               int n = ele.getNodes ()[j].getIndex ();
               Vertex3d oVtx = MeshUtil.getOppositeVtx (mMesh.getVertex (n), face);
               int o = (oVtx != null) ? oVtx.getIndex () : -1;
               
               for (int k = 0; k < 3; k++) {
                  int nk = ele.getNodes ()[k].getIndex();
                  Vertex3d okVtx = MeshUtil.getOppositeVtx (mMesh.getVertex (nk), face);
                  int ok = (okVtx != null) ? okVtx.getIndex () : -1;
                  
                  for (int l = 0; l < 3; l++) {
                     for (int m = 0; m < 3; m++) { 
                        hessian.add (new double[] {
                           3*n+l, 3*nk+m, hess.get(3*j+l, 3*k+m)});
                        
                        if (okVtx != null) {
                           hessian.add (new double[] {
                              3*n+l, 3*ok+m, hess.get (3*j+l,9+3*k+m)});
                        }
                        
                        if (oVtx != null) {
                           hessian.add (new double[] {
                              3*o+l, 3*ok+m, hess.get (9+3*j+l,3*k+m)});
                        }
                        
                        if (oVtx != null && okVtx != null) {
                           hessian.add (new double[] {
                              3*o+l, 3*ok+m, hess.get (9+3*j+l, 9+3*k+m)});
                        }
                     }
                     
                     for (int m = 0; m < nedgedofs; m++) {
                        hessian.add (new double[] {
                           3*n+l, 3*nNodes+nedgedofs * mFE[f][k] + m, 
                           hess.get (3*j*l, 18+nedgedofs*k+m)
                        });
                     }
                  }
               }
            }
                          
         }
      }
      
      
   }

   @Override
   public void addBendingForceAndStiffness () {
      // TODO Auto-generated method stub
   }
   
   ///////////// Helper functions
   
   protected void firstFundamentalForms(ArrayList<Matrix2d> abars) {
      for (int f = 0; f < this.mModel.numShellElements (); f++) {
         ShellElement3d ele = this.mModel.getShellElement (f);
         
         Matrix2d I = GeometryDerivative.firstFundamentalForm (ele, null, null);
         abars.set (f, I);
      }
   }
   
   protected void secondFundamentalForms() {
      
   }
}

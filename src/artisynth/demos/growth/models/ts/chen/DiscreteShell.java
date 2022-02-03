package artisynth.demos.growth.models.ts.chen;

import java.util.ArrayList;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.models.ts.ThinShellBase;
import artisynth.demos.growth.models.ts.chen.DiscreteShellMaterial.StretchingEnergyRv;
import artisynth.demos.growth.models.ts.chen.GeometryDerivative.FirstFundamentalFormRv;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix2d;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public class DiscreteShell extends ThinShellBase {
   
   protected MonolayerRestState mRestState;
   protected DiscreteShellMaterial mMat;
   
   protected final int mNumExtraDOFs = 1;
   
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
      
      int nNodes = 3 * this.mModel.numNodes ();
      int nEdges = MeshUtil.numEdges (mMesh);
      
      derivative.adjustSize (3 * nNodes + mNumExtraDOFs * nEdges);
      derivative.setZero ();
      
      double result = 0;
      
      // Stretching
      
      for (int f = 0; f < mMesh.numFaces (); f++) {
         ShellElement3d ele = mModel.getShellElement (f);
         
         VectorNd deriv = new VectorNd(9);
         MatrixNd hess = new MatrixNd(9, 9);
         
         StretchingEnergyRv rv = mMat.stretchingEnergy (
            ele, mRestState, f, 
            (derivative != null) ? deriv : null, 
            (hessian != null) ? hess : null
         );
         result += rv.Result;
         
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
         MatrixNd deriv = new MatrixNd(1, 18 + 3 * nedgedofs);
         MatrixNd hess = new MatrixNd(18 + 3 * nedgedofs, 18 + 3 * nedgedofs);
         
         // material bending energy 
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
         
         FirstFundamentalFormRv ffrv = GeometryDerivative.firstFundamentalForm (ele);
         abars.set (f, ffrv.Result);
      }
   }
   
   protected void secondFundamentalForms() {
      
   }
}

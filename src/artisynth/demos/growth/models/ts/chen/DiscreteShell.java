package artisynth.demos.growth.models.ts.chen;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

import java.util.ArrayList;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.demos.growth.models.ts.ThinShellBase;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix2d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SparseMatrixCell;
import maspack.matrix.SparseMatrixNd;
import maspack.matrix.VectorNd;
import maspack.solvers.PardisoSolver;


public class DiscreteShell extends ThinShellBase {
   
   protected MonolayerRestState mRestState;
   protected DiscreteShellMaterial mMat;
   
   protected VectorNd mEdgeDOFs; // len = numEdges. All zero initially.
   
   // MidedgeAngleTanFormulation.h
   protected final int mNumExtraDOFs = 1;
   
   // FE[f,e] = 'global edge index'
   protected int[][] mFE = null;
   
   public DiscreteShell(FemModel3d model, PolygonalMesh mesh) {
     super(model, mesh);
     
     double thickness = model.getShellElement (0).getDefaultThickness ();  // 1e-1
     double poissonRatio = ((LinearMaterial)model.getMaterial ()).getPoissonsRatio (); // 1/2
     
     this.setMaterialProperties (0, poissonRatio, thickness);
     
     // main.cpp::runSimulation()
     
     // Initial rest geometry of shell.
     mRestState = new MonolayerRestState(model, thickness);
     
     // Initialize first fundamental forms of mesh.
     firstFundamentalForms(mRestState.abars);
     
     // Initialize second fundamental forms to rest flat.
     for (int i = 0; i < mRestState.bbars.size (); i++) {
        mRestState.bbars.get (0).setZero ();
     }
     
     mFE = MeshUtil.createGlobalEdgeIndices (mesh);
     
     // MidedgeAngleTanForumulation.cpp::initializeExtraDOFs
     int maxGlobalEdgeIdx = -1;
     for (int f = 0; f < mFE.length; f++) {
        for (int e = 0; e < 3; e++) {
           if (mFE[f][e] > maxGlobalEdgeIdx) {
              maxGlobalEdgeIdx = mFE[f][e];
           }
        }
     }
     mEdgeDOFs = new VectorNd(maxGlobalEdgeIdx+1);
   }
   
   @Override
   public void setMaterialProperties (
      double youngsModulus, double poissonsRatio, double thickness) {
      this.mMat = new StVKMaterial(poissonsRatio);
   }

   @Override
   public void advance() {
      double reg = 1e-6;  // main.cpp
      
      // StaticSolve.h::takeOneStep
      
      VectorNd derivative = new VectorNd();
      ArrayList<MatrixCell> hessian = new ArrayList<MatrixCell>();
      
      double energy = this.elasticEnergy (derivative, hessian);
      
      int freeDOFs = derivative.size ();
      
      SparseMatrixNd H = new SparseMatrixNd(freeDOFs, freeDOFs);
      for (MatrixCell cell : hessian) {
         H.set (cell.i, cell.j, cell.val);         
      }
      
      VectorNd force = new VectorNd(derivative);
      force.negate ();
      
      SparseMatrixNd sI = new SparseMatrixNd(freeDOFs, freeDOFs);
      sI.setIdentity ();
      sI.scale (reg);
     
      H.add (sI);
      
      VectorNd maxvals = new VectorNd(freeDOFs);
      maxvals.setZero ();
      // For each row
      for (int r = 0; r < freeDOFs; r++) {
         SparseMatrixCell row = H.getRow (r);
         if (row == null) {
            continue;
         }
         
         // For each column of row.
         for (SparseMatrixCell cell = row; row != null; row = row.next) {
            maxvals.set (r, max(maxvals.get (r), abs(cell.value)));
         }
      }
      
      ArrayList<MatrixCell> Dcoeffs = new ArrayList<MatrixCell>();
      for (int i = 0; i < freeDOFs; i++) {
         double val = (maxvals.get (i) == 0.0 ? 1.0 : 1.0 / sqrt(maxvals.get (i)));
         Dcoeffs.add (new MatrixCell(i, i, val));
      }
      
      SparseMatrixNd D = new SparseMatrixNd(freeDOFs, freeDOFs);
      for (MatrixCell cell : Dcoeffs) {
         D.set (cell.i, cell.j, cell.val);         
      }

      SparseMatrixNd DHDT = new SparseMatrixNd(freeDOFs, freeDOFs);
      DHDT.set(D);
      DHDT.mul (H);
      DHDT.mulTranspose (D);
      
      VectorNd rhs = new VectorNd();
      rhs.mul (D, force);
      
      VectorNd x = new VectorNd(rhs.size ());
      
      PardisoSolver solver = new PardisoSolver();
      solver.analyze (DHDT, freeDOFs, Matrix.POSITIVE_DEFINITE);
      solver.factor ();
      solver.solve(x, rhs);
      solver.dispose ();
      
      VectorNd descentDir = new VectorNd();
      descentDir.mul (D, x);

      // Update node positions.
      Point3d newPos = new Point3d();
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode (n);
         
         descentDir.getSubVector (3*n, newPos);
         newPos.add (node.getPosition ());
         
         node.setPosition (newPos);
      }
      
      // Update edge DoFs.
      VectorNd descentDir_seg = new VectorNd(mEdgeDOFs.size ()); 
      descentDir.getSubVector (3 * mModel.numNodes (), descentDir_seg);
      mEdgeDOFs.add (descentDir_seg);
   }
   
   
   /**
    * ElasticShell.cpp :: elasticEnergy
    * 
    * Verified.
    */
   public double elasticEnergy (
      VectorNd derivative,
      ArrayList<MatrixCell> hessian 
   ) {
      int nNodes = this.mModel.numNodes ();
      int nEdges = this.mEdgeDOFs.size ();
      
      if (derivative != null) {
         derivative.adjustSize(3 * nNodes + this.mNumExtraDOFs * nEdges);
         derivative.setZero ();
      }
      
      if (hessian != null) {
         hessian.clear ();
      }
      
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
               int n = ele.getNodes ()[j].getIndex ();
               
                for (int k = 0; k < 3; k++) {
                   int nk = ele.getNodes ()[k].getIndex ();
                   
                    for (int l = 0; l < 3; l++) {
                        for (int m = 0; m < 3; m++) {
                           hessian.add (new MatrixCell (
                              3*n  + l, 
                              3*nk + m,
                              hess.get (3*j+l, 3*k+m)
                           ));
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

         this.mMat.bendingEnergy (mModel, ele, mEdgeDOFs, mFE, mRestState, face, f, nedgedofs, 
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
                  int derivative_idx = 3 * nNodes + nedgedofs * mFE[f][j] + k;
                  double derivative_i = derivative.get (derivative_idx);
                  
                  derivative_i += deriv.get (18 + nedgedofs * j + k);
                  derivative.set (derivative_idx, derivative_i);
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
                        hessian.add (new MatrixCell (
                           3*n+l, 3*nk+m, hess.get(3*j+l, 3*k+m)));
                        
                        if (okVtx != null) {
                           hessian.add (new MatrixCell (
                              3*n+l, 3*ok+m, hess.get (3*j+l,9+3*k+m)));
                        }
                        
                        if (oVtx != null) {
                           hessian.add (new MatrixCell (
                              3*o+l, 3*k+m, hess.get (9+3*j+l,3*k+m)));
                        }
                        
                        if (oVtx != null && okVtx != null) {
                           hessian.add (new MatrixCell (
                              3*o+l, 3*ok+m, hess.get (9+3*j+l, 9+3*k+m)));
                        }
                     }
                     
                     for (int m = 0; m < nedgedofs; m++) {
                        hessian.add (new MatrixCell (
                           3*n+l, 3*nNodes+nedgedofs * mFE[f][k] + m, 
                           hess.get (3*j*l, 18+nedgedofs*k+m)
                        ));
                        
                        hessian.add (new MatrixCell (
                           3*nNodes+nedgedofs*mFE[f][k]+m, 3*n+l, 
                           hess.get (18+nedgedofs*k+m, 3*j+l)
                        ));
                        
                        if (oVtx != null) {
                           hessian.add (new MatrixCell (
                              3*o+l,3*nNodes+nedgedofs*mFE[f][k]+m, 
                              hess.get(9+3*j+l, 18+nedgedofs*k+m)
                           ));
                           
                           hessian.add (new MatrixCell (
                              3*nNodes+nedgedofs*mFE[f][k]+m, 3*o+l,
                              hess.get(18+nedgedofs*k+m, 9+3*j+l)
                           ));
                        }
                     }
                  }
                  
                  for (int m = 0; m < nedgedofs; m++) {
                     for (int ni = 0; ni < nedgedofs; ni++) {
                        hessian.add (new MatrixCell (
                           3*nNodes+nedgedofs*mFE[f][j]+m, 3*nNodes+nedgedofs*mFE[f][k]+ni, 
                           hess.get (18 + nedgedofs * j + m, 18 + nedgedofs * k + ni)
                        ));
                     }
                  }
               }
            }
         }
      }
      
      return result;
   }

   @Override
   public void addForceAndStiffness () {
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
   
}

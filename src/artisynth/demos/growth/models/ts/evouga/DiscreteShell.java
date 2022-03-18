package artisynth.demos.growth.models.ts.evouga;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.demos.growth.models.ts.ThinShellBase;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix2d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SparseMatrixCell;
import maspack.matrix.SparseMatrixNd;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.solvers.PardisoSolver;

/**
 * Implementation of Discrete Shells where DoFs are expressed in terms of
 * nodal positions and surface curvature (e.g. 2nd fundamental form).
 * 
 * See https://github.com/evouga/libshell
 * See LICENSE
 */
public class DiscreteShell extends ThinShellBase {
   
   protected MonolayerRestState mRestState;
   protected DiscreteShellMaterial mMat;
   
   protected VectorNd mEdgeDOFs; // len = numEdges. All zero initially.
   
   // MidedgeAngleTanFormulation.h
   protected final int mNumExtraDOFs = 1;
   
   protected MeshConnectivity mMC = null;
   
   public double mReg = 0.025; //1e-6
   
   public DiscreteShell(FemModel3d model, PolygonalMesh mesh) {
     super(model, mesh);
     
     double thickness = model.getShellElement (0).getDefaultThickness ();  // 1e-1
     double poissonRatio = ((LinearMaterial)model.getMaterial ()).getPoissonsRatio (); // 1/2
     
     this.setMaterialProperties (1.0, poissonRatio, thickness);
     
     mMC = new MeshConnectivity(mesh);
     
     // main.cpp::runSimulation()
     
     // Initial rest geometry of shell.
     mRestState = new MonolayerRestState(mMC, model, thickness);
     
     
     for (int f = 0; f < this.mModel.numShellElements (); f++) {
        // Initialize first fundamental form of the face.
        Matrix2d I = GeometryDerivative.firstFundamentalForm (mMC, mModel, f, null, null);
        mRestState.abars.set (f, I);
        
        // Assuming the model is a 2D plane initially, assume its principle fiber 
        // direction is mMC.d. To follow, calculate the orthogonal direction of mMC.d
        // while using the first fundamental form as the basis. Both vectors
        // are stored in mMC.T.

        Vector2d dI = new Vector2d();
        dI.set (mMC.d);
        I.mul (dI);  // Input: d, Output: dI
        
        // Assume dO.x is 1 
        
        double dOy = dI.x / -dI.y;
        
        Vector2d dO = new Vector2d(1, dOy);

        mMC.T[f].setColumns (mMC.d, dO);
        if (!mMC.T[f].invert ()) {
           throw new RuntimeException();
        }
     }
     
     // Initialize second fundamental forms to rest flat.
     for (int i = 0; i < mRestState.bbars.size (); i++) {
        mRestState.bbars.get (i).setZero ();
     }
     
     // MidedgeAngleTanForumulation.cpp::initializeExtraDOFs
     mEdgeDOFs = new VectorNd(mMC.EF.length);
   }
   
   //////////////////////////////////////
   // Public Setup Operations
   //////////////////////////////////////
   
   @Override
   public void setMaterialProperties (
      double youngsModulus, double poissonsRatio, double thickness) {
      this.mMat = new StVKMaterial(youngsModulus, poissonsRatio);
   }
   
   @Override
   public void addForceAndStiffness () {
      // Force and Stiffness are handled in advance(); call that instead.
   }
   
   /**
    * Set the rest curvative of all the faces. The principle direction of each
    * face will be respected.
    * 
    * @param M
    */
   public void setBarycentricRestII(Matrix2d M) {
      for (int f = 0; f < mRestState.bbars.size (); f++) { 
         Matrix2d T = mMC.T[f];
         
         Matrix2d Tinv = new Matrix2d(T);
         if (!Tinv.invert ()) {
            throw new RuntimeException();
         }
         
         Matrix2d a0_hb0 = new Matrix2d();
         a0_hb0.set (mRestState.abars.get (f));
//         a0_hb0.sub (b);  // Assume flat curvature basis.
         
         Matrix2d g_pos = new Matrix2d();
         g_pos.setIdentity ();
         g_pos.mulTranspose (T);
         g_pos.mul (M);
         g_pos.mulTranspose (Tinv); 
         g_pos.mul (a0_hb0);
         g_pos.mul (Tinv);
         g_pos.mul (M);
         g_pos.mul (T);

         mRestState.bbars.set (f, g_pos);
      }
   }

   //////////////////////////////////////
   // Public Advance Operation
   //////////////////////////////////////

   @Override
   public void advance() {
      // StaticSolve.h::takeOneStep
      
      VectorNd derivative = new VectorNd();
      ArrayList<MatrixCell> hessian = new ArrayList<MatrixCell>();
      
      double energy = this.elasticEnergy (derivative, hessian);
      
      int freeDOFs = derivative.size ();
      
      SparseMatrixNd H = MatrixCell.BuildSparseMatrixNd(freeDOFs, freeDOFs, hessian);
      
      VectorNd force = new VectorNd(derivative);
      force.negate ();
      
      SparseMatrixNd sI = new SparseMatrixNd(freeDOFs, freeDOFs);
      sI.setIdentity ();
      sI.scale (mReg);
     
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
         for (SparseMatrixCell cell = row; cell != null; cell = cell.next) {
            maxvals.set (r, max(maxvals.get (r), abs(cell.value)));
         }
      }
      
      ArrayList<MatrixCell> Dcoeffs = new ArrayList<MatrixCell>();
      for (int i = 0; i < freeDOFs; i++) {
         double val = (maxvals.get (i) == 0.0 ? 1.0 : 1.0 / sqrt(maxvals.get (i)));
         Dcoeffs.add (new MatrixCell(i, i, val));
      }
      
      SparseMatrixNd D = MatrixCell.BuildSparseMatrixNd (freeDOFs, freeDOFs, Dcoeffs);

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
   
   //////////////////////////////////////
   // Internal Operations
   //////////////////////////////////////
   
   /**
    * ElasticShell.cpp :: elasticEnergy
    * 
    * Verified.
    */
   protected double elasticEnergy (
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
         VectorNd deriv = new VectorNd(9);
         MatrixNd hess = new MatrixNd(9, 9);
         
         result += mMat.stretchingEnergy (
            mMC, mModel, mRestState, f, 
            (derivative != null) ? deriv : null, 
            (hessian != null) ? hess : null
         );
         
         if (derivative != null) {
            VectorNd deriv_seg = new VectorNd(3);
            VectorNd derivative_seg = new VectorNd(3);
            VectorNd seg_sum = new VectorNd(3);
            
            for (int j = 0; j < 3; j++) {               
               deriv.getSubVector (3*j, deriv_seg);
               derivative.getSubVector (3*mMC.F[f][j], derivative_seg);
               
               seg_sum.add (deriv_seg, derivative_seg);
               derivative.setSubVector (3*mMC.F[f][j], seg_sum);
            }
         }
         
         if (hessian != null) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    for (int l = 0; l < 3; l++) {
                        for (int m = 0; m < 3; m++) {
                           hessian.add (new MatrixCell (
                              3*mMC.F[f][j] + l, 
                              3*mMC.F[f][k] + m,
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
         
         VectorNd deriv = new VectorNd(18 + 3 * nedgedofs);
         MatrixNd hess = new MatrixNd(18 + 3 * nedgedofs, 18 + 3 * nedgedofs);
         
         result += this.mMat.bendingEnergy (mModel, mEdgeDOFs, mMC, mRestState, face, f, nedgedofs, 
            (derivative != null) ? deriv : null, 
            (hessian != null) ? hess : null);
         
         if (derivative != null) {
            VectorNd deriv_seg = new VectorNd(3);
            VectorNd derivative_seg = new VectorNd(3);
            VectorNd seg_sum = new VectorNd(3);
            
            for (int j = 0; j < 3; j++) {               
               deriv.getSubVector (3*j, deriv_seg);
               derivative.getSubVector (3*mMC.F[f][j], derivative_seg);
               
               seg_sum.add (deriv_seg, derivative_seg);
               derivative.setSubVector (3*mMC.F[f][j], seg_sum);
               
               // Get opposite node index.
               int oppidx = mMC.vertexOppositeFaceEdge (f, j);
               
               if (oppidx != -1) {
                  deriv.getSubVector(9+3*j, deriv_seg);
                  derivative.getSubVector (3*oppidx, derivative_seg);
                  
                  seg_sum.add (deriv_seg, derivative_seg);
                  derivative.setSubVector (3*oppidx, seg_sum);
               }
               
               for (int k = 0; k < nedgedofs; k++) {
                  int derivative_idx = 3 * nNodes + nedgedofs * mMC.FE[f][j] + k;
                  double derivative_i = derivative.get (derivative_idx);
                  
                  derivative_i += deriv.get (18 + nedgedofs * j + k);
                  derivative.set (derivative_idx, derivative_i);
               }
            }
         }
         
         if (hessian != null) {
            for (int j = 0; j < 3; j++) {
               int oppidxj = mMC.vertexOppositeFaceEdge(f, j);
               
               for (int k = 0; k < 3; k++) {
                  int oppidxk = mMC.vertexOppositeFaceEdge(f, k);
                  
                  for (int l = 0; l < 3; l++) {
                     for (int m = 0; m < 3; m++) { 
                        
                        hessian.add (new MatrixCell (
                           3*mMC.F[f][j]+l, 3*mMC.F[f][k]+m, hess.get(3*j+l, 3*k+m)));
                        
                        if (oppidxk != -1) {
                           hessian.add (new MatrixCell (
                              3*mMC.F[f][j]+l, 3*oppidxk+m, hess.get (3*j+l,9+3*k+m)));
                        }
                                                
                        if (oppidxj != -1) {
                           hessian.add (new MatrixCell (
                              3*oppidxj+l, 3*mMC.F[f][k]+m, hess.get (9+3*j+l,3*k+m)));
                        }
                        
                        if (oppidxj != -1 && oppidxk != -1) {
                           hessian.add (new MatrixCell (
                              3*oppidxj+l, 3*oppidxk+m, hess.get (9+3*j+l, 9+3*k+m)));
                        }
                     }
                     
                     // matches
                     
                     for (int m = 0; m < nedgedofs; m++) {
                        hessian.add (new MatrixCell (
                           3*mMC.F[f][j]+l, 3*nNodes+nedgedofs * mMC.FE[f][k] + m, 
                           hess.get (3*j+l, 18+nedgedofs*k+m)
                        ));
                        
                        hessian.add (new MatrixCell (
                           3*nNodes+nedgedofs*mMC.FE[f][k]+m, 3*mMC.F[f][j]+l, 
                           hess.get (18+nedgedofs*k+m, 3*j+l)
                        ));
                        
                        if (oppidxj != -1) {
                           hessian.add (new MatrixCell (
                              3*oppidxj+l,3*nNodes+nedgedofs*mMC.FE[f][k]+m, 
                              hess.get(9+3*j+l, 18+nedgedofs*k+m)
                           ));
                           
                           hessian.add (new MatrixCell (
                              3*nNodes+nedgedofs*mMC.FE[f][k]+m, 3*oppidxj+l,
                              hess.get(18+nedgedofs*k+m, 9+3*j+l)
                           ));
                        }
                     }
                  }
              
                  for (int m = 0; m < nedgedofs; m++) {
                     for (int ni = 0; ni < nedgedofs; ni++) {
                        hessian.add (new MatrixCell (
                           3*nNodes+nedgedofs*mMC.FE[f][j]+m, 3*nNodes+nedgedofs*mMC.FE[f][k]+ni, 
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

   //////////////////////////////////////
   // IO Operations
   //////////////////////////////////////
   
   public void saveState(String fp) {
      ArrayList<MatrixNd> faceStates = new ArrayList<MatrixNd>();
      
      for (int f = 0; f < mMesh.numFaces (); f++) {
         Matrix2d I = getI (f);
         Matrix2d II = getII (f);
         
         MatrixNd faceState = new MatrixNd(2,4);
         faceState.setSubMatrix (0, 0, I);
         faceState.setSubMatrix (0, 2, II);
         
         faceStates.add (faceState);
      }
      
      try {
         ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream(fp));
         oos.writeObject (faceStates); 
         oos.close ();
      } catch (IOException ex) {
         throw new RuntimeException(ex);
      }
   }
   
   @SuppressWarnings("unchecked")
   public void loadRestState(String fp) {
      ArrayList<MatrixNd> faceStates = null;
      
      try {
         ObjectInputStream oos = new ObjectInputStream(new FileInputStream(fp));
         faceStates = (ArrayList<MatrixNd>) oos.readObject ();
         oos.close ();
      } catch (IOException ex) {
         throw new RuntimeException(ex);
      } catch (ClassNotFoundException ex) {
         throw new RuntimeException(ex);
      }
      
      for (int f = 0; f < mMesh.numFaces (); f++) {
         Matrix2d I = new Matrix2d();
         Matrix2d II = new Matrix2d();
         
         faceStates.get (f).getSubMatrix (0, 0, I);
         faceStates.get (f).getSubMatrix (0, 2, II);
         
//         mRestState.abars.get (f).set (I);
         mRestState.bbars.get (f).set (II);
      }
   }
   
   //////////////////////////////////////
   // Debug
   //////////////////////////////////////
   
   public Matrix2d getI(int f) {
      return GeometryDerivative.firstFundamentalForm (mMC, mModel, f, null, null);
   }
   
   public Matrix2d getII(int f) {
      Matrix2d b = MidedgeAngleTanFormulation.secondFundamentalForm (
         mModel, 
         mEdgeDOFs,
         mMC,
         mMesh.getFace (f),  
         null, null);
      return b;
   }

   public Vector3d getFaceNormal(int f, int edgeIdx) {
      return GeometryDerivative.faceNormal (mMC, mModel, mMesh.getFace (f), edgeIdx, null, null);
   }
}

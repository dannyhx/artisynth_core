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
import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.core.materials.LinearMaterial;
import artisynth.demos.growth.models.ts.ThinShellBase;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SparseMatrixNd;
import maspack.matrix.SparseNumberedBlockMatrix;
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
   
   /** Previous node positions. Used to calculate instantaneous velocity. */
   protected double[] mPrevMeshDOFs; 
   
   /** Nodes that represent the edges of the mesh. Used to store the stiffness of the edges. */
   protected FemNode3d[] mEdgeDelegates;
   
   protected SparseNumberedBlockMatrix mS;
   protected double[] mSRowMaxs;
   
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
     
     for (int n = 0; n < mModel.numNodes (); n++) {
        FemNode3d node = mModel.getNode (n);
        node.setSolveIndex (n);
     }
     
     mEdgeDelegates = new FemNode3d[mMC.numEdges()];
     for (int e = 0; e < mEdgeDelegates.length; e++) {
        FemNode3d edgeDelegate = new FemNode3d();
        mEdgeDelegates[e] = edgeDelegate;
        edgeDelegate.setSolveIndex (mModel.numNodes () + e);
     }
     
     mSRowMaxs = new double[mModel.numNodes ()];
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
      clearStiffness();
      
      syncPrevMeshDOFs ();
      
      // StaticSolve.h::takeOneStep
      
      VectorNd derivative = new VectorNd();
      ArrayList<MatrixCell> hessian = new ArrayList<MatrixCell>();
      
      double energy = this.elasticEnergy (derivative, hessian);
      
      int freeDOFs = derivative.size ();
      
//      SparseMatrixNd H = MatrixCell.BuildSparseMatrixNd(freeDOFs, freeDOFs, hessian);
      assembleStiffness ();
      
      VectorNd force = new VectorNd(derivative);
      force.negate ();
      
//      SparseMatrixNd sI = new SparseMatrixNd(freeDOFs, freeDOFs);
//      sI.setIdentity ();
//      sI.scale (mReg);
//      H.add (sI);
      Matrix3d sI = new Matrix3d();
      sI.setIdentity ();
      sI.scale (mReg);
      int nNodesEdges = mModel.numNodes () + mEdgeDelegates.length; 
      for (int i = 0; i < nNodesEdges; i++) {
         mS.getBlock (i, i).add (sI);
      }
      
//      VectorNd maxvals = new VectorNd(freeDOFs);
//      maxvals.setZero ();
//      // For each row
//      for (int r = 0; r < freeDOFs; r++) {
//         SparseMatrixCell row = H.getRow (r);
//         if (row == null) {
//            continue;
//         }
//         
//         // For each column of row.
//         for (SparseMatrixCell cell = row; cell != null; cell = cell.next) {
//            maxvals.set (r, max(maxvals.get (r), abs(cell.value)));
//         }
//      }
//      
//      ArrayList<MatrixCell> Dcoeffs = new ArrayList<MatrixCell>();
//      for (int i = 0; i < freeDOFs; i++) {
//         double val = (maxvals.get (i) == 0.0 ? 1.0 : 1.0 / sqrt(maxvals.get (i)));
//         Dcoeffs.add (new MatrixCell(i, i, val));
//      }
//      
//      SparseMatrixNd D = MatrixCell.BuildSparseMatrixNd (freeDOFs, freeDOFs, Dcoeffs);
      double[] SRowMaxs = calcRowMaxs();
      for (int i = 0; i < nNodesEdges; i++) {
         SRowMaxs[i] = (SRowMaxs[i] == 0.0 ? 1.0 : 1.0 / sqrt(SRowMaxs[i]));
      }
      
//      SparseMatrixNd DHDT = new SparseMatrixNd(freeDOFs, freeDOFs);
//      DHDT.set(D);
//      DHDT.mul (H);
//      DHDT.mulTranspose (D);
      mulDiagBySparse (SRowMaxs, mS);
      mulSparseByDiag (mS, SRowMaxs);
      
      VectorNd rhs = new VectorNd();
      rhs.mul (D, force);
      
      VectorNd x = new VectorNd(rhs.size ());
      
      PardisoSolver solver = new PardisoSolver();
//      solver.analyze (DHDT, freeDOFs, Matrix.POSITIVE_DEFINITE);
      solver.analyze (mS, freeDOFs, Matrix.POSITIVE_DEFINITE);
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
   // Public Read Operations
   //////////////////////////////////////
   
   public double getEnergy(double timestep) {
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode (n);
         
         Point3d pos = node.getPosition ();
         Point3d prevPos = new Point3d(
            mPrevMeshDOFs[n*3 + 0],
            mPrevMeshDOFs[n*3 + 1], 
            mPrevMeshDOFs[n*3 + 2]);
         
         Vector3d velo = new Vector3d();
         velo.sub (pos, prevPos);
         velo.scale (1/timestep);
         
         node.setVelocity (velo);
      }
      
      double energy = mModel.getEnergy ();
      
      for (FemNode3d node : mModel.getNodes()) {
         node.getVelocity ().setZero ();
      }
      
      return energy;
   }
   
   //////////////////////////////////////
   // Internal Operations
   //////////////////////////////////////
   
   protected void syncPrevMeshDOFs() {
      if (mPrevMeshDOFs == null) {
         mPrevMeshDOFs = new double[mModel.numNodes () * 3];
      }
      
      for (int n = 0; n < mModel.numNodes (); n++) {
         Point3d pos = mModel.getNode (n).getPosition ();
         
         mPrevMeshDOFs[n*3 + 0] = pos.x;
         mPrevMeshDOFs[n*3 + 1] = pos.y;
         mPrevMeshDOFs[n*3 + 2] = pos.z;
      }
   }
   
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
            Matrix3d hess_block = new Matrix3d();
            
            for (int j = 0; j < 3; j++) {
               FemNode3d nodej = mModel.getNode (mMC.F[f][j]);
                for (int k = 0; k < 3; k++) {
                   FemNode3d nodek = mModel.getNode (mMC.F[f][k]);
                   
                   hess.getSubMatrix (3*j, 3*k, hess_block);
                   nodej.getNodeNeighbor (nodek).getK00 ().add (hess_block);
//                    for (int l = 0; l < 3; l++) {
//                        for (int m = 0; m < 3; m++) {
//                           hessian.add (new MatrixCell (
//                              3*mMC.F[f][j] + l, 
//                              3*mMC.F[f][k] + m,
//                              hess.get (3*j+l, 3*k+m)
//                           ));
//                        }
//                    }
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
            Matrix3d hess_block = new Matrix3d();
            
            for (int j = 0; j < 3; j++) {
               int oppidxj = mMC.vertexOppositeFaceEdge(f, j);
               
               FemNode3d nodej = mModel.getNode (mMC.F[f][j]);
               FemNode3d nodejopp = mModel.getNode (oppidxj);
               
               for (int k = 0; k < 3; k++) {
                  int oppidxk = mMC.vertexOppositeFaceEdge(f, k);
                  
                  FemNode3d nodek = mModel.getNode (mMC.F[f][k]);
                  FemNode3d nodekopp = mModel.getNode (oppidxk);
                  
                  hess.getSubMatrix (3*j, 3*k, hess_block);
                  
                  // Node-Node stiffness
                  
                  nodej.getNodeNeighbor (nodek).getK00 ().add (hess_block);;
                  
                  if (oppidxk != -1) {
                     FemNodeNeighbor neigh = nodej.getIndirectNeighbor (nodekopp);
                     if (neigh == null) {
                        neigh = nodej.addIndirectNeighbor (nodekopp);
                     }
                     neigh.getK00 ().add (hess_block);
                  }
                  
                  if (oppidxj != -1) {
                     FemNodeNeighbor neigh = nodejopp.getIndirectNeighbor (nodek);
                     if (neigh == null) {
                        neigh = nodejopp.addIndirectNeighbor (nodek);
                     }
                     neigh.getK00 ().add (hess_block);
                  }
                  
                  if (oppidxj != -1 && oppidxk != -1) {
                     FemNodeNeighbor neigh = nodejopp.getIndirectNeighbor (nodekopp);
                     if (neigh == null) {
                        neigh = nodejopp.addIndirectNeighbor (nodekopp);
                     }
                     neigh.getK00 ().add (hess_block);
                  }
                  
                  // Node-Edge stiffness
                  
                  FemNode3d edgeDelegate_k = mEdgeDelegates[mMC.FE[f][k]];
                  FemNodeNeighbor neigh = nodej.getIndirectNeighbor (edgeDelegate_k);
                  if (neigh == null) {
                     neigh = nodej.addIndirectNeighbor (edgeDelegate_k);
                  }
                  hess.getSubMatrix (3*j, 18+nedgedofs*k, hess_block);
                  neigh.getK00 ().add (hess_block);
                  //
                  neigh = edgeDelegate_k.getIndirectNeighbor (nodej);
                  if (neigh == null) {
                     neigh = edgeDelegate_k.addIndirectNeighbor (nodej);
                  }
                  hess.getSubMatrix (18+nedgedofs*k, 3*j, hess_block);
                  neigh.getK00 ().add (hess_block);
                  //
                  if (oppidxj != -1) {
                     neigh = nodejopp.getIndirectNeighbor (edgeDelegate_k);
                     if (neigh == null) {
                        neigh = nodejopp.addIndirectNeighbor (edgeDelegate_k);
                     }
                     hess.getSubMatrix (9+3*j, 18+nedgedofs*k, hess_block);
                     neigh.getK00 ().add (hess_block);
                     //
                     neigh = edgeDelegate_k.getIndirectNeighbor (nodejopp);
                     if (neigh == null) {
                        neigh = edgeDelegate_k.addIndirectNeighbor (nodejopp);
                     }
                     hess.getSubMatrix (18+nedgedofs*k, 9+3*j, hess_block);
                     neigh.getK00 ().add (hess_block);
                  }
                  
                  // Edge-Edge stiffness
                  
                  FemNode3d edgeDelegate_j = mEdgeDelegates[mMC.FE[f][j]];
                  neigh = edgeDelegate_j.getIndirectNeighbor (edgeDelegate_k);
                  if (neigh == null) {
                     neigh = edgeDelegate_j.addIndirectNeighbor (edgeDelegate_k);
                  }
                  hess.getSubMatrix (18 + nedgedofs * j, 18 + nedgedofs * k, hess_block);
                  neigh.getK00 ().add (hess_block);
                  
//                  for (int l = 0; l < 3; l++) {
//                     for (int m = 0; m < 3; m++) { 
//                        
//                        hessian.add (new MatrixCell (
//                           3*mMC.F[f][j]+l, 3*mMC.F[f][k]+m, hess.get(3*j+l, 3*k+m)));
//                        
//                        if (oppidxk != -1) {
//                           hessian.add (new MatrixCell (
//                              3*mMC.F[f][j]+l, 3*oppidxk+m, hess.get (3*j+l,9+3*k+m)));
//                        }
//                                                
//                        if (oppidxj != -1) {
//                           hessian.add (new MatrixCell (
//                              3*oppidxj+l, 3*mMC.F[f][k]+m, hess.get (9+3*j+l,3*k+m)));
//                        }
//                        
//                        if (oppidxj != -1 && oppidxk != -1) {
//                           hessian.add (new MatrixCell (
//                              3*oppidxj+l, 3*oppidxk+m, hess.get (9+3*j+l, 9+3*k+m)));
//                        }
//                     }
//                     
//                     for (int m = 0; m < nedgedofs; m++) {
//                        hessian.add (new MatrixCell (
//                           3*mMC.F[f][j]+l, 3*nNodes+nedgedofs * mMC.FE[f][k] + m, 
//                           hess.get (3*j+l, 18+nedgedofs*k+m)
//                        ));
//                        
//                        hessian.add (new MatrixCell (
//                           3*nNodes+nedgedofs*mMC.FE[f][k]+m, 3*mMC.F[f][j]+l, 
//                           hess.get (18+nedgedofs*k+m, 3*j+l)
//                        ));
//                        
//                        if (oppidxj != -1) {
//                           hessian.add (new MatrixCell (
//                              3*oppidxj+l,3*nNodes+nedgedofs*mMC.FE[f][k]+m, 
//                              hess.get(9+3*j+l, 18+nedgedofs*k+m)
//                           ));
//                           
//                           hessian.add (new MatrixCell (
//                              3*nNodes+nedgedofs*mMC.FE[f][k]+m, 3*oppidxj+l,
//                              hess.get(18+nedgedofs*k+m, 9+3*j+l)
//                           ));
//                        }
//
//                  for (int m = 0; m < nedgedofs; m++) {
//                     for (int ni = 0; ni < nedgedofs; ni++) {
//                        hessian.add (new MatrixCell (
//                           3*nNodes+nedgedofs*mMC.FE[f][j]+m, 3*nNodes+nedgedofs*mMC.FE[f][k]+ni, 
//                           hess.get (18 + nedgedofs * j + m, 18 + nedgedofs * k + ni)
//                        ));
//                     }
//                  }
               }
              
            }
         }
      }
      
      return result;
   }

   //////////////////////////////////////
   // Stiffness
   //////////////////////////////////////
   
   public void clearStiffness() {
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = mS.getBlockByNumber (b);
            B.setZero ();
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = mS.getBlockByNumber (b);
            B.setZero ();
         }        
      }
      
      for (int e = 0; e < mEdgeDelegates.length; e++) {
         FemNode3d edgeDelegate = mEdgeDelegates[e];
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = mS.getBlockByNumber (b);
            B.setZero ();
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = mS.getBlockByNumber (b);
            B.setZero ();
         }        
      }
      
      for (int i = 0; i < mSRowMaxs.length; i++) {
         mSRowMaxs[i] = 0;
      }
   }
   
   public void assembleStiffness() {
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            nbr.addSolveBlocks (mS, node);
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            nbr.addSolveBlocks (mS, node);
         }        
      }
      
      for (int e = 0; e < mEdgeDelegates.length; e++) {
         FemNode3d edgeDelegate = mEdgeDelegates[e];
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            nbr.addSolveBlocks (mS, edgeDelegate);
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            nbr.addSolveBlocks (mS, edgeDelegate);
         }        
      }
   }
   
   protected void updateRowMaxsByBlock(int ni, int nj, MatrixBlock B) {
      for (int r = 0; r < 3; r++) {
         for (int c = 0; c < 3; c++) {
            double v = B.get (r, c);
            mSRowMaxs[3*ni + c] = max(mSRowMaxs[3*ni + c], abs(v));
         }
      }
   }
   
   public double[] calcRowMaxs() {
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            updateRowMaxsByBlock(n, nb, mS.getBlock (n, nb));
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            updateRowMaxsByBlock(n, nb, mS.getBlock (n, nb));
         }        
      }

      for (int e = 0; e < mEdgeDelegates.length; e++) {
         FemNode3d edgeDelegate = mEdgeDelegates[e];
         int ed = edgeDelegate.getSolveIndex();
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            updateRowMaxsByBlock(ed, nb, mS.getBlock (ed, nb));
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            updateRowMaxsByBlock(ed, nb, mS.getBlock (ed, nb));
         }        
      }
      
      return mSRowMaxs;
   }
   
   protected void mulDiagByBlock(int ni, int nj, MatrixBlock B, double[] D) {
      for (int r = 0; r < 3; r++) {
         for (int c = 0; c < 3; c++) {
            B.set (r, c, B.get (r, c) * D[3*ni+r]);
         }
      }
   }
   
   public void mulDiagBySparse(double[] D, SparseNumberedBlockMatrix S) {
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulDiagByBlock(n, nb, mS.getBlock (n, nb), D);
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulDiagByBlock(n, nb, mS.getBlock (n, nb), D);
         }        
      }

      for (int e = 0; e < mEdgeDelegates.length; e++) {
         FemNode3d edgeDelegate = mEdgeDelegates[e];
         int ed = edgeDelegate.getSolveIndex();
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulDiagByBlock(ed, nb, mS.getBlock (ed, nb), D);
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulDiagByBlock(ed, nb, mS.getBlock (ed, nb), D);
         }        
      }
   }
   
   protected void mulBlockByDiag(int ni, int nj, MatrixBlock B, double[] D) {
      for (int r = 0; r < 3; r++) {
         for (int c = 0; c < 3; c++) {
            B.set (r, c, B.get (r, c) * D[3*nj+c]);
         }
      }
   }
   
   public void mulSparseByDiag(SparseNumberedBlockMatrix S, double[] D) {
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulBlockByDiag(n, nb, mS.getBlock (n, nb), D);
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulBlockByDiag(n, nb, mS.getBlock (n, nb), D);
         }        
      }

      for (int e = 0; e < mEdgeDelegates.length; e++) {
         FemNode3d edgeDelegate = mEdgeDelegates[e];
         int ed = edgeDelegate.getSolveIndex();
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulBlockByDiag(ed, nb, mS.getBlock (ed, nb), D);
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            mulBlockByDiag(ed, nb, mS.getBlock (ed, nb), D);
         }        
      }
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

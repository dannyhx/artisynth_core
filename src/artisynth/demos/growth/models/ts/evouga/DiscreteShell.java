package artisynth.demos.growth.models.ts.evouga;

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
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.models.ts.ThinShellBase;
import artisynth.demos.growth.util.MathUtil;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix1x1;
import maspack.matrix.Matrix1x1Block;
import maspack.matrix.Matrix1x3;
import maspack.matrix.Matrix1x3Block;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x1;
import maspack.matrix.Matrix3x1Block;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.solvers.PardisoSolver;
import maspack.util.FunctionTimer;

/**
 * Implementation of Discrete Shells where DoFs are expressed in terms of nodal
 * positions and surface curvature (e.g. 2nd fundamental form).
 * 
 * See https://github.com/evouga/libshell See LICENSE
 */
public class DiscreteShell extends ThinShellBase {

   protected MonolayerRestState mRestState;
   protected DiscreteShellMaterial mMat;

   /**
    * Container for the edge DoFs. Size = NumEdges * mNumExtraDOFs. Initialized
    * to zero.
    */
   protected VectorNd mEdgeDOFs;

   /** Number of degrees of freedom per edge. MidedgeAngleTanFormulation.h */
   protected final int mNumExtraDOFs = 1;

   /** Mesh Topology. */
   protected MeshConnectivity mMC = null;

   /**
    * Scalar of identity matrix that gets added with stiffness matrix. Default:
    * 1e-6.
    */
   public double mReg = 0.025;

   /** Previous node positions. Used to calculate instantaneous velocity. */
   protected double[] mPrevMeshDOFs;

   /** Nodes that represent the edges of the mesh. */
   protected FemNode3d[] mEdgeDelegates;

   /** Global stiffness matrix (i.e. hessian). */
   protected SparseNumberedBlockMatrix mS;

   /** Diagonal vector that can operate on mS. */
   protected VectorNd mD;

   /** Is the first frame being computed? */
   protected boolean mIsFirstStep = true;

   protected FunctionTimer mTimer = new FunctionTimer ();

   /**
    * If false, then only populate the upper-triangular half of mS. Leave as
    * true until fix SparseBlockMatrix.doGetBlockCRSIndices()
    */
   protected boolean mIsEntireHessian = true;

   public DiscreteShell (FemModel3d model, PolygonalMesh mesh) {
      super (model, mesh);

      double thickness = model.getShellElement (0).getDefaultThickness (); // 1e-1
      double poissonRatio =
         ((LinearMaterial)model.getMaterial ()).getPoissonsRatio (); // 1/2

      this.setMaterialProperties (1.0, poissonRatio, thickness);

      mMC = new MeshConnectivity (mesh);

      // main.cpp::runSimulation()

      // Initial rest geometry of shell.
      mRestState = new MonolayerRestState (mMC, model, thickness);

      for (int f = 0; f < this.mModel.numShellElements (); f++) {
         // Initialize first fundamental form of the face.
         Matrix2d I =
            GeometryDerivative
               .firstFundamentalForm (mMC, mModel, f, null, null);
         mRestState.abars.set (f, I);

         // Assuming the model is a 2D plane initially, assume its principle
         // fiber
         // direction is mMC.d. To follow, calculate the orthogonal direction of
         // mMC.d
         // while using the first fundamental form as the basis. Both vectors
         // are stored in mMC.T.

         Vector2d dI = new Vector2d ();
         dI.set (mMC.d);
         I.mul (dI); // Input: d, Output: dI

         // Assume dO.x is 1

         double dOy = dI.x / -dI.y;

         Vector2d dO = new Vector2d (1, dOy);

         mMC.T[f].setColumns (mMC.d, dO);
         if (!mMC.T[f].invert ()) {
            throw new RuntimeException ();
         }
      }

      // Initialize second fundamental forms to rest flat.
      for (int i = 0; i < mRestState.bbars.size (); i++) {
         mRestState.bbars.get (i).setZero ();
      }

      // MidedgeAngleTanForumulation.cpp::initializeExtraDOFs
      mEdgeDOFs = new VectorNd (mMC.numEdges () * mNumExtraDOFs);

      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode (n);
         node.setSolveIndex (n);
      }

      mEdgeDelegates = new FemNode3d[mMC.numEdges ()];
      for (int d = 0; d < mEdgeDelegates.length; d++) {
         FemNode3d edgeDelegate = new GrowNode3d (false);
         mEdgeDelegates[d] = edgeDelegate;
         edgeDelegate.setSolveIndex (mModel.numNodes () + d);
         edgeDelegate.setFlag (StiffnessMatrixUtil.EDGE_DELEGATE_FLAG);
      }

      int numBlockDofs = numNodesAndEdges ();
      int[] rowSz = new int[numBlockDofs];
      int[] colSz = new int[numBlockDofs];
      for (int i = 0; i < numBlockDofs; i++) {
         int dof = (i < mModel.numNodes ()) ? 3 : mNumExtraDOFs;

         rowSz[i] = dof;
         colSz[i] = dof;
      }

      mS = new SparseNumberedBlockMatrix (rowSz, colSz);
      mD = new VectorNd (numFreeDoFs ());
   }

   //////////////////////////////////////
   // Public Setup Operations
   //////////////////////////////////////

   @Override
   public void setMaterialProperties (
      double youngsModulus, double poissonsRatio, double thickness) {
      this.mMat = new StVKMaterial (youngsModulus, poissonsRatio);
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
   public void setBarycentricRestII (Matrix2d M) {
      for (int f = 0; f < mRestState.bbars.size (); f++) {
         Matrix2d T = mMC.T[f];

         Matrix2d Tinv = new Matrix2d (T);
         if (!Tinv.invert ()) {
            throw new RuntimeException ();
         }

         Matrix2d a0_hb0 = new Matrix2d ();
         a0_hb0.set (mRestState.abars.get (f));
         // a0_hb0.sub (b); // Assume flat curvature basis.

         Matrix2d g_pos = new Matrix2d ();
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
   public void advance () {

      if (!mIsFirstStep) {
         StiffnessMatrixUtil.clearStiffness (mModel, mEdgeDelegates, mS);
         mD.setZero ();
      }

      syncPrevMeshDOFs ();

      // StaticSolve.h::takeOneStep

      VectorNd derivative = new VectorNd (mD.size ());
      ArrayList<MatrixCell> hessian = new ArrayList<MatrixCell> ();

      mTimer.reset ();
      mTimer.start ();

      double energy = this.elasticEnergy (derivative, hessian);

      mTimer.stop ();
      System.out.println ("Elastic Energy: " + mTimer.result (1));
      mTimer.reset ();

      System.out.println ("I: " + GeometryDerivative.mTimerI.result (1));
      GeometryDerivative.mTimerI.reset ();

      System.out
         .println ("II: " + MidedgeAngleTanFormulation.mTimerII.result (1));
      MidedgeAngleTanFormulation.mTimerII.reset ();

      System.out.println ("Stretch " + StVKMaterial.mTimerStretch.result (1));
      StVKMaterial.mTimerStretch.reset ();

      System.out.println ("Bending " + StVKMaterial.mTimerBending.result (1));
      StVKMaterial.mTimerBending.reset ();

      VectorNd force = new VectorNd (derivative);
      force.negate ();

      Matrix3d sI = new Matrix3d ();
      sI.setIdentity ();
      sI.scale (mReg);
      for (int i = 0; i < numNodesAndEdges (); i++) {
         if (i < mModel.numNodes ()) {
            mS.getBlock (i, i).add (sI);
         }
         else {
            ((Matrix1x1)mS.getBlock (i, i)).m00 += mReg;
         }
      }

      StiffnessMatrixUtil.getRowMaxs (mModel, mEdgeDelegates, mS, mD);

      double[] mD_ = mD.getBuffer ();
      for (int i = 0; i < mD.size (); i++) {
         mD_[i] = (mD_[i] == 0.0) ? 1.0 : 1.0 / sqrt (mD_[i]);
      }

      StiffnessMatrixUtil
         .mulDiagBySparse (mD, mModel, mEdgeDelegates, mS, true);
      StiffnessMatrixUtil
         .mulDiagBySparse (mD, mModel, mEdgeDelegates, mS, false);

      VectorNd rhs = new VectorNd (force.size ());
      MathUtil.mulDiagVecByVec (mD, force, rhs);

      VectorNd x = new VectorNd (rhs.size ());
      PardisoSolver solver = new PardisoSolver ();
      solver.analyze (mS, numFreeDoFs (), Matrix.SPD);

      mTimer.start ();

      solver.factor ();
      solver.solve (x, rhs);

      mTimer.stop ();
      System.out.println ("Solve: " + mTimer.result (1));

      solver.dispose ();

      VectorNd descentDir = new VectorNd (x.size ());
      MathUtil.mulDiagVecByVec (mD, x, descentDir);

      // Update node positions.
      Point3d newPos = new Point3d ();
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode (n);

         descentDir.getSubVector (3 * n, newPos);
         newPos.add (node.getPosition ());

         node.setPosition (newPos);

         // Update surface mesh
         mModel.getSurfaceVertex (node).setPosition (newPos);
      }

      // Update edge DoFs.
      VectorNd descentDir_seg = new VectorNd (mEdgeDOFs.size ());
      descentDir.getSubVector (3 * mModel.numNodes (), descentDir_seg);
      mEdgeDOFs.add (descentDir_seg);

      mIsFirstStep = false;
   }

   //////////////////////////////////////
   // Public Read Operations
   //////////////////////////////////////

   public double getEnergy (double timestep) {
      for (int n = 0; n < mModel.numNodes (); n++) {
         FemNode3d node = mModel.getNode (n);

         Point3d pos = node.getPosition ();
         Point3d prevPos =
            new Point3d (
               mPrevMeshDOFs[n * 3 + 0], mPrevMeshDOFs[n * 3 + 1],
               mPrevMeshDOFs[n * 3 + 2]);

         Vector3d velo = new Vector3d ();
         velo.sub (pos, prevPos);
         velo.scale (1 / timestep);

         node.setVelocity (velo);
      }

      double energy = mModel.getEnergy ();

      for (FemNode3d node : mModel.getNodes ()) {
         node.getVelocity ().setZero ();
      }

      return energy;
   }

   //////////////////////////////////////
   // Internal Operations
   //////////////////////////////////////

   protected void syncPrevMeshDOFs () {
      if (mPrevMeshDOFs == null) {
         mPrevMeshDOFs = new double[mModel.numNodes () * 3];
      }

      for (int n = 0; n < mModel.numNodes (); n++) {
         Point3d pos = mModel.getNode (n).getPosition ();

         mPrevMeshDOFs[n * 3 + 0] = pos.x;
         mPrevMeshDOFs[n * 3 + 1] = pos.y;
         mPrevMeshDOFs[n * 3 + 2] = pos.z;
      }
   }

   /**
    * ElasticShell.cpp :: elasticEnergy
    * 
    * Verified.
    */
   protected double elasticEnergy (
      VectorNd derivative, ArrayList<MatrixCell> hessian) {
      int nNodes = this.mModel.numNodes ();

      if (derivative != null) {
         derivative.setZero ();
      }

      if (hessian != null) {
         hessian.clear ();
      }

      double result = 0;

      // Stretching

      // mTimerSB.start ();

      for (int f = 0; f < mMesh.numFaces (); f++) {
         VectorNd deriv = new VectorNd (9);
         MatrixNd hess = new MatrixNd (9, 9);

         result +=
            mMat
               .stretchingEnergy (
                  mMC, mModel, mRestState, f,
                  (derivative != null) ? deriv : null,
                  (hessian != null) ? hess : null);

         if (derivative != null) {
            VectorNd deriv_seg = new VectorNd (3);
            VectorNd derivative_seg = new VectorNd (3);
            VectorNd seg_sum = new VectorNd (3);

            for (int j = 0; j < 3; j++) {
               deriv.getSubVector (3 * j, deriv_seg);
               derivative.getSubVector (3 * mMC.F[f][j], derivative_seg);

               seg_sum.add (deriv_seg, derivative_seg);
               derivative.setSubVector (3 * mMC.F[f][j], seg_sum);
            }
         }

         if (hessian != null) {
            Matrix3d hess_block = new Matrix3d ();

            for (int j = 0; j < 3; j++) {
               FemNode3d nodej = mModel.getNode (mMC.F[f][j]);
               for (int k = 0; k < 3; k++) {
                  FemNode3d nodek = mModel.getNode (mMC.F[f][k]);

                  if (mIsEntireHessian
                  || nodej.getSolveIndex () <= nodek.getSolveIndex ()) {
                     hess.getSubMatrix (3 * j, 3 * k, hess_block);
                     StiffnessMatrixUtil
                        .getDirectNeighborBlock (nodej, nodek, mS, true)
                        .add (hess_block);
                  }
               }
            }
         }
      }

      // Bending terms

      int nedgedofs = this.mNumExtraDOFs;

      for (int f = 0; f < mMesh.numFaces (); f++) {
         Face face = mMesh.getFace (f);

         VectorNd deriv = new VectorNd (18 + 3 * nedgedofs);
         MatrixNd hess = new MatrixNd (18 + 3 * nedgedofs, 18 + 3 * nedgedofs);

         result +=
            this.mMat
               .bendingEnergy (
                  mModel, mEdgeDOFs, mMC, mRestState, face, f, nedgedofs,
                  (derivative != null) ? deriv : null,
                  (hessian != null) ? hess : null);

         if (derivative != null) {
            VectorNd deriv_seg = new VectorNd (3);
            VectorNd derivative_seg = new VectorNd (3);
            VectorNd seg_sum = new VectorNd (3);

            for (int j = 0; j < 3; j++) {
               deriv.getSubVector (3 * j, deriv_seg);
               derivative.getSubVector (3 * mMC.F[f][j], derivative_seg);

               seg_sum.add (deriv_seg, derivative_seg);
               derivative.setSubVector (3 * mMC.F[f][j], seg_sum);

               // Get opposite node index.
               int oppidx = mMC.vertexOppositeFaceEdge (f, j);

               if (oppidx != -1) {
                  deriv.getSubVector (9 + 3 * j, deriv_seg);
                  derivative.getSubVector (3 * oppidx, derivative_seg);

                  seg_sum.add (deriv_seg, derivative_seg);
                  derivative.setSubVector (3 * oppidx, seg_sum);
               }

               for (int k = 0; k < nedgedofs; k++) {
                  int derivative_idx =
                     3 * nNodes + nedgedofs * mMC.FE[f][j] + k;
                  double derivative_i = derivative.get (derivative_idx);

                  derivative_i += deriv.get (18 + nedgedofs * j + k);
                  derivative.set (derivative_idx, derivative_i);
               }
            }
         }

         if (hessian != null) {
            Matrix3d hess_block_3x3 = new Matrix3d ();
            Matrix3x1 hess_block_3x1 = new Matrix3x1 (); // Node_Edge
            Matrix1x3 hess_block_1x3 = new Matrix1x3 (); // Edge_Node
            double hess_ele = 0; // Edge_Edge

            for (int j = 0; j < 3; j++) {
               int oppidxj = mMC.vertexOppositeFaceEdge (f, j);

               FemNode3d nodej = mModel.getNode (mMC.F[f][j]);
               FemNode3d nodejopp =
                  (oppidxj != -1) ? mModel.getNode (oppidxj) : null;

               for (int k = 0; k < 3; k++) {
                  int oppidxk = mMC.vertexOppositeFaceEdge (f, k);

                  FemNode3d nodek = mModel.getNode (mMC.F[f][k]);
                  FemNode3d nodekopp =
                     (oppidxk != -1) ? mModel.getNode (oppidxk) : null;

                  // Node-Node stiffness

                  if (mIsEntireHessian
                  || nodej.getSolveIndex () <= nodek.getSolveIndex ()) {
                     hess.getSubMatrix (3 * j, 3 * k, hess_block_3x3);
                     StiffnessMatrixUtil
                        .getDirectNeighborBlock (nodej, nodek, mS, true)
                        .add (hess_block_3x3);
                  }

                  if (oppidxk != -1 && (mIsEntireHessian
                  || nodej.getSolveIndex () <= nodekopp.getSolveIndex ())) {
                     hess.getSubMatrix (3 * j, 9 + 3 * k, hess_block_3x3);
                     StiffnessMatrixUtil
                        .getIndirectNeighborBlock (nodej, nodekopp, mS)
                        .add (hess_block_3x3);
                  }

                  if (oppidxj != -1 && (mIsEntireHessian
                  || nodejopp.getSolveIndex () <= nodek.getSolveIndex ())) {
                     hess.getSubMatrix (9 + 3 * j, 3 * k, hess_block_3x3);
                     StiffnessMatrixUtil
                        .getIndirectNeighborBlock (nodejopp, nodek, mS)
                        .add (hess_block_3x3);
                  }

                  if (oppidxj != -1 && oppidxk != -1 && (mIsEntireHessian
                  || nodejopp.getSolveIndex () <= nodekopp.getSolveIndex ())) {
                     hess.getSubMatrix (9 + 3 * j, 9 + 3 * k, hess_block_3x3);
                     StiffnessMatrixUtil
                        .getIndirectNeighborBlock (nodejopp, nodekopp, mS)
                        .add (hess_block_3x3);
                  }

                  // Node-Edge stiffness

                  FemNode3d edgeDelegate_k = mEdgeDelegates[mMC.FE[f][k]];

                  if (mIsEntireHessian || nodej
                     .getSolveIndex () <= edgeDelegate_k.getSolveIndex ()) {
                     hess
                        .getSubMatrix (
                           3 * j, 18 + nedgedofs * k, hess_block_3x1); // 3x1
                     Matrix3x1 sb3x1 =
                        (Matrix3x1Block)StiffnessMatrixUtil
                           .getIndirectEdgeNeighborBlock (
                              nodej, edgeDelegate_k,
                              FemEdgeNeighborType.NODE_EDGE, mS);
                     sb3x1.m00 += hess_block_3x1.m00;
                     sb3x1.m10 += hess_block_3x1.m10;
                     sb3x1.m20 += hess_block_3x1.m20;
                  }

                  if (mIsEntireHessian || edgeDelegate_k
                     .getSolveIndex () <= nodej.getSolveIndex ()) {
                     hess
                        .getSubMatrix (
                           18 + nedgedofs * k, 3 * j, hess_block_1x3); // 1x3
                     Matrix1x3 sb1x3 =
                        (Matrix1x3Block)StiffnessMatrixUtil
                           .getIndirectEdgeNeighborBlock (
                              edgeDelegate_k, nodej,
                              FemEdgeNeighborType.EDGE_NODE, mS);
                     sb1x3.m00 += hess_block_1x3.m00;
                     sb1x3.m01 += hess_block_1x3.m01;
                     sb1x3.m02 += hess_block_1x3.m02;
                  }

                  if (oppidxj != -1) {
                     if (mIsEntireHessian || nodejopp
                        .getSolveIndex () <= edgeDelegate_k.getSolveIndex ()) {
                        hess
                           .getSubMatrix (
                              9 + 3 * j, 18 + nedgedofs * k, hess_block_3x1); // 3x1
                        Matrix3x1 sb3x1 =
                           (Matrix3x1Block)StiffnessMatrixUtil
                              .getIndirectEdgeNeighborBlock (
                                 nodejopp, edgeDelegate_k,
                                 FemEdgeNeighborType.NODE_EDGE, mS);
                        sb3x1.m00 += hess_block_3x1.m00;
                        sb3x1.m10 += hess_block_3x1.m10;
                        sb3x1.m20 += hess_block_3x1.m20;
                     }

                     if (mIsEntireHessian || edgeDelegate_k
                        .getSolveIndex () <= nodejopp.getSolveIndex ()) {
                        hess
                           .getSubMatrix (
                              18 + nedgedofs * k, 9 + 3 * j, hess_block_1x3); // 1x3
                        Matrix1x3 sb1x3 =
                           (Matrix1x3Block)StiffnessMatrixUtil
                              .getIndirectEdgeNeighborBlock (
                                 edgeDelegate_k, nodejopp,
                                 FemEdgeNeighborType.EDGE_NODE, mS);
                        sb1x3.m00 += hess_block_1x3.m00;
                        sb1x3.m01 += hess_block_1x3.m01;
                        sb1x3.m02 += hess_block_1x3.m02;
                     }
                  }

                  // Edge-Edge stiffness

                  FemNode3d edgeDelegate_j = mEdgeDelegates[mMC.FE[f][j]];

                  if (mIsEntireHessian || edgeDelegate_j
                     .getSolveIndex () <= edgeDelegate_k.getSolveIndex ()) {
                     hess_ele =
                        hess.get (18 + nedgedofs * j, 18 + nedgedofs * k); // 1x1
                     Matrix1x1 sb1x1 =
                        (Matrix1x1Block)StiffnessMatrixUtil
                           .getIndirectEdgeNeighborBlock (
                              edgeDelegate_j, edgeDelegate_k,
                              FemEdgeNeighborType.EDGE_EDGE, mS);
                     sb1x1.m00 += hess_ele;
                  }
               }
            }
         }
      }

      // mTimerSB.stop ();
      // System.out.println (mTimerSB.result (1));
      // mTimerSB.reset ();

      return result;
   }

   //////////////////////////////////////
   // Accessors
   //////////////////////////////////////

   protected int numNodesAndEdges () {
      return this.mModel.numNodes () + mMC.numEdges ();
   }

   protected int numFreeDoFs () {
      return 3 * this.mModel.numNodes () + mNumExtraDOFs * mMC.numEdges ();
   }

   //////////////////////////////////////
   // IO Operations
   //////////////////////////////////////

   public void saveState (String fp) {
      ArrayList<MatrixNd> faceStates = new ArrayList<MatrixNd> ();

      for (int f = 0; f < mMesh.numFaces (); f++) {
         Matrix2d I = getI (f);
         Matrix2d II = getII (f);

         MatrixNd faceState = new MatrixNd (2, 4);
         faceState.setSubMatrix (0, 0, I);
         faceState.setSubMatrix (0, 2, II);

         faceStates.add (faceState);
      }

      try {
         ObjectOutputStream oos =
            new ObjectOutputStream (new FileOutputStream (fp));
         oos.writeObject (faceStates);
         oos.close ();
      }
      catch (IOException ex) {
         throw new RuntimeException (ex);
      }
   }

   @SuppressWarnings("unchecked")
   public void loadRestState (String fp) {
      ArrayList<MatrixNd> faceStates = null;

      try {
         ObjectInputStream oos =
            new ObjectInputStream (new FileInputStream (fp));
         faceStates = (ArrayList<MatrixNd>)oos.readObject ();
         oos.close ();
      }
      catch (IOException ex) {
         throw new RuntimeException (ex);
      }
      catch (ClassNotFoundException ex) {
         throw new RuntimeException (ex);
      }

      for (int f = 0; f < mMesh.numFaces (); f++) {
         Matrix2d I = new Matrix2d ();
         Matrix2d II = new Matrix2d ();

         faceStates.get (f).getSubMatrix (0, 0, I);
         faceStates.get (f).getSubMatrix (0, 2, II);

         // mRestState.abars.get (f).set (I);
         mRestState.bbars.get (f).set (II);
      }
   }

   //////////////////////////////////////
   // Debug
   //////////////////////////////////////

   public Matrix2d getI (int f) {
      return GeometryDerivative
         .firstFundamentalForm (mMC, mModel, f, null, null);
   }

   public Matrix2d getII (int f) {
      Matrix2d b =
         MidedgeAngleTanFormulation
            .secondFundamentalForm (
               mModel, mEdgeDOFs, mMC, mMesh.getFace (f), null, null);
      return b;
   }

   public Vector3d getFaceNormal (int f, int edgeIdx) {
      return GeometryDerivative
         .faceNormal (mMC, mModel, mMesh.getFace (f), edgeIdx, null, null);
   }
}

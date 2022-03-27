package artisynth.demos.growth.models.ts.evouga;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.VectorNd;

import static java.lang.Math.abs;
import static java.lang.Math.max;

public class StiffnessMatrixUtil {
   
   /**
    * Zero-out the blocks of the given sparse matrix.
    * 
    * @param model
    * @param edgeDelegates
    * @param S
    */
   public static void clearStiffness(
      FemModel3d model, FemNode3d[] edgeDelegates, SparseNumberedBlockMatrix S
   ) {
      for (int n = 0; n < model.numNodes (); n++) {
         FemNode3d node = model.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }        
      }
      
      for (int e = 0; e < edgeDelegates.length; e++) {
         FemNode3d edgeDelegate = edgeDelegates[e];
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }        
      }
   }
   
   /**
    * Populate the given sparse matrix with the FemNodeNeighbor blocks.
    * 
    * @param model
    * @param edgeDelegates
    * @param S
    */
   public static void initBlocksInSparseMatrix(
      FemModel3d model, FemNode3d[] edgeDelegates, SparseNumberedBlockMatrix S   
   ) {
      for (int n = 0; n < model.numNodes (); n++) {
         FemNode3d node = model.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            nbr.addSolveBlocks (S, node);
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            nbr.addSolveBlocks (S, node);
         }        
      }
      
      for (int e = 0; e < edgeDelegates.length; e++) {
         FemNode3d edgeDelegate = edgeDelegates[e];
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            nbr.addSolveBlocks (S, edgeDelegate);
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            nbr.addSolveBlocks (S, edgeDelegate);
         }        
      }
   }
   
   //////////////////////////////////////
   // Row Max
   //////////////////////////////////////
   
   protected static void _updateRowMaxsByBlock(
      VectorNd maxs, int bi, int bj, MatrixBlock B
   ) {
      double[] maxs_ = maxs.getBuffer ();
      
      for (int r = 0; r < 3; r++) {
         for (int c = 0; c < 3; c++) {
            maxs_[3*bi + r] = max(maxs_[3*bi + r], abs(B.get (r, c)));
         }
      }
   }
   
   /**
    * Populate the given vector with the max of each row in sparse matrix S.
    * 
    * @param model
    * @param edgeDelegates
    * @param S
    * @param rvMaxs
    */
   public static void getRowMaxs(
      FemModel3d model, FemNode3d[] edgeDelegates, SparseNumberedBlockMatrix S, 
      VectorNd rvMaxs
   ) {
      for (int n = 0; n < model.numNodes (); n++) {
         FemNode3d node = model.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _updateRowMaxsByBlock(rvMaxs, n, nb, S.getBlock (n, nb));
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _updateRowMaxsByBlock(rvMaxs, n, nb, S.getBlock (n, nb));
         }        
      }

      for (int e = 0; e < edgeDelegates.length; e++) {
         FemNode3d edgeDelegate = edgeDelegates[e];
         int ed = edgeDelegate.getSolveIndex();
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _updateRowMaxsByBlock(rvMaxs, ed, nb, S.getBlock (ed, nb));
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _updateRowMaxsByBlock(rvMaxs, ed, nb, S.getBlock (ed, nb));
         }        
      }
   }
   
   //////////////////////////////////////
   // Diagonal Multiplication
   //////////////////////////////////////
   
   /**
    * Helper function for mulDiagBySparse. A block from the sparse matrix is
    * multiplied by its respective sub-vector of the diagonal vector.
    * 
    * @param bi
    * @param bj
    * @param B
    * @param D
    * @param isDiagLeftSide
    */
   protected static void _mulDiagByBlock(
      int bi, int bj, MatrixBlock B, double[] D, boolean isDiagLeftSide
   ) {
      for (int r = 0; r < 3; r++) {
         for (int c = 0; c < 3; c++) {
            if (isDiagLeftSide) {
               B.set (r, c, B.get (r, c) * D[3*bi+r]);
            } else {
               B.set (r, c, B.get (r, c) * D[3*bj+c]);
            }
         }
      }
   }
   
   /**
    * Multiple a diagonal matrix with a sparse matrix.
    * 
    * @param D
    * Diagonal matrix, given as a diagonal vector.
    * @param model
    * @param edgeDelegates
    * @param S
    * @param isDiagLeftSide
    * D*S is perform if true, otherwise S*D.
    */
   public static void mulDiagBySparse(
      VectorNd D, FemModel3d model, FemNode3d[] edgeDelegates, 
      SparseNumberedBlockMatrix S, boolean isDiagLeftSide
   ) {
      double[] D_ = D.getBuffer ();
      
      for (int n = 0; n < model.numNodes (); n++) {
         FemNode3d node = model.getNode(n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _mulDiagByBlock(n, nb, S.getBlock (n, nb), D_, isDiagLeftSide);
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _mulDiagByBlock(n, nb, S.getBlock (n, nb), D_, isDiagLeftSide);
         }        
      }

      for (int e = 0; e < edgeDelegates.length; e++) {
         FemNode3d edgeDelegate = edgeDelegates[e];
         int ed = edgeDelegate.getSolveIndex();
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _mulDiagByBlock(ed, nb, S.getBlock (ed, nb), D_, isDiagLeftSide);
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _mulDiagByBlock(ed, nb, S.getBlock (ed, nb), D_, isDiagLeftSide);
         }        
      }
   }
   
   //////////////////////////////////////
   // Convenience methods
   //////////////////////////////////////
   
   public static Matrix3d getIndirectNeighborK00(FemNode3d nodeA, FemNode3d nodeB) {
      FemNodeNeighbor neigh = nodeA.getIndirectNeighbor (nodeB);
      if (neigh == null) {
         neigh = nodeA.addIndirectNeighbor (nodeB);
      }
      return neigh.getK00 ();
   }
}

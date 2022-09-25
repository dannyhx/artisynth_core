package artisynth.demos.growth.models.ts.evouga;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.demos.growth.GrowNode3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.VectorNd;

public class StiffnessMatrixUtil {

   public final static int EDGE_DELEGATE_FLAG = 256;

   /**
    * Zero-out the blocks of the given sparse matrix.
    * 
    * @param model
    * @param edgeDelegates
    * @param S
    */
   public static void clearStiffness (
      FemModel3d model, FemNode3d[] edgeDelegates,
      SparseNumberedBlockMatrix S) {
      for (int n = 0; n < model.numNodes (); n++) {
         FemNode3d node = model.getNode (n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors ()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }
         for (FemNodeNeighbor nbr : node.getIndirectNeighbors ()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }
      }

      for (int e = 0; e < edgeDelegates.length; e++) {
         FemNode3d edgeDelegate = edgeDelegates[e];
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors ()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors ()) {
            int b = nbr.getBlockNumber ();
            MatrixBlock B = S.getBlockByNumber (b);
            B.setZero ();
         }
      }
   }

   // /**
   // * Populate the given sparse matrix with the FemNodeNeighbor blocks.
   // *
   // * @param model
   // * @param edgeDelegates
   // * @param S
   // */
   // public static void initBlocksInSparseMatrix(
   // FemModel3d model, FemNode3d[] edgeDelegates, SparseNumberedBlockMatrix S
   // ) {
   // for (int n = 0; n < model.numNodes (); n++) {
   // FemNode3d node = model.getNode(n);
   // for (FemNodeNeighbor nbr : node.getNodeNeighbors()) {
   // nbr.addSolveBlocks (S, node);
   // }
   // for (FemNodeNeighbor nbr : node.getIndirectNeighbors()) {
   // nbr.addSolveBlocks (S, node);
   // }
   // }
   //
   // for (int e = 0; e < edgeDelegates.length; e++) {
   // FemNode3d edgeDelegate = edgeDelegates[e];
   // for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors()) {
   // nbr.addSolveBlocks (S, edgeDelegate);
   // }
   // for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors()) {
   // nbr.addSolveBlocks (S, edgeDelegate);
   // }
   // }
   // }

   //////////////////////////////////////
   // Row Max
   //////////////////////////////////////

   protected static void _updateRowMaxsByBlock (
      VectorNd maxs, int rowOffset, MatrixBlock B) {
      double[] maxs_ = maxs.getBuffer ();

      for (int r = 0; r < B.rowSize (); r++) {
         for (int c = 0; c < B.colSize (); c++) {
            maxs_[rowOffset + r] =
               max (maxs_[rowOffset + r], abs (B.get (r, c)));
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
   public static void getRowMaxs (
      FemModel3d model, FemNode3d[] edgeDelegates, SparseNumberedBlockMatrix S,
      VectorNd rvMaxs) {
      for (int n = 0; n < model.numNodes (); n++) {
         FemNode3d node = model.getNode (n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors ()) {
            int nb = nbr.getNode ().getSolveIndex ();
            if (S.getBlock (n, nb) == null) {
               continue;
            }
            _updateRowMaxsByBlock (
               rvMaxs, S.getBlockRowOffset (n), S.getBlock (n, nb));
         }
         if (node.getIndirectNeighbors () != null) {
            for (FemNodeNeighbor nbr : node.getIndirectNeighbors ()) {
               int nb = nbr.getNode ().getSolveIndex ();
               _updateRowMaxsByBlock (
                  rvMaxs, S.getBlockRowOffset (n), S.getBlock (n, nb));
            }
         }
      }

      for (int e = 0; e < edgeDelegates.length; e++) {
         FemNode3d edgeDelegate = edgeDelegates[e];
         int ed = edgeDelegate.getSolveIndex ();
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors ()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _updateRowMaxsByBlock (
               rvMaxs, S.getBlockRowOffset (ed), S.getBlock (ed, nb));
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors ()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _updateRowMaxsByBlock (
               rvMaxs, S.getBlockRowOffset (ed), S.getBlock (ed, nb));
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
   protected static void _mulDiagByBlock (
      int rowOffset, int colOffset, MatrixBlock B, double[] D,
      boolean isDiagLeftSide) {
      for (int r = 0; r < B.rowSize (); r++) {
         for (int c = 0; c < B.colSize (); c++) {
            if (isDiagLeftSide) {
               B.set (r, c, B.get (r, c) * D[rowOffset + r]);
            }
            else {
               B.set (r, c, B.get (r, c) * D[colOffset + c]);
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
   public static void mulDiagBySparse (
      VectorNd D, FemModel3d model, FemNode3d[] edgeDelegates,
      SparseNumberedBlockMatrix S, boolean isDiagLeftSide) {
      double[] D_ = D.getBuffer ();

      for (int n = 0; n < model.numNodes (); n++) {
         FemNode3d node = model.getNode (n);
         for (FemNodeNeighbor nbr : node.getNodeNeighbors ()) {
            int nb = nbr.getNode ().getSolveIndex ();
            if (S.getBlock (n, nb) == null) {
               continue;
            }
            _mulDiagByBlock (
               S.getBlockRowOffset (n), S.getBlockColOffset (nb),
               S.getBlock (n, nb), D_, isDiagLeftSide);
         }
         if (node.getIndirectNeighbors () != null) {
            for (FemNodeNeighbor nbr : node.getIndirectNeighbors ()) {
               int nb = nbr.getNode ().getSolveIndex ();
               _mulDiagByBlock (
                  S.getBlockRowOffset (n), S.getBlockColOffset (nb),
                  S.getBlock (n, nb), D_, isDiagLeftSide);
            }
         }
      }

      for (int e = 0; e < edgeDelegates.length; e++) {
         FemNode3d edgeDelegate = edgeDelegates[e];
         int ed = edgeDelegate.getSolveIndex ();
         for (FemNodeNeighbor nbr : edgeDelegate.getNodeNeighbors ()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _mulDiagByBlock (
               S.getBlockRowOffset (ed), S.getBlockColOffset (nb),
               S.getBlock (ed, nb), D_, isDiagLeftSide);
         }
         for (FemNodeNeighbor nbr : edgeDelegate.getIndirectNeighbors ()) {
            int nb = nbr.getNode ().getSolveIndex ();
            _mulDiagByBlock (
               S.getBlockRowOffset (ed), S.getBlockColOffset (nb),
               S.getBlock (ed, nb), D_, isDiagLeftSide);
         }
      }
   }

   //////////////////////////////////////
   // Convenience methods
   //////////////////////////////////////

   public static Matrix3x3Block getDirectNeighborBlock (
      FemNode3d nodeA, FemNode3d nodeB, SparseNumberedBlockMatrix S,
      boolean isFirstStep) {
      FemNodeNeighbor neigh = nodeA.getNodeNeighbor (nodeB);
      if (isFirstStep) {
         neigh.addSolveBlocks (S, nodeA);
      }
      return (Matrix3x3Block)S.getBlockByNumber (neigh.getBlockNumber ());
   }

   public static Matrix3x3Block getIndirectNeighborBlock (
      FemNode3d nodeA, FemNode3d nodeB, SparseNumberedBlockMatrix S) {
      if (nodeA.getNodeNeighbor (nodeB) != null) {
         return getDirectNeighborBlock (nodeA, nodeB, S, false);
      }

      FemNodeNeighbor neigh = nodeA.getIndirectNeighbor (nodeB);
      if (neigh == null) {
         neigh = nodeA.addIndirectNeighbor (nodeB);
         neigh.addSolveBlocks (S, nodeA);
      }

      return (Matrix3x3Block)S.getBlockByNumber (neigh.getBlockNumber ());
   }

   public static MatrixBlock getIndirectEdgeNeighborBlock (
      FemNode3d nodeA, FemNode3d nodeB, FemEdgeNeighborType neighType,
      SparseNumberedBlockMatrix S) {
      FemNodeNeighbor neigh = nodeA.getIndirectNeighbor (nodeB);
      if (neigh == null) {
         neigh = ((GrowNode3d)nodeA).addIndirectEdgeNeighbor (nodeB, neighType);
         neigh.addSolveBlocks (S, nodeA);
      }
      return S.getBlockByNumber (neigh.getBlockNumber ());
   }
}

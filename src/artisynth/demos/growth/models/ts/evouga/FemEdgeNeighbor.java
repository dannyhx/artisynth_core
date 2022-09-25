package artisynth.demos.growth.models.ts.evouga;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import maspack.matrix.Matrix1x1;
import maspack.matrix.Matrix1x1Block;
import maspack.matrix.Matrix1x3;
import maspack.matrix.Matrix1x3Block;
import maspack.matrix.Matrix3x1;
import maspack.matrix.Matrix3x1Block;
import maspack.matrix.MatrixBase;
import maspack.matrix.MatrixBlock;
import maspack.matrix.SparseNumberedBlockMatrix;

public class FemEdgeNeighbor extends FemNodeNeighbor {
   protected FemEdgeNeighborType myType;
   protected MatrixBase myKE;

   public FemEdgeNeighbor (FemNode3d node, FemEdgeNeighborType type) {
      super (node);
      myType = type;
      myKE = createKE ();
   }

   protected int getOrCreateBlock (
      SparseNumberedBlockMatrix S, int bi, int bj) {
      MatrixBlock blk = (MatrixBlock)S.getBlock (bi, bj);
      if (blk == null) {
         blk = createKEBlock ();
         S.addBlock (bi, bj, blk);
      }
      return blk.getBlockNumber ();
   }

   protected MatrixBase createKE () {
      if (myType == FemEdgeNeighborType.NODE_EDGE) {
         return new Matrix3x1 ();
      }
      else if (myType == FemEdgeNeighborType.EDGE_NODE) {
         return new Matrix1x3 ();
      }
      else if (myType == FemEdgeNeighborType.EDGE_EDGE) {
         return new Matrix1x1 ();
      }
      else {
         throw new RuntimeException ("Unsupported");
      }
   }

   protected MatrixBlock createKEBlock () {
      if (myType == FemEdgeNeighborType.NODE_EDGE) {
         return new Matrix3x1Block ();
      }
      else if (myType == FemEdgeNeighborType.EDGE_NODE) {
         return new Matrix1x3Block ();
      }
      else if (myType == FemEdgeNeighborType.EDGE_EDGE) {
         return new Matrix1x1Block ();
      }
      else {
         throw new RuntimeException ("Unsupported");
      }
   }
}

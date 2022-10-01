package artisynth.demos.growth;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.demos.growth.util.MathUtil;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class PolarityElementAux {
   /** Primary direction of growth. */
   public Vector3d mPolDir = new Vector3d (0, 1, 0);

   /** 3 directions of growth. */
   public Matrix3d mFrame = new Matrix3d ();

   /**
    * Growth tensor. Contains magnitude of growth for each node. (numNodes x
    * 3dof).
    */
   public MatrixNd mElementGrowthTensor;

   /**
    * Rotated growth tensor. (numNodes x 6). Each row contains a symmetrical 3x3
    * plastic strain matrix.
    */
   public MatrixNd mRotatedElementGrowthStrains;

   /**
    * Transformed variation of the rotated growth tensor where each column
    * contains a symmetrical 3x3 plastic strain matrix for a given integration
    * point of the element. (6 x numIntegPts).
    */
   public MatrixNd mStrainAtIntegPts;

   /** Matrix representation of the strain at each edge. */
   public Matrix3d mBendStrain;

   /**
    * Given a FEM model rest state, assign a polarity concentration to each node
    * such that a smooth gradient (0 to 1) is formed along dir.
    */
   public static void createPolGradientAgainstAxis (
      FemModel3d model, int axisIdx, int isTwdPosAxis) {
      //
      if (axisIdx != 0 && axisIdx != 1 && axisIdx != 2) {
         throw new AssertionError ("axisIdx should be 0 (x), 1 (y), or 2 (z)");
      }

      if (isTwdPosAxis != -1 && isTwdPosAxis != 1) {
         throw new AssertionError (
            "isTwdPosAxis should be -1 (i.e. flipped axis) or 1 (default)");
      }

      Double minVal = null;
      Double maxVal = null;

      // Find start and end boundary of model.
      for (FemNode3d node : model.getNodes ()) {
         Point3d vtxPnt = node.getRestPosition ();

         double v = vtxPnt.get (axisIdx);

         if (minVal == null || v < minVal) {
            minVal = v;
         }

         if (maxVal == null || v > maxVal) {
            maxVal = v;
         }
      }

      Point3d startPnt = new Point3d ();
      Vector3d gradientRuler = new Vector3d ();
      if (axisIdx == 0) {
         gradientRuler.set (Vector3d.X_UNIT);
         startPnt.set (Vector3d.X_UNIT);
      }
      else if (axisIdx == 1) {
         gradientRuler.set (Vector3d.Y_UNIT);
         startPnt.set (Vector3d.Y_UNIT);
      }
      else {
         gradientRuler.set (Vector3d.Z_UNIT);
         startPnt.set (Vector3d.Z_UNIT);
      }
      gradientRuler.scale (maxVal - minVal); // Towards positive.
      gradientRuler.scale (isTwdPosAxis); // Flip to negative if necessary.

      if (isTwdPosAxis == -1) {
         startPnt.scale (maxVal);
      }
      else {
         startPnt.scale (minVal);
      }

      createPolGradientAgainstVector (model, startPnt, gradientRuler);
   }

   public static void createPolGradientAgainstParallelVector (
      FemModel3d model, Point3d s, Vector3d v) {
      v = new Vector3d (v);
      scaleVectorAcrossModel (model, s, v);
      createPolGradientAgainstVector (model, s, v);
   }

   /**
    * Given a FEM model rest state, assign a polarity concentration to each node
    * such that a smooth gradient (0 to 1) is formed, parallel to v.
    */
   protected static void createPolGradientAgainstVector (
      FemModel3d model, Point3d s, Vector3d v) {
      for (FemNode3d node : model.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d)node;

         Point3d nodePnt = gNode.getRestPosition ();
         Vector3d n_s = new Vector3d ().sub (nodePnt, s);

         // Project n_s onto vNorm. Find how much the projection extends
         // relative to v.
         double scale = n_s.dot (v) / v.normSquared ();
         if (scale < 0 - MathUtil.ELIPSON_EX
         || scale > 1 + MathUtil.ELIPSON_EX) {
            throw new AssertionError (
               "Unexpected polarity concentration. Is start and end within bounds of model? Scale: %.2f"
                  .formatted (scale));
         }

         gNode.mPolConc = scale;
      }
   }

   public static void calcPolDir (FemModel3d model) {
      for (ShellElement3d ele : model.getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement)ele;
         GrowNode3d[] nodes = gEle.getNodes ();

         Vector3d polDir =
            MathUtil
               .triangleGradient (
                  new Point3d[] { nodes[0].getRestPosition (),
                                  nodes[1].getRestPosition (),
                                  nodes[2].getRestPosition () },
                  new double[] { nodes[0].mPolConc, nodes[1].mPolConc,
                                 nodes[2].mPolConc });
         polDir.normalize ();

         gEle.mPolAux.mPolDir.set (polDir);
      }
   }

   /* --- Helper Methods --- */

   /**
    * Scale the given vector so its length matches the parallel length of the
    * model.
    * 
    * @param s
    * Assumed to be "bottom" of the model.
    */
   protected static void scaleVectorAcrossModel (
      FemModel3d model, Point3d s, Vector3d v) {

      double maxScale = 0;

      // Find start and end boundary of model.
      for (FemNode3d node : model.getNodes ()) {
         Point3d vtxPnt = node.getRestPosition ();

         Vector3d n_s = new Vector3d ().sub (vtxPnt, s);

         double scale = n_s.dot (v) / v.normSquared ();
         if (scale < -MathUtil.ELIPSON_EX) {
            throw new AssertionError (
               "Expected s to be bottom of model. Got negative scale: %.2f"
                  .formatted (scale));
         }

         if (!Double.isNaN (scale)) {
            maxScale = Math.max (maxScale, scale);
         }
      }

      v.scale (maxScale);
   }
}

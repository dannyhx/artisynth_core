package artisynth.demos.growth.util.test;

import artisynth.demos.growth.util.MathUtil;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

public class MathUtilTest {

   protected void test_trianglePointTo2D () {
      Point3d[] tri =
         new Point3d[] { new Point3d (0, 0, 0), new Point3d (1, 1, 0),
                         new Point3d (-1, 1, 0) };

      Vector2d vec2d =
         MathUtil
            .vec3dToTangentCoords (tri, new Vector3d (0, 1, 0).normalize ());

      System.out.println (vec2d);
   }

   protected void test_vectorAnglePair () {
      Vector3d A = new Vector3d (1, 0, 0);
      Vector3d B = new Vector3d (0.49, -0.49, 0);

      double ang = MathUtil.vectorPairAngle (A, B);

      System.out.println (ang * (180 / Math.PI));
   }

   protected void test_triangleGradient () {
      Point3d[] pts =
         new Point3d[] { new Point3d (0, 0, 0), new Point3d (1, 0, 0),
                         new Point3d (0.5, Math.sqrt (1 - 1 / 4), 0), };
      double[] xs = new double[] { 1, 0, 0.5 };

      Vector3d g = MathUtil.triangleGradient (pts, xs);

      System.out.println (g);
   }

   public static void main (String[] args) {
      MathUtilTest t = new MathUtilTest ();
      t.test_triangleGradient ();
   }
}

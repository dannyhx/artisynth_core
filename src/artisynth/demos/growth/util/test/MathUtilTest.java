package artisynth.demos.growth.util.test;

import artisynth.demos.growth.util.MathUtil;
import maspack.matrix.Point2d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

public class MathUtilTest {
   
   protected void test_trianglePointTo2D() {
      Point3d[] tri = new Point3d[] {
          new Point3d(0,0,0),
          new Point3d(1,1,0), 
          new Point3d(-1,1,0)};
                                  
      Vector2d vec2d = MathUtil.vec3dToTangentCoords (tri, new Vector3d(0,1,0).normalize ());
      
      System.out.println (vec2d);
   }
   
   
   public static void main(String[] args) {
      MathUtilTest t = new MathUtilTest();
      t.test_trianglePointTo2D ();
   }
}

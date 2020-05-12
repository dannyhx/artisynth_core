package artisynth.demos.growth.collision;

import artisynth.demos.growth.collision.NarrowPhaseCCD;
import artisynth.demos.growth.collision.NarrowPhaseCCDTest;
import maspack.matrix.*;
import maspack.util.*;

public class NarrowPhaseCCDTest extends UnitTest {

   public void testSimple() {
      NarrowPhaseCCD collider = new NarrowPhaseCCD();
      
      {
         // vertex p and triangle abc (t = 0)
         Vector3d p0 = new Vector3d (-5.8305, -0.1719, 0.1101);
         Vector3d p1 = new Vector3d (-5.8305, -0.1719, -0.0084);

         // [-20.0 -20.0 0.0, 20.0 20.0 0.0, -20.0 20.0 0.0,
         
//         Vector3d a0 = new Vector3d (20, 20, 0);
//         Vector3d b0 = new Vector3d (-20, 20, 0);
//         Vector3d c0 = new Vector3d (-20, -20, 0);
         
         Vector3d a0 = new Vector3d (-20, -20, 0);
         Vector3d b0 = new Vector3d (20, 20, 0);
         Vector3d c0 = new Vector3d (-20, 20, 0);

         double collide = collider.collideVertexTrianglePnts (
            p0, p1, a0, a0, b0, b0, c0, c0, 1e-5).hitTime;

         System.out.println ("(VT points) collision? " + collide);
      }
      
      // Example vertex-triangle intersection from points at t = 0 and t = 1
      {
         
         // vertex p and triangle abc (t = 0)
         Vector3d p0 = new Vector3d (0, -1, 0);

         Vector3d a0 = new Vector3d (1, 1, 0);
         Vector3d b0 = new Vector3d (0, 1, 1);
         Vector3d c0 = new Vector3d (0, 1, -1);

         // vertex p and triangle abc (t = 1)
         Vector3d p1 = new Vector3d (0, 1, 0);

         Vector3d a1 = new Vector3d (1, -1, 0);
         Vector3d b1 = new Vector3d (0, -1, 1);
         Vector3d c1 = new Vector3d (0, -1, -1);

         double collide = collider.collideVertexTrianglePnts (
            p0, p1, a0, a1, b0, b1, c0, c1, 1e-5).hitTime;

         System.out.println ("(VT points) collision? " + collide);
      }

      // Example vertex-triangle intersection from points at t = 0 and velocities
      {
         // vertex p and triangle abc (t = 0)
         Vector3d p = new Vector3d (1, -1, 0);
         Vector3d a = new Vector3d (1, 1, 0);
         Vector3d b = new Vector3d (0, 1, 1);
         Vector3d c = new Vector3d (0, 1, -1);

         // velocities
         Vector3d pDir = new Vector3d (0, 2, 0);
         Vector3d aDir = new Vector3d (0, -2, 0);
         Vector3d bDir = new Vector3d (0, -2, 0);
         Vector3d cDir = new Vector3d (0, -2, 0);

         double collide = collider.collideVertexTriangleDiff (
            p, pDir, a, aDir, b, bDir, c, cDir, 1e-5f).hitTime;

         System.out.println ("(VT rays) collision? " + collide);
      }

      // Example edge-edge intersection from points at t = 0 and t = 1
      {
         // edges ab and cd (t = 0)
         Vector3d a0 = new Vector3d (1, -1, 0);
         Vector3d b0 = new Vector3d (-1, -1, 0);

         Vector3d c0 = new Vector3d (0, 1, -1);
         Vector3d d0 = new Vector3d (0, 1, 1);

         // edges ab and cd (t = 1)
         Vector3d a1 = new Vector3d (1, 1, 0);
         Vector3d b1 = new Vector3d (-1, 1, 0);

         Vector3d c1 = new Vector3d (0, -1, -1);
         Vector3d d1 = new Vector3d (0, -1, 1);

         double collide = collider.collideEdgeEdgePnts (
            a0, a1, b0, b1, c0, c1, d0, d1, 1e-5f).hitTime;

         System.out.println ("(EE points) collision? " + collide);
      }

      // Example edge-edge intersection from points at t = 0 and t = 1
      {

         // edges ab and cd (t = 0)
         Vector3d a = new Vector3d (1, -1, 0);
         Vector3d b = new Vector3d (-1, -1, 0);

         Vector3d c = new Vector3d (0, 1, -1);
         Vector3d d = new Vector3d (0, 1, 1);

         // velocities of ab and cd
         Vector3d aDir = new Vector3d (0, 2, 0);
         Vector3d bDir = new Vector3d (0, 2, 0);

         Vector3d cDir = new Vector3d (0, -2, 0);
         Vector3d dDir = new Vector3d (0, -2, 0);

         double collide = collider.collideEdgeEdgeDiff (
            a, aDir, b, bDir, c, cDir, d, dDir, 1e-5f).hitTime;

         System.out.println ("(EE rays) collision? " + collide);
      }
   }

   public void testSpecial() {
      NarrowPhaseCCD collider = new NarrowPhaseCCD();

      Vector3d p0 = new Vector3d (0.25, 0.25, 0.25);
      Vector3d a0 = new Vector3d (0, 0, 0);
      Vector3d b0 = new Vector3d (1, 0, 0);
      Vector3d c0 = new Vector3d (0, 1, 0);
      Vector3d d0 = new Vector3d ();

      Vector3d pd = new Vector3d (0, 0, -1.1);
      Vector3d ad = new Vector3d (0, 0, -0.1);
      Vector3d bd = new Vector3d (-1, 0, -1);
      Vector3d cd = new Vector3d (0, 0, -0.1);
      Vector3d dd = new Vector3d ();

      double collide = collider.collideVertexTriangleDiff (
         p0, pd, a0, ad, b0, bd, c0, cd, 1e-5f).hitTime;

      System.out.println ("special 1=" + collide);

      pd = new Vector3d (0, 0, -0.5);
      collide = collider.collideVertexTriangleDiff (
         p0, pd, a0, ad, b0, bd, c0, cd, 1e-5f).hitTime;
      System.out.println ("special 2=" + collide);

      ad.set (-0.270,  0.007,  0.223);
      bd.set (-0.323,  0.179, -0.139);
      cd.set ( 0.134, -0.241,  0.129);

      RandomGenerator.setSeed (0x1234);
      for (int i=0; i<20; i++) {
         pd.setRandom();
         collide = collider.collideVertexTriangleDiff (
            p0, pd, a0, ad, b0, bd, c0, cd, 1e-5f).hitTime;
         System.out.println ("rand VF " + i + "=" + collide);
      }

      a0.set (1, 0, 0);
      b0.set (-1, 0, 0);
      c0.set (0, 0.1, -1);
      d0.set (0, 0.1, 1);

      for (int i=0; i<20; i++) {
         ad.setRandom();
         bd.setRandom();
         cd.setRandom();
         dd.setRandom();
         collide = collider.collideEdgeEdgeDiff (
            a0, ad, b0, bd, c0, cd, d0, dd, 1e-5f).hitTime;
         System.out.println ("rand EE " + i + "=" + collide);
      }
   }

   public void timeSpecial() {
      NarrowPhaseCCD collider = new NarrowPhaseCCD();

      Vector3d p0 = new Vector3d (0.25, 0.25, 0.25);
      Vector3d a0 = new Vector3d (0, 0, 0);
      Vector3d b0 = new Vector3d (1, 0, 0);
      Vector3d c0 = new Vector3d (0, 1, 0);
      Vector3d d0 = new Vector3d ();

      Vector3d pd = new Vector3d (0, 0, -1.1);
      Vector3d ad = new Vector3d (0, 0, -0.1);
      Vector3d bd = new Vector3d (-1, 0, -1);
      Vector3d cd = new Vector3d (0, 0, -0.1);
      Vector3d dd = new Vector3d ();

      double collide;

      ad.set (-0.270,  0.007,  0.223);
      bd.set (-0.323,  0.179, -0.139);
      cd.set ( 0.134, -0.241,  0.129);

      RandomGenerator.setSeed (0x1234);

      int pcnt = 20;
      Vector3d[] pds = new Vector3d[pcnt];
      Vector3d[] ads = new Vector3d[pcnt];
      Vector3d[] bds = new Vector3d[pcnt];
      Vector3d[] cds = new Vector3d[pcnt];
      Vector3d[] dds = new Vector3d[pcnt];
      for (int i=0; i<pcnt; i++) {
         pds[i] = new Vector3d();
         pds[i].setRandom();
      }
      for (int i=0; i<pcnt; i++) {
         ads[i] = new Vector3d();
         ads[i].setRandom();
         bds[i] = new Vector3d();
         bds[i].setRandom();
         cds[i] = new Vector3d();
         cds[i].setRandom();
         dds[i] = new Vector3d();         
         dds[i].setRandom();
      }
      // System.out.println ("ads=");
      // for (int i=0; i<pcnt; i++) {
      //    System.out.printf (
      //       "        Vec3f(%g, %g, %g),\n", ads[i].x, ads[i].y, ads[i].z);
      // }

      int tcnt = 200000;
      double sum = 0;
      System.out.println ("warming up ...");
      // warm up
      a0.set (0, 0, 0);
      b0.set (1, 0, 0);
      c0.set (0, 1, 0);
      for (int k=0; k<tcnt; k++) {
         for (int i=0; i<pcnt; i++) {
            sum += collider.collideVertexTriangleDiff (
               p0, pds[i], a0, ad, b0, bd, c0, cd, 1e-5f).hitTime;
         }
      }
      a0.set (1, 0, 0);
      b0.set (-1, 0, 0);
      c0.set (0, 0.1, -1);
      d0.set (0, 0.1, 1);
      for (int k=0; k<tcnt; k++) {      
         for (int i=0; i<pcnt; i++) {
            sum += collider.collideEdgeEdgeDiff (
               a0, ads[i], b0, bds[i], c0, cds[i], d0, dds[i], 1e-5f).hitTime;
         }
      }
      System.out.println ("timing ...");
      FunctionTimer timer = new FunctionTimer();
      a0.set (0, 0, 0);
      b0.set (1, 0, 0);
      c0.set (0, 1, 0);
      timer.start();
      for (int k=0; k<tcnt; k++) {
         for (int i=0; i<pcnt; i++) {
            sum += collider.collideVertexTriangleDiff (
               p0, pds[i], a0, ad, b0, bd, c0, cd, 1e-5f).hitTime;
         }
      }
      timer.stop();
      System.out.println ("VF time: " + timer.result(pcnt*tcnt));

      a0.set (1, 0, 0);
      b0.set (-1, 0, 0);
      c0.set (0, 0.1, -1);
      d0.set (0, 0.1, 1);
      timer.start();
      for (int k=0; k<tcnt; k++) {      
         for (int i=0; i<pcnt; i++) {
            sum += collider.collideEdgeEdgeDiff (
               a0, ads[i], b0, bds[i], c0, cds[i], d0, dds[i], 1e-5f).hitTime;
         }
      }
      timer.stop();
      System.out.println ("EE time: " + timer.result(pcnt*tcnt));
      System.out.println ("sum=" + sum);
   }

   public void test() {
      testSimple();
      testSpecial();
      timeSpecial();
   }

   public static void main (String[] args) {
      NarrowPhaseCCDTest tester = new NarrowPhaseCCDTest();

      tester.runtest();
   }

}

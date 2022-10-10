package artisynth.demos.growth.util;

import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class BackUtil {
   public static void initRestBackNodes (
      FemModel3d model, double defaultThickness) {
      boolean isShell = model.numShellElements () > 0;

      int numFrontNodes = (isShell) ? model.numNodes () : model.numNodes () / 2;

      for (int f = 0; f < numFrontNodes; f++) {
         FemNode3d fnode = model.getNode (f);

         double sumThickness = 0;
         int ecnt = 0;

         Vector3d bNrm = new Vector3d ();
         for (FemElement e : fnode.getAdjacentElements ()) {
            FemElement3dBase ele = (FemElement3dBase)e;

            // Front-surface nodes.
            FemNode3d adjNode0 = ele.getNodes ()[0];
            FemNode3d adjNode1 = ele.getNodes ()[1];
            FemNode3d adjNode2 = ele.getNodes ()[2];

            Vector3d adjNrm =
               computeNormal (
                  adjNode0.getRestPosition (), adjNode1.getRestPosition (),
                  adjNode2.getRestPosition ());

            // Scale by angle rather than area
            double scale = getAngle (fnode, adjNode0, adjNode1, adjNode2);

            // System.out
            // .printf (
            // "%s, %s, %s\n", adjNode0.getRestPosition ().toString ("%.2f"),
            // adjNode1.getRestPosition ().toString ("%.2f"),
            // adjNode2.getRestPosition ().toString ("%.2f"));
            //
            // System.out
            // .printf (
            // "%s, %s, %s\n", adjNode0.getName (), adjNode1.getName (),
            // adjNode2.getName ());
            //
            // System.out.printf ("Nrm: %s\n", adjNrm);

            bNrm.scaledAdd (scale, adjNrm);

            sumThickness += defaultThickness;
            ecnt++;
         }

         if (ecnt > 0) {
            bNrm.normalize ();
            bNrm.scale (sumThickness / ecnt);
         }

         // System.out.printf ("ScaledNrm: %s\n", bNrm);

         Point3d bnodeResPos =
            (Point3d)new Point3d (fnode.getRestPosition ()).add (bNrm);

         if (isShell) {
            fnode.setBackRestPosition (bnodeResPos);
            fnode.setBackPosition (fnode.getBackRestPosition ());
         }
         else {
            FemNode3d bnode = model.getNode (model.numNodes () / 2 + f);
            bnode.setRestPosition (bnodeResPos);
            bnode.setPosition (bnode.getRestPosition ());
         }

      }
   }

   /**
    * Computes a normal for three points oriented counter-clockwise and returns
    * the area of the associated triangle.
    */
   protected static Vector3d computeNormal (
      Point3d p0, Point3d p1, Point3d p2) {
      Vector3d nrm = new Vector3d ();

      Vector3d d01 = new Vector3d ();
      Vector3d d02 = new Vector3d ();
      d01.sub (p1, p0);
      d02.sub (p2, p0);
      nrm.cross (d01, d02);
      double mag = nrm.norm ();
      if (mag != 0) {
         nrm.scale (1 / mag);
      }
      return nrm;
   }

   /**
    * Get the angle (in radians) at node A relative to a 3-node element.
    */
   protected static double getAngle (
      FemNode3d A, FemNode3d a, FemNode3d b, FemNode3d c) {
      FemNode3d B = null;
      FemNode3d C = null;
      for (FemNode3d node : new FemNode3d[] { a, b, c }) {
         if (node.getNumber () != A.getNumber ()) {
            if (B == null)
               B = node;
            else
               C = node;
         }
      }

      Vector3d BA =
         new Vector3d (B.getRestPosition ()).sub (A.getRestPosition ());
      Vector3d CA =
         new Vector3d (C.getRestPosition ()).sub (A.getRestPosition ());

      return BA.angle (CA);
   }
}

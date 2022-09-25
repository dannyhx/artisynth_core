package artisynth.demos.growth.util;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import java.util.ArrayList;

import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.demos.growth.GrowIntegrationData3d;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;

public class FemUtil {

   public static void setFpFromRestMesh (
      ElementClass eleCls, FemModel3d model, PolygonalMesh targetMeshFront,
      PolygonalMesh targetMeshBack) {
      // Modify world state to target

      ArrayList<Vertex3d> targetFrontVtxs = targetMeshFront.getVertices ();
      ArrayList<Vertex3d> targetBackVtxs = targetMeshBack.getVertices ();

      for (int v = 0; v < targetFrontVtxs.size (); v++) {
         // Front

         FemNode3d node = model.getNode (v);
         node.setPosition (targetFrontVtxs.get (v).pnt);

         // Back

         if (eleCls == ElementClass.VOLUMETRIC) {
            node = model.getNode (targetFrontVtxs.size () + v);
            node.setPosition (targetBackVtxs.get (v).pnt);
         }
         else { // Shell
            node.setBackPosition (targetBackVtxs.get (v).pnt);
         }
      }

      // Debug

      // for (FemNode3d node : model.getNodes ()) {
      // node.setPosition (node.getRestPosition ());
      //
      // if (eleCls == ElementClass.SHELL) {
      // node.setBackPosition (node.getBackRestPosition ());
      // }
      // }

      // Set plastic strain (curled world relative to flat rest)

      for (int f = 0; f < model.numAllElements (); f++) {
         FemElement3dBase ele = null;
         if (eleCls == ElementClass.VOLUMETRIC) {
            ele = model.getElement (f);
         }
         else { // Shell
            ele = model.getShellElement (f);
         }

         FemNode3d[] nodes = ele.getNodes ();

         IntegrationPoint3d[] ipts = ele.getIntegrationPoints ();
         IntegrationData3d[] idats = ele.getIntegrationData ();

         for (int k = 0; k < ele.numIntegrationPoints (); k++) {
            IntegrationPoint3d ipt = ipts[k];
            GrowIntegrationData3d idat = (GrowIntegrationData3d)idats[k];

            Matrix3d J = new Matrix3d ();
            ipt.computeJacobian (J, nodes, eleCls);

            Matrix3d invJ0 = new Matrix3d ();
            double det = ipt.computeInverseRestJacobian (invJ0, nodes);
            if (det < 0) {
               throw new AssertionError ("Cannot invert rest jacobian");
            }

            Matrix3d F = new Matrix3d ();
            F.mul (J, invJ0);

            idat.setFp (F);
         }
      }

      // Restore world state back to rest state

      for (FemNode3d node : model.getNodes ()) {
         node.setPosition (node.getRestPosition ());

         if (eleCls == ElementClass.SHELL) {
            node.setBackPosition (node.getBackRestPosition ());
         }
      }
   }

   public static double getBottomSurfaceStretchNeededForCurl (
      double stressMultiplier, double thickness, double meshX,
      double meshXDiv) {
      double eleWd = meshX / (float)meshXDiv;

      meshXDiv /= stressMultiplier;

      // Interior angle between each element of cylinder.
      double theta = ((meshXDiv - 2) * PI) / meshXDiv;

      double slopeTheta = theta / 2.0;

      //

      // double adj = thickness / tan(slopeTheta); // q=default. too much
      // double adj = thickness / tan(slopeTheta); // w/ q=1. almost

      // double adj = thickness * (1/sqrt(3)) / tan(slopeTheta); // partial

      // double adj = thickness * (0.5 + 0.5*(1/sqrt(3))) / tan(slopeTheta); //
      // a little too much
      double adj = thickness / tan (slopeTheta) * (1 / sqrt (3)); // Most
                                                                  // accurate.
                                                                  // Requires
                                                                  // 0.1x mesh
                                                                  // size and
                                                                  // high res.

      double g = (eleWd + 2 * adj) / eleWd - 1.0;

      return g;
   }
}

package artisynth.demos.growth.util;

import java.util.LinkedList;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.demos.growth.GrowIntegrationData3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/**
 * Utility functions that operate on a FEM-Shell model. 
 */
public class ShellUtil {
   
   protected static final double PROBLEMATIC_AREA_TRESHOLD = 1e-6;
   protected static final int VISIT_FLAG = ModelComponentBase.createTempFlag ();
   
   /**
    * Get the world-space normal vector of the given node.
    * 
    * @param isRest
    * If true, compute normal in reference-space.
    */
   public static Vector3d getNormal(FemNode3d node, boolean isRest) {
      Vector3d nodeNormal = new Vector3d();
      
      for (ShellElement3d ele : node.getAdjacentShellElements ()) {
         Vector3d eleNrm = getNormal((ShellTriElement)ele, isRest);
         nodeNormal.add (eleNrm);
      }
      
      nodeNormal.scale (1.0 / node.getAdjacentShellElements ().size ());
      nodeNormal.normalize ();
      return nodeNormal;
   }
   
   /**
    * Get the world-space normal vector of the given element.
    * 
    * @param isRest
    * If true, compute normal in reference-space.
    */
   public static Vector3d getNormal(ShellTriElement ele, boolean isRest) {
      if (isRest) {
         return MathUtil.getNormal (
            ele.getNodes ()[0].getRestPosition (), 
            ele.getNodes ()[1].getRestPosition (), 
            ele.getNodes ()[2].getRestPosition () 
         );
      }
      else {
         return MathUtil.getNormal (
            ele.getNodes ()[0].getPosition (), 
            ele.getNodes ()[1].getPosition (), 
            ele.getNodes ()[2].getPosition () 
         );
      }
   }
   
   /**
    * Get the world-space area of a triangle that is specified using 3 nodes.
    * 
    * @param isRest
    * If true, compute the area in reference-space.
    */
   public static double area(FemNode3d[] nodes, boolean isRest) {
      if (isRest) {
         return MathUtil.area (
            nodes[0].getRestPosition (), 
            nodes[1].getRestPosition (), 
            nodes[2].getRestPosition () 
         );
      }
      else {
         return MathUtil.area (
            nodes[0].getPosition (), 
            nodes[1].getPosition (), 
            nodes[2].getPosition () 
         );
      }
   }
   
   /**
    * Get the average plastic deformation gradient across the integration 
    * points of an element. 
    */
   public static Matrix3d idatAvgFp(ShellElement3d ele) {
      Matrix3d avgFp = new Matrix3d();
      
      for (IntegrationData3d idat : ele.getIntegrationData ()) {
         GrowIntegrationData3d gid = (GrowIntegrationData3d)idat;
         avgFp.add ( gid.getFp () );
      }
      
      avgFp.scale ( 1.0 / ele.numIntegrationPoints () );
      return avgFp;
   }
   
   /**
    * Given a list of elements, return the non-duplicate list of nodes.
    */
   public static FemNode3d[] uniqueNodes(LinkedList<ShellElement3d> eles) {    
      LinkedList<FemNode3d> unqNodes = new LinkedList<FemNode3d>();
      
      // Clear visit flags (sanity check)
      for (ShellElement3d ele : eles) {
         for (FemNode3d node : ele.getNodes ()) {
            node.clearFlag (VISIT_FLAG); }}
 
      for (ShellElement3d ele : eles) {
         for (FemNode3d node : ele.getNodes ()) {
            if (node.checkFlag (VISIT_FLAG)) {
               continue;
            }
            node.setFlag (VISIT_FLAG);
            
            unqNodes.add (node);
         }
      }

      // Clear visit flags
      for (ShellElement3d ele : eles) {
         for (FemNode3d node : ele.getNodes ()) {
            node.clearFlag (VISIT_FLAG); }}
      
      FemNode3d[] unqNodesArr = new FemNode3d[unqNodes.size ()];
      unqNodesArr = unqNodes.toArray (unqNodesArr);
      
      return unqNodesArr;
   }
   
   /**
    * Signal that the FEM Model has been modified. This forces the rest volume,
    * mass, stiffness matrix, and such to be recomputed. 
    */
   public static void invalidateFem(FemModel3d femModel) {
      femModel.invalidateRestData ();
      femModel.invalidateStressAndStiffness ();
      for (ShellElement3d ele : femModel.getShellElements ()) {
         ele.updateRestVolumeAndMass ();
      }
   }
   
   /**
    * Create an element's jacobian matrix that describes the change of 
    * some node quantity (e.g. world-space velocity) with respect to the change
    * in local coordinates of the shell element.
    * 
    * Formulation is shown in equation A.2 of the thesis paper
    * "Large Growth Deformations of Thin Tissue using Solid-Shells". 
    * Replace x_a and y_a with the front and back node quantities.
    * 
    * @param iPt
    * Location that gradient is sampled from.
    * 
    * @param nodes
    * Nodes that comprise the element. 
    * 
    * @param frontVals
    * Node quantities of the front nodes.
    * 
    * @param backVals
    * Node quantities of the back nodes. If null, frontVals is used in place.
    */
   public static Matrix3d createCustomJacobian(IntegrationPoint3d iPt, FemNode3d[] nodes,
   Vector3d[] frontVals, Vector3d[] backVals) {
      // POST_CORRECT
      
      Matrix3d J = new Matrix3d();
      
      if (backVals == null) {
         backVals = frontVals;
      }
      
      for (int i=0; i<nodes.length; i++) {
         Vector3d d = new Vector3d();
         d.sub (frontVals[i], backVals[i]);
         
         Vector3d val = new Vector3d();
         val.scaledAdd (1, frontVals[i]);
         val.scaledAdd (-0.5*(1-iPt.getCoords().z), d);  
         
         // Incorrect code. But kept in-case if it's required for reproducing
         // exact growth patterns.
//         Vector3d val = new Vector3d();
//         val.scaledAdd (1-0.5*(1-iPt.getCoords().z), frontVals[i]);
//         val.scaledAdd (0.5*(1-iPt.getCoords().z), backVals[i]);  
         
         double s0 = iPt.getGNs()[i].x; // dN/dE1
         double s1 = iPt.getGNs()[i].y; // dN/dE2
         double s2 = iPt.getShapeWeights().get(i)*0.5; // 1/2 N
         
         // J = (g_1, g_2, g_3)
         J.m00 += s0*val.x; J.m01 += s1*val.x; J.m02 += s2*d.x;
         J.m10 += s0*val.y; J.m11 += s1*val.y; J.m12 += s2*d.y;
         J.m20 += s0*val.z; J.m21 += s1*val.z; J.m22 += s2*d.z;
      }
      
      return J;
   }
   
   /**
    * Compute the world-space global position for a given integration point. 
    */
   public static Point3d computePositionAtIntegrationPoint(int k, ShellElement3d ele) {
      IntegrationPoint3d iPt = ele.getIntegrationPoints ()[k];

      Point3d pos = new Point3d();
      double[] Nbuf = iPt.getShapeWeights().getBuffer();   
      double dirScale = 0.5*(1-iPt.getCoords().z);
      FemNode3d[] nodes = ele.getNodes ();
      for (int i=0; i<nodes.length; i++) {
         Point3d fntPos = nodes[i].getPosition ();
         Point3d bckPos = nodes[i].getBackPosition ();
         Vector3d dir = new Vector3d().sub (fntPos, bckPos);
         
         Point3d ctrb = new Point3d();
         ctrb.scaledAdd (-dirScale, dir, fntPos);  
         
         pos.scaledAdd (Nbuf[i], ctrb);
      }      
      
      return pos;
   }
   
   /* --- Debug --- */
   
   public static boolean detectOverlappingNode(FemModel3d femModel) {
      for (FemNode3d n0 : femModel.getNodes ()) {
         for (FemNode3d n1 : femModel.getNodes ()) {
            if (n0 != n1 && n0.getRestPosition ().distance (n1.getRestPosition ()) < MathUtil.ELIPSON) {
               System.out.println ("Overlapping Node: " + n0.getName () + " " + n1.getName ());
               return true;
            }
         }
      }
      
      return false;
   }
   
   public static boolean detectNegativeInverseRestJacobian(
   FemModel3d femModel) {
      for (ShellElement3d ele : femModel.getShellElements ()) {
         IntegrationPoint3d[] ipts = ele.getIntegrationPoints ();
         IntegrationData3d[] idts = ele.getIntegrationData ();
         
         for (int k = 0; k < ipts.length; k++) {
            double invDetJ = idts[k].computeInverseRestJacobian (
               ipts[k], ele.getNodes ());
            
            if (invDetJ <= MathUtil.ELIPSON) {
               System.out.println ("Negative jaco in ele: " + ele.getIndex ());
               return true;
            }
         }
      }
      
      return false;
   }
   
   public static boolean detectProblematicRestArea(FemModel3d femModel) {
      for (ShellElement3d ele : femModel.getShellElements ()) {
         if (detectProblematicRestArea(ele)) {
            return true;
         }
      }
      return false;
   }
   
   public static boolean detectProblematicRestArea(ShellElement3d ele) {
      double area = ShellUtil.area (ele.getNodes (), true);
      boolean rv = (area < PROBLEMATIC_AREA_TRESHOLD);
      
      if (rv) {
         System.out.println ("Detected problematic rest area");
      }
      
      return rv;
   }
   
   public static Point3d getPosition(FemNode3d node, boolean isFront, boolean isWorld) {
      return 
         (isFront && isWorld) ? node.getPosition () :
         (isFront && !isWorld) ? node.getRestPosition () : 
         (!isFront && isWorld) ? node.getBackPosition () :
                                 node.getBackRestPosition ();
   }
   
   public static Point3d getAvgPosition(LinkedList<FemNode3d> nodes, boolean isFront, boolean isWorld) {
      Point3d avgPos = new Point3d();
      
      for (FemNode3d node : nodes) {
         avgPos.add ( getPosition(node, isFront, isWorld) );
      }
      
      avgPos.scale (1.0/nodes.size ());
      return avgPos;
   }
   
   public static void setPosition(FemNode3d node, boolean isFront, boolean isWorld, Point3d newPos) {
         if (isFront && isWorld) 
            node.setPosition (newPos);
         else if (isFront && !isWorld)
            node.setRestPosition (newPos); 
         else if (!isFront && isWorld) 
            node.setBackPosition (newPos);
         else 
            node.setBackRestPosition (newPos);
   }
}

package artisynth.demos.growth.util;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.demos.growth.def.NeighborElement;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class HingeUtil {
   
   public static Matrix3d computeStrain(ShellTriElement ele) {
      Matrix3d strain = HingeUtil.computeMembraneStrain (ele);
//      Matrix3d bendStrain = HingeUtil.computeBendingStrain (ele); 
//      strain.add(bendStrain);
      return strain;
   }
   
   public static Matrix3d computeMembraneStrain(ShellTriElement ele) {
      Vector3d eleNORRest = ShellUtil.getNormal (ele, true);
      Matrix3d eleStrain = new Matrix3d();
      
      // For each of the 3 adjacent faces of the element, 
      // calculate the edge lengths and t vectors.
      
      double[] ls = new double[3];
      double[] lrs = new double[3];
      Vector3d[] ts = new Vector3d[3];
      
      int e = 0;
      for (NeighborElement neigh : HingeUtil.adjacentFaces (ele)) {
         // 2 nodes of the edge.
         FemNode3d nodeH = neigh.edgeNodes[0];
         FemNode3d nodeT = neigh.edgeNodes[1];
         Point3d nodeHPosWorld = nodeH.getPosition ();
         Point3d nodeTPosWorld = nodeT.getPosition (); 
         Point3d nodeHPosRest = nodeH.getRestPosition ();
         Point3d nodeTPosRest = nodeT.getRestPosition (); 
         
         // Edge vector and length.
         Vector3d edgeVecWorld = new Vector3d(nodeHPosWorld).sub (nodeTPosWorld);
         Vector3d edgeVecRest = new Vector3d(nodeHPosRest).sub (nodeTPosRest);
         double l = edgeVecWorld.norm(); 
         double lr = edgeVecRest.norm (); 
         
         // t vector. Perpendicular to triangle normal and edge.
         Vector3d edgeVecRestUnit = new Vector3d(edgeVecRest).normalize();
         Point3d edgeCenterRest = (Point3d)new Point3d(nodeTPosRest).
            scaledAdd (0.5, edgeVecRest);
         Vector3d t = new Vector3d(eleNORRest).cross(edgeVecRestUnit); 
         HingeUtil.ensureExteriorVecT (ele, edgeCenterRest, t, true);
         
         // Append results.
         ls[e] = l;
         lrs[e] = lr;
         ts[e] = t;
         
         e++;
      }
      
      // 
      
      e = 0;
      for (NeighborElement neigh : HingeUtil.adjacentFaces (ele)) {
         FemNode3d nodeH = neigh.edgeNodes[0];
         FemNode3d nodeT = neigh.edgeNodes[1];
         int h = ShellUtil.getIndex (nodeH);
         int t = ShellUtil.getIndex (nodeT);
         
         double s = lrs[e] * lrs[e] - ls[e] * ls[e];
         
//         if (
//            (h == 0 && t == 1 || h == 1 && t == 0) || 
//            (h == 2 && t == 3 || h == 3 && t == 2) ||
//            (h == 1 && t == 2 || h == 2 && t == 1)
//         ) {
//            s = -0.1;
//         }
         
         Vector3d tj = ts[(e+1) % 3];
         Vector3d tk = ts[(e+2) % 3];
         
         Matrix3d op1 = new Matrix3d();
         op1.outerProduct (tj, tk);
         
         Matrix3d op2 = new Matrix3d();
         op2.outerProduct (tk, tj);
         
         Matrix3d opSum = new Matrix3d(); 
         opSum.add (op1);
         opSum.add (op2);
         
         eleStrain.scaledAdd (s, opSum);
         
         e++;
      }
      
      double area = ShellUtil.area(ele.getNodes (), true); 
      eleStrain.scale (1./(8*area*area));
      
      return eleStrain;
   }
   
   public static Matrix3d computeBendingStrain(ShellTriElement ele) {
      Vector3d eleNORWorld = ShellUtil.getNormal(ele, false);
      Vector3d eleNORRest = ShellUtil.getNormal (ele, true);
      Matrix3d eleStrain = new Matrix3d();
      
      // For each of the 3 adjacent faces of the element.
      for (NeighborElement neigh : HingeUtil.adjacentFaces (ele)) {
         if (neigh.ele == null) {
            continue;
         }
         
         // 2 nodes of the edge.
         FemNode3d nodeH = neigh.edgeNodes[0];
         FemNode3d nodeT = neigh.edgeNodes[1];
         Point3d nodeHPosRest = nodeH.getRestPosition ();
         Point3d nodeTPosRest = nodeT.getRestPosition ();
         
         // Adjacent face.
         ShellTriElement oEle = neigh.ele;
         Vector3d oEleNORWorld = ShellUtil.getNormal (oEle, false);
         Vector3d oEleNORRest = ShellUtil.getNormal (oEle, true);
         
         // Average normal of the 2 faces.
         Vector3d edgeNormalRestUnit = new Vector3d(eleNORRest).add(oEleNORRest).normalize ();
         
         // Edge direction.
         Vector3d edgeVecRest = new Vector3d(nodeHPosRest).sub (nodeTPosRest);
         Vector3d edgeVecRestUnit = new Vector3d(edgeVecRest).normalize ();
         
         // Perpendicular of the average normal and edge direction. 
         Vector3d t = new Vector3d().cross (edgeNormalRestUnit, edgeVecRestUnit);
         t.normalize ();
         
         // Perpendicular should point away from triangle.
         Point3d edgeCenterRest = (Point3d) new Point3d(nodeTPosRest).
            scaledAdd (0.5, edgeVecRest);
         HingeUtil.ensureExteriorVecT (ele, edgeCenterRest, t, true);
                                 
         // t outer product.
         Matrix3d txtt = new Matrix3d(); 
         txtt.outerProduct (t, t);
  
         // Calculate angle between the two normals in world space.
         double ang = eleNORWorld.angle (oEleNORWorld);
         
         // Monotonic function. 
         double phi = 2 * Math.tan (ang);
            
         // Edge strain.
         Matrix3d edgeStrain = new Matrix3d();
         edgeStrain.scaledAdd (phi, txtt);
         edgeStrain.scale (1 / edgeVecRest.norm ());
         
         // Add edge strain to element strain.
         eleStrain.add (edgeStrain);
      }
      
      double eleAreaRest = ShellUtil.area(ele.getNodes (), true);
      eleStrain.scale (0.5 * (1./eleAreaRest));
      
      return eleStrain;
   }
   

   public static void ensureExteriorVecT(
      ShellTriElement ele, Point3d edgeCenter, Vector3d t, boolean isRest) 
   {
      Point3d centroid = ShellUtil.getCentroid (ele, isRest);
      Vector3d edgeToCenter = new Vector3d().sub (centroid, edgeCenter);
      
      // If t is facing towards triangle.
      if (t.dot (edgeToCenter) > 0) {
         t.scale (-1);
      }
   }
 
   /**
    * Given an element, get its 3 adjacent elements (if any).
    */
   protected static NeighborElement[] adjacentFaces(ShellTriElement ele) {
      NeighborElement[] neighs = new NeighborElement[3];
      FemNode3d[] nodes = ele.getNodes ();
      
      for (int e = 0; e < 3; e++) {
         NeighborElement neigh = new NeighborElement(); 
         neigh.edgeNodes[0] = nodes[e];
         neigh.edgeNodes[1] = nodes[(e+1)%3];
         neigh.ele = HingeUtil.getOppositeElement (
            ele, neigh.edgeNodes[0], neigh.edgeNodes[1]);
         
         neighs[e] = neigh;
      }
      
      return neighs;
   }
   
   /**
    * Given an element and two of its nodes, get the adjacent element that 
    * shares the same nodes.
    */
   protected static ShellTriElement getOppositeElement(
      ShellTriElement ele, FemNode3d n1, FemNode3d n2) 
   {
      for (ShellElement3d adjEle : n1.getAdjacentShellElements ()) {
         int numSharedNodes = 0;
         for (FemNode3d adjNode : adjEle.getNodes ()) {
            if (adjNode == n1 || adjNode == n2) {
               numSharedNodes++;
            }
            
            if (numSharedNodes == 2) {
               return (ShellTriElement)adjEle;
            }
         }
      }
      
      return null;
   }
   

}

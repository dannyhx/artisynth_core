package artisynth.demos.growth.util;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.demos.growth.def.NeighborElement;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

public class HingeUtil {
   
   public static Matrix3d computeStrain(ShellTriElement ele) {
      Matrix3d strain = HingeUtil.computeMembraneStrain (ele);
//      Matrix3d bendStrain = HingeUtil.computeBendingStrain (ele); 
//      strain.add(bendStrain);
      return strain;
   }
   
   public static Matrix3d computeMembraneStrain(ShellTriElement ele) {
      Vector3d eleNOR = ShellUtil.getNormal (ele, false);
      Matrix3d eleStrain = new Matrix3d();

      // For each of the 3 adjacent faces of the element, 
      // calculate the edge lengths and t vectors.
      
      NeighborElement[] adjFaces = HingeUtil.adjacentFaces (ele);
      
      double[] ls = new double[3];
      double[] lrs = new double[3];
      Vector3d[] ts = new Vector3d[3];
      
      int e = 0;
      for (NeighborElement neigh : adjFaces) {
         // 2 nodes of the edge.
         FemNode3d nodeH = neigh.edgeNodes[0];
         FemNode3d nodeT = neigh.edgeNodes[1];
         Point3d nodeHPos = nodeH.getLocalPosition ();
         Point3d nodeTPos = nodeT.getLocalPosition (); 
         
         // Edge vector and length.
         Vector3d edgeVec = new Vector3d(nodeHPos).sub (nodeTPos);
         Vector3d edgeVecRest = new Vector3d(
            nodeH.getRestPosition ()).sub (nodeT.getRestPosition ());
         double l = edgeVec.norm(); 
         double lr = edgeVecRest.norm (); 
         
         Vector3d t = HingeUtil.computeVecT (
            ele, eleNOR, edgeVec, nodeTPos, false);

         // Append results.
         ls[e] = l;
         lrs[e] = lr;
         ts[e] = new Vector3d(t.x, t.y, 0);
         
         e++;
      }
      
      e = 0;
      for (NeighborElement neigh : adjFaces) {
         FemNode3d nodeH = neigh.edgeNodes[0];
         FemNode3d nodeT = neigh.edgeNodes[1];
         int h = ShellUtil.getIndex (nodeH);
         int t = ShellUtil.getIndex (nodeT);
         
         double s = lrs[e] * lrs[e] - ls[e] * ls[e];
         
         Vector3d tj = ts[(e+1) % 3];
         Vector3d tk = ts[(e+2) % 3];
         
         Matrix3d op1 = new Matrix3d();
         op1.outerProduct (tj, tk);
         
         Matrix3d op2 = new Matrix3d();
         op2.outerProduct (tk, tj);
         
         eleStrain.scaledAdd (s, op1);
         eleStrain.scaledAdd (s, op2);
         
         e++;
      }
      
      double area = ShellUtil.area(ele.getNodes (), true); 
      eleStrain.scale (1./(8*area*area));
      
      return eleStrain;
   }
   
   public static Matrix3d computeBendingStrain(ShellTriElement ele) {
      Vector3d eleNOR = ShellUtil.getNormal(ele, false);
      Vector3d eleNORRest = ShellUtil.getNormal(ele, true);
      
      Matrix3d eleStrain = new Matrix3d();
      
      // For each of the 3 adjacent faces of the element.
      for (NeighborElement neigh : HingeUtil.adjacentFaces (ele)) {
         if (neigh.ele == null) {
            continue;
         }
         
         // 2 nodes of the edge.
         FemNode3d nodeH = neigh.edgeNodes[0];
         FemNode3d nodeT = neigh.edgeNodes[1];
         Point3d nodeHPos = nodeH.getPosition ();
         Point3d nodeTPos = nodeT.getPosition ();
         
         // Adjacent face.
         ShellTriElement oEle = neigh.ele;
         Vector3d oEleNOR = ShellUtil.getNormal (oEle, false);
         Vector3d oEleNORRest = ShellUtil.getNormal (oEle, true);
         
         // Edge direction.
         Vector3d edgeVec = new Vector3d(nodeHPos).sub (nodeTPos);
         
         // Perpendicular of the average normal and edge direction. 
         Vector3d t = HingeUtil.computeVecT (
            ele, eleNOR, edgeVec, nodeTPos, false);
         t.z = 0;
                                 
         // t outer product.
         Matrix3d edgeStrain = new Matrix3d(); 
         edgeStrain.outerProduct (t, t);
  
         // Calculate angle between the two normals in world space.
         double ang = eleNOR.angle (oEleNOR);
         double angRest = eleNORRest.angle (oEleNORRest);
//         System.out.printf ("Angle: %.2f, RestAngle: %.2f\n", ang * Math.PI, angRest * Math.PI);
         
         // Monotonic function. 
         double phi = 2 * Math.tan (ang);
         double phiRest = 2 * Math.tan (angRest);
         
         double deltaPhi = phiRest - phi;
//         System.out.printf ("DeltaPhi: %.2f\n", deltaPhi);
            
         // Edge strain.
         edgeStrain.scale (deltaPhi * (1 / edgeVec.norm ()));
         
         // Add edge strain to element strain.
         eleStrain.add (edgeStrain);
      }
      
      double eleArea = ShellUtil.area(ele.getNodes (), true);
      eleStrain.scale (0.5 * (1./eleArea));
      
      eleStrain.mul (eleNOR);
      
      return eleStrain;
   }
   
   protected static Vector3d computeVecT(
      ShellTriElement ele, Vector3d nrm, Vector3d edgeVec, Point3d edgeNodeTPos, 
      boolean isRest) 
   {
      Vector3d edgeVecUnit = new Vector3d(edgeVec).normalize();
      Point3d edgeCenter = (Point3d)new Point3d(edgeNodeTPos).
         scaledAdd (0.5, edgeVec);
      Vector3d t = new Vector3d(nrm).cross(edgeVecUnit); 
      HingeUtil.ensureExteriorVecT (ele, edgeCenter, t, isRest);

      return t; 
   }

   protected static void ensureExteriorVecT(
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
         if (adjEle == ele) {
            continue;
         }
         
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

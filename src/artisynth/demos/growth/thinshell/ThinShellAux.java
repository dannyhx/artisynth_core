package artisynth.demos.growth.thinshell;

import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.util.Pair;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.MeshUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x4;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;

public class ThinShellAux {
   
   protected MatrixNd mMatDDEBending;
   protected double mMatWeakening;
   
   protected PolygonalMesh mMesh; 
   protected FemModel3d mModel;
   
   public ThinShellAux(FemModel3d model, PolygonalMesh mesh) {
      this.mMatDDEBending = new MatrixNd(3, 5);
      this.setMaterial ("ribbon");
      this.mMatWeakening = 0;
      
      this.mMesh = mesh;
      this.mModel = model;
      this.refreshIndirectNodeNeighbors ();
   }
   
   public void setMaterial(String matName) {
      MatrixNd m = this.mMatDDEBending;
      
      if (matName == "paper") {
         // materials/paper.json
         m.setRow (0, new double[] {0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3});
         m.setRow (1, new double[] {0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3});
         m.setRow (2, new double[] {0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3});
      } else if (matName == "ribbon") {
         m.setRow (0, new double[] {117.070122e-6, 202.682922e-6, 298.615936e-6, 244.588867e-6, 238.437820e-6});
         m.setRow (1, new double[] {87.509773e-6, 36.396919e-6, 103.628098e-6, 91.630119e-6, 97.175949e-6});
         m.setRow (2, new double[] {59.774277e-6, 59.260632e-6, 64.199661e-6, 55.017326e-6, 59.221714e-6});
      } else {
         throw new RuntimeException("Unsupported material.");
      }
   }
   
   public void refreshIndirectNodeNeighbors() {
      for (FemNode3d node : mModel.getNodes ()) {
         node.clearIndirectNeighbors ();
      }
      
      for (Face face : mMesh.getFaces ()) {
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);
            edge = EdgeDataMap.getRealHalfEdge (edge);
            if (edge.opposite == null) 
            {
               continue;
            }
            
            FemNode3d n0 = mModel.getNode( edge.head.getIndex () );
            FemNode3d n1 = mModel.getNode( edge.tail.getIndex () );
            Vertex3d[] oppVtxs = MeshUtil.getOppositeVtxs (edge);
            FemNode3d n2 = mModel.getNode( oppVtxs[0].getIndex () );
            FemNode3d n3 = mModel.getNode( oppVtxs[1].getIndex () );
            
            for (FemNode3d nodei : new FemNode3d[] {n0, n1, n2, n3}) {
               int bi = nodei.getSolveIndex ();
               
               for (FemNode3d nodej : new FemNode3d[] {n0, n1, n2, n3} ) {
                  int bj = nodej.getSolveIndex ();
                  
                  // Assumes !mModel.mySolveMatrixSymmetricP. 
                  if (bj >= bi) {
                     nodei.addIndirectNeighbor (nodej);
                     nodej.addIndirectNeighbor (nodei);
                  }
               }
            }
         }
      }
   }
   
   /* Space s */
   public void addBendingForceAndStiffness() {
      for (Face face : mMesh.getFaces ()) {
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);
            
            // Its real counter-part will create the EdgeData instead.
            if (!EdgeDataMap.isRealHalfEdge(edge) || edge.opposite == null) {
               continue;
            }
            
//            System.out.printf ("Edge: [%d,%d]\n", 
//               edge.head.getIndex (), edge.tail.getIndex ());
            
            Pair<MatrixNd,VectorNd> fs = computeBendingForceAndStiffness(edge);
            MatrixNd forces = MathUtil.vecToMat_colMajor (fs.second (), 3, 4);
            MatrixNd stiffness = fs.first();
            
            FemNode3d n0 = mModel.getNode( edge.head.getIndex () );
            FemNode3d n1 = mModel.getNode( edge.tail.getIndex () );
            Vertex3d[] oppVtxs = MeshUtil.getOppositeVtxs (edge);
            FemNode3d n2 = mModel.getNode( oppVtxs[0].getIndex () );
            FemNode3d n3 = mModel.getNode( oppVtxs[1].getIndex () );
            
            int i = 0;
            for (FemNode3d nodei : new FemNode3d[] {n0, n1, n2, n3}) {
               int bi = nodei.getSolveIndex ();               
               
               Vector3d iForce = new Vector3d();
               forces.getColumn (i, iForce);
               nodei.getInternalForce ().add(iForce);
               
//               System.out.printf ("Force: %s\n", iForce.toString ("%.2f"));
               
               int j = 0;
               for (FemNode3d nodej : new FemNode3d[] {n0, n1, n2, n3} ) {
                  int bj = nodej.getSolveIndex ();
                  
                  // Assumes !mModel.mySolveMatrixSymmetricP. 
                  if (bj >= bi) {
                     
                     // Get stiffness matrix between nodei and nodej.
                     Matrix3d ijStiff = new Matrix3d();
                     stiffness.getSubMatrix (3*i, 3*j, ijStiff);
                     
                     FemNodeNeighbor neigh = nodei.getIndirectNeighbor(nodej);
                     neigh.getK00 ().add (ijStiff);
                  }
                  
                  j++;
               }
               
               i++;
            }
         }
      }
   }
   
   /* --- Secondary Functions ---*/
   
   /**
    * Calculate the bending forces for the given edge. The 4 nodes (2 from edge
    * and 2 that are opposite of the edge) will have their forces and stiffness
    * determined.
    * 
    * [0] -> Edge Head Node
    * [1] -> Edge Tail Node
    * [2] -> Opposite Node 
    * [3] -> Opposite Node of Opposite Face.
    * 
    * physics.cpp/bending_force(edge) <Space s>
    * 
    * @param edge
    * 
    * @return
    * 
    * Stiffness across the 4 nodes. 12x12 contains 3x3 stiffness for each
    * node-node relationship. 
    * 
    * Forces of the nodes.
    */
   protected Pair<MatrixNd,VectorNd> computeBendingForceAndStiffness(
      HalfEdge edge) 
   {
      FemNode3d edgeHead = mModel.getNode( edge.head.getIndex () );
      FemNode3d edgeTail = mModel.getNode( edge.tail.getIndex () );
      
      Vertex3d[] oppVtxs = MeshUtil.getOppositeVtxs (edge);
      FemNode3d oppNode0 = mModel.getNode( oppVtxs[0].getIndex () );
      FemNode3d oppNode1 = mModel.getNode( oppVtxs[1].getIndex () );
      
      ShellTriElement ele0 = (ShellTriElement)mModel.getShellElement ( 
         edge.getFace ().getIndex ());
      ShellTriElement ele1 = (ShellTriElement)mModel.getShellElement ( 
         edge.getOppositeFace ().getIndex ());
      
      Vector3d n0 = ShellUtil.getNormal (ele0, false);
      Vector3d n1 = ShellUtil.getNormal (ele1, false);
      
      Point3d x0 = edgeHead.getPosition (); 
      Point3d x1 = edgeTail.getPosition ();
      Point3d x2 = oppNode0.getPosition ();
      Point3d x3 = oppNode1.getPosition ();
      
      double theta = MathUtil.dihedralAngle (x0, x1, n0, n1);
      
//      System.out.printf ("Theta: %.2f\n", theta);
      
      double h0 = MathUtil.distanceBetweenPointAndLine (x2, x0, x1) ;
      double h1 = MathUtil.distanceBetweenPointAndLine (x3, x0, x1) ;
      
      Vector2d w_f0 = MathUtil.barycentricWeights (x2, x0, x1);
      Vector2d w_f1 = MathUtil.barycentricWeights (x3, x0, x1); 
      
      Matrix3x4 dthetaM = new Matrix3x4();
      Vector3d w_f00_n0 = new Vector3d(n0).scale (-w_f0.x / h0); 
      Vector3d w_f10_n1 = new Vector3d(n1).scale (-w_f1.x / h1); 
      Vector3d w_f01_n0 = new Vector3d(n0).scale (-w_f0.y / h0); 
      Vector3d w_f11_n1 = new Vector3d(n1).scale (-w_f1.y / h1); 
      dthetaM.setColumn (0, new Vector3d(w_f00_n0).add (w_f10_n1));
      dthetaM.setColumn (1, new Vector3d(w_f01_n0).add (w_f11_n1));
      dthetaM.setColumn (2, new Vector3d(n0).scale (h0));
      dthetaM.setColumn (3, new Vector3d(n1).scale (h1));
      VectorNd dtheta = MathUtil.matToVec_colMajor (dthetaM);
      
      double coeff = bendingCoeff(edge, theta);
      
      // Compute dtheta outer product with scaling.
      MatrixNd dtheta_op = new MatrixNd(12,12);
      double[] dthetaBuf = dtheta.getBuffer ();
      if (dthetaBuf.length != 12) {
         throw new RuntimeException ("Assertion");
      }
      MathUtil.outerProduct (dthetaBuf, dthetaBuf, dtheta_op);
      dtheta_op.scale (-coeff * 0.5);
      
      double restTheta = mModel.myEdgeDataMap.get (edge).mRestTheta;
      dtheta.scale (-coeff * (theta - restTheta) * 0.5);
      
      return new Pair<MatrixNd,VectorNd>(dtheta_op, dtheta);
   }
   
   /* --- Secondary Functions --- */
   
   /**
    * physics.cpp/bending_coeff
    * 
    * DEV: bendingStiffness is called for each edge side. But not matter since
    * min() is called on both.
    * Makes possible assumption adjacent elements do not share nodes, and 
    * that adjacent elements have their own material model. 
    * 
    * @param edge
    * @param theta
    * @return
    */
   protected double bendingCoeff(HalfEdge edge, double theta) {
      FemNode3d edgeHead = mModel.getNode( edge.head.getIndex () );
      FemNode3d edgeTail = mModel.getNode( edge.tail.getIndex () );
      
      Point3d x0 = edgeHead.getPosition (); 
      Point3d x1 = edgeTail.getPosition ();
      
      ShellTriElement ele0 = (ShellTriElement)mModel.getShellElement ( 
         edge.getFace ().getIndex ());
      ShellTriElement ele1 = (ShellTriElement)mModel.getShellElement ( 
         edge.getOppositeFace ().getIndex ());
      
      double a = 
         ShellUtil.area (ele0.getNodes (), false) + 
         ShellUtil.area (ele1.getNodes (), false);
      
      // Edge vector norm.
      double l = new Vector3d(x1).sub (x0).norm ();
      
      double ke0 = bendingStiffness(edge, mMatDDEBending, a, l, theta);
//      double ke1 = bendingStiffness(edge, mMatDDEBending, a, l, theta);
      
      double ke = ke0;                    // min(ke0, ke1)
      double weakening = mMatWeakening;   // max(f0->weakening, f1->weakening)
      ke *= 1/(1 + weakening * 1);   // Rightmost 1 in place of edge->damage.
      double shape = (l*l) / (2*a);
      
      return ke * shape; 
   }
   
   /**
    * dde.cpp/bending_stiffness
    * 
    * DEV: only (x,y) of du is used, but still seems to work if shell's rest 
    * patch is rotated to be vertically flat.
    * 
    * @param edge
    * @param data
    * @param a
    * @param l
    * @param theta
    * @param initial_angle
    * @return
    */
   protected double bendingStiffness(HalfEdge edge, MatrixNd data, 
      double a, double l, double theta) 
   {
      double initialAngle = 0;  // Default argument in .hpp file.
      
      double curv = theta * l / a; 
      double alpha = curv/2; 
      double value = alpha*0.2; 
      
      if (value > 4) 
         value = 4;
      
      int value_i = (int)value;
      if (value_i < 0) 
         value_i = 0; 
      
      if (value_i > 3)
         value_i = 3; 
      
      value -= value_i; 
      
      // Calculate edge's material vector.
      FemNode3d edgeHead = mModel.getNode( edge.head.getIndex () );
      FemNode3d edgeTail = mModel.getNode( edge.tail.getIndex () );
      Point3d eh_u = edgeHead.getRestPosition ();
      Point3d et_u = edgeTail.getRestPosition ();
      Vector3d du = new Vector3d(eh_u).sub (et_u); 
      
      double biasAngle = (Math.atan2 (du.y, du.x) + initialAngle) * 4 / Math.PI;
      
      if (biasAngle < 0) biasAngle =   - biasAngle;
      if (biasAngle > 4) biasAngle = 8 - biasAngle;
      if (biasAngle > 2) biasAngle = 4 - biasAngle;
      
      int biasId = (int) biasAngle;
      if (biasId < 0) biasId = 0; 
      if (biasId > 1) biasId = 1; 
      
      biasAngle -= biasId; 
      
      double actualKe = 
         data.get(biasId,  value_i)   * (1 - biasAngle) * (1-value) + 
         data.get(biasId+1,value_i)   * (    biasAngle) * (1-value) + 
         data.get(biasId,  value_i+1) * (1 - biasAngle) * value + 
         data.get(biasId+1,value_i+1) *      biasAngle  * value; 
      
      if (actualKe < 0) actualKe = 0; 
      
      return actualKe;  
   }
   
   
   
   public static void main (String[] args) {
      // TODO Auto-generated method stub

   }

}

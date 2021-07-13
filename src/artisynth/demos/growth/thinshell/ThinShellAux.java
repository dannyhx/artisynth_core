package artisynth.demos.growth.thinshell;

import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;
import maspack.util.Pair;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.demos.growth.thinshell.EdgeDataMap.EdgeData;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.MeshUtil;
import artisynth.demos.growth.util.ShellUtil;
import artisynth.demos.growth.util.SolverUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix1x3;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x4;
import maspack.matrix.Matrix6x3;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;

public class ThinShellAux {
   
   /** Current data-driven material model */
   protected boolean mIsDDE = false; 
   protected double[][][][] mMatDDEStretchingSamples;
   protected MatrixNd mMatDDEBending;
   
   /** Current alternative material model */
   protected double mAltStretching;
   protected double mAltBending;
   protected double mAltPoisson;
   
   /** Material weakening. Left to 0. */
   protected double mMatWeakening;
   
   protected PolygonalMesh mMesh; 
   protected FemModel3d mModel;
   
   /**
    * Auxiliary helper class to provide thin-shell forces and stiffness to 
    * membranes. 
    * 
    * Reference: ArcSim 3.0.
    */
   public ThinShellAux(FemModel3d model, PolygonalMesh mesh) {
      if (this.mIsDDE) {
         this.setDDEMaterial ("ribbon");
      } else {
         this.setAltMaterial ("default");
      }
      
      this.mMatWeakening = 0;
      
      this.mMesh = mesh;
      this.mModel = model;
      this.refreshIndirectNodeNeighbors ();
   }
   
   protected void setDDEMaterial(String matName) {
      this.mMatDDEStretchingSamples = new double[0][0][0][0];
      this.mMatDDEBending = new MatrixNd(3, 5);
      
      MatrixNd b = this.mMatDDEBending;
      
      if (matName == "paper") {
         // materials/paper.json
         b.setRow (0, new double[] {0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3});
         b.setRow (1, new double[] {0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3});
         b.setRow (2, new double[] {0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3, 0.4e-3});
      } else if (matName == "ribbon") {
         b.setRow (0, new double[] {117.070122e-6, 202.682922e-6, 298.615936e-6, 244.588867e-6, 238.437820e-6});
         b.setRow (1, new double[] {87.509773e-6, 36.396919e-6, 103.628098e-6, 91.630119e-6, 97.175949e-6});
         b.setRow (2, new double[] {59.774277e-6, 59.260632e-6, 64.199661e-6, 55.017326e-6, 59.221714e-6});
      } else {
         throw new RuntimeException("Unsupported material.");
      }
   }
   
   protected void setAltMaterial(String matName) {
      if (matName == "default") {
         // elastic modulus
         double Y = 1e6;  
         double thickness = 1e-2;  // 1e-2 for working bending
         this.mAltPoisson = 0.25;
         
         double A = Y / (1 - Math.pow(this.mAltPoisson, 2));
         this.mAltStretching = A * thickness;
         this.mAltBending = A / 12 * Math.pow(thickness, 3);
      } else {
         throw new RuntimeException("Unsupported material.");
      }
   }
   
   public void setAltMaterial(
      double youngsModulus, double poissonsRatio, double thickness) 
   {
      this.mAltPoisson = poissonsRatio;
      
      double A = youngsModulus / (1 - Math.pow(poissonsRatio, 2));
      this.mAltStretching = A * thickness;
      this.mAltBending = A / 12 * Math.pow(thickness, 3);
   }
   
   /* --- Primary Methods --- */
   
   
   /** 
    * Create the indirect node neighbors that's needed to associate the 
    * opposite vertices of each triangle pair in terms of stiffness. 
    * 
    * This should be called after every remesh. 
    */
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
               int bi = nodei.getLocalSolveIndex ();
               
               for (FemNode3d nodej : new FemNode3d[] {n0, n1, n2, n3} ) {
                  int bj = nodej.getLocalSolveIndex ();
                  
                  // Assumes !mModel.mySolveMatrixSymmetricP. 
                  if (bj >= bi) {
                     FemNodeNeighbor neigh = nodei.getNodeNeighbor (nodej);
                     if (neigh == null) {
                        nodei.addIndirectNeighbor (nodej);
                        
                        System.out.println ("neigh");
                     }
                  }
               }
            }
         }
      }
   }
   
   /** Compute and add the stretching force and stiffness of the FE model. */
   public void addStretchingForceAndStiffness() {
      for (Face face : mMesh.getFaces ()) {
         Pair<MatrixNd,VectorNd> fs = computeStretchingForcesAndStiffness(face);
         MatrixNd stiffness = fs.first; 
         VectorNd force = fs.second;
         
         FemNode3d[] nodes = mModel.getShellElement (face.getIndex ()).getNodes (); 
         
         int i = 0;
         for (FemNode3d nodei : nodes) {
            int bi = nodei.getLocalSolveIndex ();               
            
            Vector3d iForce = new Vector3d();
            force.getSubVector (i * 3, iForce);
            nodei.getInternalForce ().add(iForce);
            
            int j = 0;
            for (FemNode3d nodej : nodes) {
               int bj = nodej.getLocalSolveIndex ();
               
               // Assumes !mModel.mySolveMatrixSymmetricP. 
               if (bj >= bi) {
                  
                  // Get stiffness matrix between nodei and nodej.
                  Matrix3d ijStiff = new Matrix3d();
                  stiffness.getSubMatrix (3*i, 3*j, ijStiff);
                  
                  FemNodeNeighbor neigh = nodei.getNodeNeighbor (nodej);
                  neigh.getK00 ().add(ijStiff);
               }
               
               j++;
            }
            
            i++;
         }
      }
   }
   
   /* Compute and add the bending force and stiffness of the FE model. */
   public void addBendingForceAndStiffness() {
      for (Face face : mMesh.getFaces ()) {
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = face.getEdge (e);
            
            // Its real counter-part will create the EdgeData instead.
            if (!EdgeDataMap.isRealHalfEdge(edge) || edge.opposite == null) {
               continue;
            }
            
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
               int bi = nodei.getLocalSolveIndex ();               
               
               Vector3d iForce = new Vector3d();
               forces.getColumn (i, iForce);
               nodei.getInternalForce ().add(iForce);
               
               int j = 0;
               for (FemNode3d nodej : new FemNode3d[] {n0, n1, n2, n3} ) {
                  int bj = nodej.getLocalSolveIndex ();
                  
                  // Assumes !mModel.mySolveMatrixSymmetricP. 
                  if (bj >= bi) {
                     
                     // Get stiffness matrix between nodei and nodej.
                     Matrix3d ijStiff = new Matrix3d();
                     stiffness.getSubMatrix (3*i, 3*j, ijStiff);
                     
                     FemNodeNeighbor neigh = nodei.getNodeNeighbor (nodej);
                     if (neigh == null) {
                        neigh = nodei.getIndirectNeighbor(nodej);
                     }
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
    * Calculate the nodal bending forces for the given edge. 
    * The 4 nodes (2 from edge and 2 that are opposite of the edge) will have 
    * their forces and stiffness determined.
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
    * node-node neighbor relationship. 
    * 
    * Matrix where the i-th column is the force of the i-th nodes.
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
      
      double theta = ShellUtil.getDihedralAngle (this.mModel, edge, false);
      double restTheta = ShellUtil.getDihedralAngle (this.mModel, edge, true);
      restTheta += mModel.myEdgeDataMap.get (edge).mAngStrain;
      
      double h0 = MathUtil.distanceBetweenPointAndLine (x2, x0, x1) ;
      double h1 = MathUtil.distanceBetweenPointAndLine (x3, x0, x1) ;
      
      h0 = 0.9; 
      h1 = 0.9;
      
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
      
      double coeff = bending_coeff(edge, theta);
      
      MatrixNd dtheta_op = MathUtil.outerProduct (dtheta, dtheta);
      dtheta_op.scale (-coeff * 0.5);
      
      dtheta.scale (-coeff * (theta - restTheta) * 0.5);
      
//      System.out.printf ("w_f0: %.2f\n", w_f0.norm());
      
      return new Pair<MatrixNd,VectorNd>(dtheta_op, dtheta);
   }
   
   /**
    * Get the nodal force and stiffness associated with the given element.
    * 
    * physics.cpp::stretching_force
    * 
    * @param face
    * 
    * @return
    * 
    * Stiffness across the 3 nodes. 9x9 contains 3x3 stiffness for each
    * node-node neighbor relationship. 
    * 
    * Vector of nodal forces. Nodal offsets are 3*i.
    */
   protected Pair<MatrixNd, VectorNd> computeStretchingForcesAndStiffness(
      Face face) 
   {
      Pair<Matrix3d, Matrix3d> F_invDm = deformation_gradient(face);
      Matrix3d F = F_invDm.first; 
      Matrix3d invDm = F_invDm.second;
      
      ShellElement3d ele = mModel.getShellElement (face.getIndex ());
      FemNode3d[] nodes = ele.getNodes ();
      Point3d[] xs = new Point3d[] {
         nodes[0].getPosition (),
         nodes[1].getPosition (), 
         nodes[2].getPosition ()
      };
      
      // 1 - (Fg - 1) = Simulated Deformation.
      Matrix3d E = new Matrix3d(Matrix3d.IDENTITY);
      E.scale (2);
      E.sub (ele.getPlasticDeformation ());
      
      // F * E (i.e. F * Fg)
      F.mul (E);
      
      Matrix3d G = new Matrix3d();
      G.mulTransposeLeft (F, F);
      G.sub(Matrix3d.IDENTITY);
      G.scale(0.5);
      
      Matrix3d Y = new Matrix3d();
      Y.mul(invDm, E);
      
      // Rows of Y.
      Vector3d Yr0 = new Vector3d();
      Vector3d Yr1 = new Vector3d();
      Y.getRow (0, Yr0);
      Y.getRow (1, Yr1);
      
      Vector3d _Yr0_Yr1 = new Vector3d();
      _Yr0_Yr1.scaledAdd(-1, Yr0);
      _Yr0_Yr1.scaledAdd(-1, Yr1);
      
      Matrix3d D = new Matrix3d();
      D.setRow(0, _Yr0_Yr1);
      D.setRow(1, Yr0);
      D.setRow(2, Yr1);
      
      // Columns of D.
      Vector3d Dc0 = new Vector3d();
      Vector3d Dc1 = new Vector3d();
      Vector3d Dc2 = new Vector3d();
      D.getColumn (0, Dc0);
      D.getColumn (1, Dc1);
      D.getColumn (2, Dc2);
      
      // Columns of D in 1x3 matrix form.
      Matrix1x3 Dc0m = new Matrix1x3();
      Matrix1x3 Dc1m = new Matrix1x3();
      Matrix1x3 Dc2m = new Matrix1x3();
      Dc0m.set(Dc0);
      Dc1m.set(Dc1);
      Dc2m.set(Dc2);
      
      MatrixNd DD0 = MathUtil.kronecker (Dc0m, Matrix3d.IDENTITY);
      MatrixNd DD1 = MathUtil.kronecker (Dc1m, Matrix3d.IDENTITY);
      MatrixNd DD2 = MathUtil.kronecker (Dc2m, Matrix3d.IDENTITY);
      
      MatrixNd[] DDs = new MatrixNd[] {DD0, DD1, DD2};
      
      // Store node positions into single 9-vector.
      VectorNd X = new VectorNd(9);
      for (int i = 0; i < 9; i++) {
         X.set(i, xs[i/3].get (i%3));
      }
      
      VectorNd[] fs = new VectorNd[] {
        new VectorNd(3), 
        new VectorNd(3), 
        new VectorNd(3)
      };
      DD0.mul (fs[0], X);   // fs[0] = DD0 * X
      DD1.mul (fs[1], X); 
      DD2.mul (fs[2], X); 
      
      VectorNd grad_f = new VectorNd(9);
      MatrixNd hess_f = new MatrixNd(9,9);
      
      ///
      
      double a = ShellUtil.area (ele.getNodes (), true); 
      
      if (this.mIsDDE) {
//         Matrix2d Gxy = new Matrix2d();
//         G.getSubMatrix (0, 0, Gxy);
//         
//         Vector4d k = stretching_stiffness_dde(Gxy);
//         k.scale (1);  // weakening_mult
//         
//         MatrixNd Du = DD0;
//         MatrixNd Dv = DD1; 
//         
//         VectorNd fuu = new VectorNd(9).mulTranspose (Du, fs[0]);
//         VectorNd fvv = new VectorNd(9).mulTranspose (Dv, fs[1]);
//         
//         VectorNd fuv_0 = new VectorNd(9).mulTranspose (Du, fs[1]); 
//         VectorNd fuv_1 = new VectorNd(9).mulTranspose (Dv, fs[0]);
//         VectorNd fuv = new VectorNd(9).add(fuv_0).add(fuv_1).scale (0.5);
//         
//         grad_f.scaledAdd (k.get (0) * G.get (0, 0), fuu);
//         grad_f.scaledAdd (k.get (2) * G.get (1, 1), fvv);
//         grad_f.scaledAdd (k.get (1) * G.get (0, 0), fvv);
//         grad_f.scaledAdd (k.get (1) * G.get (1, 1), fuu);
//         grad_f.scaledAdd (2 * k.get (3) * G.get (0, 1), fuv);
//         
//         MatrixNd fuu_x_fuu = MathUtil.outerProduct (fuu, fuu);
//         MatrixNd fvv_x_fvv = MathUtil.outerProduct (fvv, fvv);
//         MatrixNd fuu_x_fvv = MathUtil.outerProduct (fuu, fvv);
//         MatrixNd fvv_x_fuu = MathUtil.outerProduct (fvv, fuu);
//         MatrixNd fuv_x_fuv = MathUtil.outerProduct (fuv, fuv);
//         
//         MatrixNd Dut_Du = new MatrixNd(); 
//         MatrixNd Dvt_Dv = new MatrixNd(); 
//         
//         Dut_Du.mulTransposeLeft (Du, Du);
//         Dvt_Dv.mulTransposeLeft (Dv, Dv);
//         
//         hess_f.scaledAdd(k.get (0), fuu_x_fuu);
//         hess_f.scaledAdd(k.get (0) * Math.max(G.get (0, 0), 0), Dut_Du);
//         
//         hess_f.scaledAdd(k.get (2), fvv_x_fvv);
//         hess_f.scaledAdd(k.get (0) * Math.max(G.get (1, 1), 0), Dvt_Dv);
//         
//         hess_f.scaledAdd(k.get (1), fuu_x_fvv);
//         hess_f.scaledAdd(k.get (1) * Math.max(G.get (0, 0), 0), Dvt_Dv);
//         hess_f.scaledAdd(k.get (1), fvv_x_fuu);
//         hess_f.scaledAdd(k.get (1) * Math.max(G.get (1, 1), 0), Dut_Du);
//         
//         hess_f.scaledAdd(2 * k.get (3), fuv_x_fuv);
//         
//         //
//         
//         hess_f.scale(-a);
//         grad_f.scale(-a);
         throw new RuntimeException("Review code before running.");
      } else {
         // Original specifies for negative a, but causes oscillation problem.
         double gf = a * this.mAltStretching;
         
         Matrix3d Gc = new Matrix3d(); 
         MathUtil.max(G, Matrix3d.ZERO, Gc);
         
         for (int i=0; i<3; i++) {
            for(int j=0; j<=i; j++) {
               VectorNd DDit_fj = new VectorNd(9);
               VectorNd DDjt_fi = new VectorNd(9); 
               DDit_fj.mulTranspose (DDs[i], fs[j]);  // DDs[i].t() * fs[j]
               DDjt_fi.mulTranspose (DDs[j], fs[i]); 
               
               MatrixNd DDit_DDj = new MatrixNd(9, 9); 
               MatrixNd DDjt_DDi = new MatrixNd(9, 9); 
               DDit_DDj.mulTransposeLeft (DDs[i], DDs[j]);  
               DDjt_DDi.mulTransposeLeft (DDs[j], DDs[i]);   
               
               VectorNd dG = new VectorNd(9);
               dG.add(DDit_fj);
               dG.add(DDjt_fi);
               dG.scale(0.5);
               
               MatrixNd dG2 = new MatrixNd(9, 9);
               dG2.add( DDit_DDj );
               dG2.add( DDjt_DDi );
               dG2.scale(0.5);
               
               VectorNd d_trace_c = new VectorNd(dG).scale (0.5);
               
               if (i == j) {
                  grad_f.scaledAdd(
                     gf * (1 - this.mAltPoisson) * G.get (i, j), dG);
                  grad_f.scaledAdd(
                     gf * this.mAltPoisson * G.trace (), dG);
               } else {
                  grad_f.scaledAdd(
                     2.0 * gf * (1 - this.mAltPoisson) * G.get (i, j), dG);
               }
               
               MatrixNd dG_x_dG = MathUtil.outerProduct (dG, dG); // 9x9
               if (i == j) {
                  MatrixNd d_trace_c_x_dG = MathUtil.outerProduct (d_trace_c, dG);
                  
                  hess_f.scaledAdd (gf * (1 - this.mAltPoisson), dG_x_dG);
                  hess_f.scaledAdd (
                     gf * (1 - this.mAltPoisson) * Gc.get (i, j), dG2);
                  hess_f.scaledAdd (gf * this.mAltPoisson * Gc.trace (), dG2);
                  hess_f.scaledAdd (gf * this.mAltPoisson, d_trace_c_x_dG);
               } else {
                  hess_f.scaledAdd (2 * gf * (1 - this.mAltPoisson), dG_x_dG);
               }
            }
         }
      }
      
      return new Pair<MatrixNd, VectorNd>(hess_f, grad_f);
   }
   
   /* --- Tertiary Functions --- */
   
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
   protected double bending_coeff(HalfEdge edge, double theta) {
      FemNode3d edgeHead = mModel.getNode( edge.head.getIndex () );
      FemNode3d edgeTail = mModel.getNode( edge.tail.getIndex () );
      
      Point3d x0 = edgeHead.getPosition (); 
      Point3d x1 = edgeTail.getPosition ();
      
      ShellTriElement ele0 = (ShellTriElement)mModel.getShellElement ( 
         edge.getFace ().getIndex ());
      ShellTriElement ele1 = (ShellTriElement)mModel.getShellElement ( 
         edge.getOppositeFace ().getIndex ());
      
      double a = 
         ShellUtil.area (ele0.getNodes (), true) + 
         ShellUtil.area (ele1.getNodes (), true);
      
      // Edge vector norm.
      double l = new Vector3d(x1).sub (x0).norm ();
      
      double ke0 = (this.mIsDDE) ?
         bending_stiffness_dde(edge, mMatDDEBending, a, l, theta) :
         this.mAltBending;
            
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
   protected double bending_stiffness_dde(HalfEdge edge, MatrixNd data, 
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
   
   protected Vector4d stretching_stiffness_dde(Matrix2d G) {
      int nsamples = 30;
      
      double a = (G.get (0, 0) + 0.25) * nsamples;
      double b = (G.get (1, 1) + 0.25) * nsamples; 
      double c = Math.abs(G.get(0,1)) * nsamples;
      
      a = MathUtil.clamp (a, 0, nsamples-1-1e-5); 
      b = MathUtil.clamp (b, 0, nsamples-1-1e-5); 
      c = MathUtil.clamp (c, 0, nsamples-1-1e-5);
      
      int ai = (int) Math.floor (a);
      int bi = (int) Math.floor (b);
      int ci = (int) Math.floor (c); 
      
      if (ai < 0) ai = 0; 
      if (bi < 0) bi = 0; 
      if (ci < 0) ci = 0; 
      
      if (ai > nsamples-2) ai = nsamples-2; 
      if (bi > nsamples-2) bi = nsamples-2;
      if (ci > nsamples-2) ci = nsamples-2;
      
      a = a - ai; 
      b = b - bi; 
      c = c - ci; 
      
      double[][][] weight = new double[2][2][2];
      weight[0][0][0]=(1-a)*(1-b)*(1-c);
      weight[0][0][1]=(1-a)*(1-b)*(  c);
      weight[0][1][0]=(1-a)*(  b)*(1-c);
      weight[0][1][1]=(1-a)*(  b)*(  c);
      weight[1][0][0]=(  a)*(1-b)*(1-c);
      weight[1][0][1]=(  a)*(1-b)*(  c);
      weight[1][1][0]=(  a)*(  b)*(1-c);
      weight[1][1][1]=(  a)*(  b)*(  c);
      
      Vector4d stiffness = new Vector4d();
      for(int i=0; i<2; i++)
          for(int j=0; j<2; j++)
              for(int k=0; k<2; k++)
                  for(int l=0; l<4; l++)
                      {
                          double lval = stiffness.get (l);
                          stiffness.set(l, lval + mMatDDEStretchingSamples[ai+i][bi+j][ci+k][l]*weight[i][j][k]);
                      }
      
      return stiffness; 
   }

   /** 
    * Compute the element-wise deformation gradient.
    * 
    * @return
    * F and invDm. 
    */
   protected Pair<Matrix3d, Matrix3d> deformation_gradient(Face face) {
      // Collect nodal positions and element normal.
      
      FemNode3d[] nodes = mModel.getShellElement (face.getIndex ()).getNodes (); 
      
      Point3d[] xs = new Point3d[] {
         nodes[0].getPosition (),
         nodes[1].getPosition (), 
         nodes[2].getPosition ()
      };
      
      Point3d[] us = new Point3d[] {
         nodes[0].getRestPosition (),
         nodes[1].getRestPosition (),
         nodes[2].getRestPosition ()
      };
      
      ShellTriElement ele = (ShellTriElement)mModel.getShellElement ( 
         face.getIndex ());
      
      Vector3d nrm = ShellUtil.getNormal (ele, false);
      Vector3d nrmRest = ShellUtil.getNormal (ele, true);
      
      // Compute element derivative (world-space).
      Matrix3d d = derivative(xs[0], xs[1], xs[2], nrm);
      
      // Compute element derivative (material-space). compute_ms_data
      Matrix3d dm = derivative(us[0], us[1], us[2], nrmRest);  
      
      // d / dm 
      Matrix3d invDm = new Matrix3d();
      double det = invDm.fastInvert (dm);
      if (det == 0) {
         throw new RuntimeException("Failed to invert dm.");
      }
      Matrix3d F = new Matrix3d();
      F.mul(d, invDm);
      
      return new Pair<Matrix3d, Matrix3d>(F, invDm);
   }
   
   /**
    * Compute the positional derivative of the given face. 
    * 
    * @param w0 
    * @param w1 
    * @param w2 
    * @param dz Face normal.
    * 
    * @return 3x3 derivative. 
    */
   protected Matrix3d derivative(
      Vector3d w0, Vector3d w1, Vector3d w2, Vector3d dz) 
   {
      Vector3d w1_w0 = new Vector3d(w1).sub(w0);
      Vector3d w2_w0 = new Vector3d(w2).sub(w0);
      
      Matrix3d d = new Matrix3d();
      d.setColumn (0, w1_w0);
      d.setColumn (1, w2_w0);
      d.setColumn (2, dz);
      
      return d; 
   }
   
   // --- Remeshing Utilities --- //
   
   /**
    * Get a matrix representation of the given face's edge strain.
    * 
    * plasticity.cpp::edges_to_face
    * 
    * @param face
    * @return
    */
   public Matrix3d bendStrain_edgesToFace(Face face) {
      Matrix3d S = new Matrix3d();
      
      ShellTriElement ele = (ShellTriElement)mModel.getShellElement (
         face.getIndex ());
      Vector3d nrmRest = ShellUtil.getNormal (ele, true);
      
      for (int e = 0; e < 3; e++) {
         HalfEdge edge = face.getEdge (e);
         
         int h = edge.head.getIndex ();
         int t = edge.tail.getIndex ();
         
         FemNode3d hNode = mModel.getNode (h);
         FemNode3d tNode = mModel.getNode (t);
         
         Point3d hu = hNode.getRestPosition ();
         Point3d tu = tNode.getRestPosition ();
         
         Vector3d e_mat = new Vector3d(hu).sub(tu);
         Vector3d t_mat = new Vector3d(e_mat).normalize ().cross (nrmRest);
         
         EdgeData edgeData = mModel.myEdgeDataMap.get (edge);
         double angStrain = (edgeData != null) ? edgeData.mAngStrain : 0;
         
         Matrix3d S_step = MathUtil.outerProduct (t_mat, t_mat);
         S_step.scale (0.5 * angStrain * e_mat.norm ());
         
         S.sub (S_step);
      }
      
      double areaRest = ShellUtil.area (ele.getNodes (), true);
      S.scale (1 / areaRest);
      
      return S;
   }
   
   /**
    * Convert a matrix-representation of the given face's edge strain into 
    * a Vec3 where each element corresponds to an edge.
    * 
    * plasticity.cpp::face_to_edges
    * 
    * @param face
    * @param S
    * @return
    */
   public Vector3d bendStrain_faceToEdges(Face face, Matrix3d S) {
      ShellTriElement ele = (ShellTriElement)mModel.getShellElement (face.getIndex ());
      Vector3d nrm = ShellUtil.getNormal (ele, true);
      
      Matrix6x3 A = new Matrix6x3();
      
      for (int e = 0; e < 3; e++) {
         HalfEdge edge = face.getEdge (e);
         
         int h = edge.head.getIndex ();
         int t = edge.tail.getIndex ();
         
         FemNode3d hNode = mModel.getNode (h);
         FemNode3d tNode = mModel.getNode (t);
         
         Point3d hu = hNode.getRestPosition ();
         Point3d tu = tNode.getRestPosition ();
         
         Vector3d e_mat = new Vector3d(hu).sub(tu);
         Vector3d t_mat = new Vector3d(e_mat).normalize ().cross (nrm);
         
         // Se
         Matrix3d Se = MathUtil.outerProduct (t_mat, t_mat);
         Se.scale (-0.5 * e_mat.norm ());
         
         A.setColumn (e, new double[] {
            Se.get (0,0), Se.get (1,1), Se.get(2,2), 
            Se.get(0,1), Se.get(0,2), Se.get(1,2)
         });
      }

      double areaRest = ShellUtil.area (ele.getNodes (), true);
      VectorNd y = new VectorNd(
         S.get(0,0), S.get(1,1), S.get (2,2), 
         S.get (0,1), S.get (0,2), S.get (1,2));
      y.scale (areaRest);
      
      VectorNd rv = SolverUtil.solve_llsq (A, y);
      return new Vector3d(rv);
   }
   

}

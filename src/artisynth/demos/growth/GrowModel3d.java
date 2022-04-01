package artisynth.demos.growth;

import artisynth.core.femmodels.FemDeformedPoint;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.WedgeElement;
import artisynth.demos.growth.models.ts.evouga.StiffnessMatrixUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.render.Renderer;
import maspack.render.Renderer.DrawMode;

/** 
 * Extension of FemModel3d to account for growth. 
 *  
 * Currently only contains modifications relating to display-purposes. 
 */
public class GrowModel3d extends FemModel3d {
   
   public GrowModel3d () {
      super();
   }
   
   /** Logic to draw the directors. */
   public void render(Renderer renderer, int flags) {
      if (myDirectorRenderLen > 0) {
         renderer.beginDraw (DrawMode.LINES);
         renderer.setLineWidth (myRenderProps.getLineWidth());
         renderer.setColor (myRenderProps.getLineColor());
         for (FemNode3d n : myNodes) {
            if (n.hasDirector()) {
               // Draw an arrow from the back node to the front node.
               float[] x = n.myRenderCoords;
               float[] y = n.getBackNode ().getRenderCoords ();
               renderer.addVertex (
                  x[0], x[1], x[2]);
               renderer.addVertex (
                  y[0], y[1], y[2]);
            }
         }
         renderer.endDraw ();
      }
   }
   
   
   
   /* --- Nodal Stress --- */
   
   /** Get the (interpolated) plastic deformation gradient at each node. */
   public Matrix3d[] getNodalPlasticDeformationGradient () {
      Matrix3d[] nodalFgs = new Matrix3d[numNodes()];
      for (int n = 0; n < numNodes(); n++) {
         nodalFgs[n] = new Matrix3d();
      }
      
      for (ShellElement3d ele : getShellElements ()) {
         GrowTriElement gEle = (GrowTriElement)ele; 
         
         IntegrationPoint3d[] ipnts = gEle.getIntegrationPoints();
         GrowIntegrationData3d[] idata = gEle.getIntegrationData();
         MatrixNd nodalExtrapMat = gEle.getNodalExtrapolationMatrix();
         
         // For each integration point
         for (int k=0; k<ipnts.length; k++) {
            // For each node
            for (int en = 0; en < gEle.numNodes (); en++) {  
               FemNode3d node = gEle.getNodes ()[en];
               
               // % contribute of this integration pt to this node
               double a = nodalExtrapMat.get (en, k);
               
               if (a != 0) {
                  // "MyNode_#33" -> 33 begins at string index 8         
                  int nodeGlobalIdx = ShellUtil.getIndex (gEle.getNodes ()[en]);
                  nodalFgs[nodeGlobalIdx].scaledAdd (
                     a/node.numAdjacentElements(), 
                     idata[k].getFp () != null 
                        ? idata[k].getFp () 
                        : Matrix3d.IDENTITY);
               }
            }
         }
      }
      
      return nodalFgs;
   }

   protected WedgeElement mSampleWedgeElement = new WedgeElement();
   /** 
    * Get the (interpolated) residual bending plastic stress at each node.
    */
   public double[] getNodalResidualPlasticBendingStrain () {
      double[] nodalRS = new double[numNodes()];
      
      Matrix3d F = new Matrix3d();
      Matrix3d invJ0 = new Matrix3d();
      
      double avgDetRemain = 0;
      double avgExpDet = 0;
      
      for (FemElement3dBase ele : this.getAllElements ()) {
         GrowElementBase gEle = (GrowElementBase)ele; 
         
         IntegrationPoint3d[] ipnts = null;
         GrowIntegrationData3d[] idata = gEle.getIntegrationData();
         MatrixNd nodalExtrapMat = null;
         
         boolean isShellEle = (gEle instanceof GrowTriElement);
         
         if (isShellEle) {
            GrowTriElement sEle = (GrowTriElement) gEle;
            ipnts = sEle.getIntegrationPoints();
         } else {
            GrowWedgeElement vEle = (GrowWedgeElement) gEle;
            ipnts = vEle.getIntegrationPoints ();
         }
         
         nodalExtrapMat = mSampleWedgeElement.getNodalExtrapolationMatrix();

         // For each integration point
         for (int k=0; k<ipnts.length; k++) {
            
            // Only measure residual strain in bending-surface.
            if (k >= 3) {
               continue;
            }
            
            // Calculate F
            double detInvJ0 = ipnts[k].computeInverseRestJacobian (invJ0, gEle.getNodes ());
            if (detInvJ0 <= 0) {
               throw new RuntimeException("Detected negative detInvJ0.");
            }
            double detF = ipnts[k].computeGradient (F, gEle.getNodes (), invJ0);
            if (detF <= 0) {
               throw new RuntimeException("Detected negative detF.");
            }
            
            // Expected F 
            double detExpF = idata[k].getFp ().determinant ();
            double RS = detExpF - detF;
            
            avgDetRemain += (RS) * (1.0/3);
            avgExpDet += detExpF * (1.0/3);
            
            // For each node
            for (int en = 0; en < ele.numNodes (); en++) {
               FemNode3d node = gEle.getNodes ()[en];
               if (!isShellEle && !ShellUtil.isVolBackNode (node)) {
                  continue;
               }
               
               // % contribute of this integration pt to this node
               int vNodeIdx = en;
               if (isShellEle) {
                  vNodeIdx += 3;
               }
               
               double a = nodalExtrapMat.get (vNodeIdx, k);
               if (a == 0) {
                  continue;
               }
               
//               if (a < 0 || RS < 0) {
//                  throw new RuntimeException("");
//               }
               
               int nodeGlobalIdx = node.getIndex ();
               
               double inc = Math.abs(a)/node.numAdjacentElements() * RS;
               
//               if (!node.hasDirector ()) {
//                  inc /= 5;
//               }
               
               nodalRS[nodeGlobalIdx] += inc;
            }
         }
         
      }

      avgDetRemain /= this.numAllElements ();
      avgExpDet /= this.numAllElements ();
      System.out.printf ("avgDetRemain: [%.6f], avgExpDet: [%.6f]\n", avgDetRemain, avgExpDet);
      
      return nodalRS;
   }
   
   protected FemDeformedPoint createFemDeformedPoint() {
      return new GrowDeformedPoint();
   }
   
   // Discrete Shell. Ignore edge delegate nodes.
   
   public void addSolveBlocks(SparseNumberedBlockMatrix S) {
      setNodalIncompBlocksAllocated(getSoftIncompMethod() == IncompMethod.NODAL);

      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d node = myNodes.get(i);
         for (FemNodeNeighbor nbr : getNodeNeighbors(node)) {
            nbr.addSolveBlocks (S, node);
         }
         // used for soft nodal-based incompressibilty:
         for (FemNodeNeighbor nbr : getIndirectNeighbors(node)) {
            if (nbr.getNode ().checkFlag (StiffnessMatrixUtil.EDGE_DELEGATE_FLAG)) {
               continue;
            }
            nbr.addSolveBlocks (S, node);
         }        
      }
      // System.out.println ("sparsity=\n" + S.getBlockPattern());
   }
   
   public void addVelJacobian(
      SparseNumberedBlockMatrix M, double s) {

      if (!myStressesValidP || !myStiffnessesValidP) {
         updateStressAndStiffness();
      }
      double sm = -s*myMassDamping;
      double sk = -s*myStiffnessDamping;
      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d node = myNodes.get(i);
         if (node.getLocalSolveIndex() != -1) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(node)) {
               //addNeighborVelJacobian(M, node, nbr, s);
               nbr.addVelJacobian (M, node, sm, sk, myUseConsistentMass);
            }
            // used for soft nodal-based incompressibilty:
            for (FemNodeNeighbor nbr : getIndirectNeighbors(node)) {
               //addNeighborVelJacobian(M, node, nbr, s);
               if (nbr.getNode ().checkFlag (StiffnessMatrixUtil.EDGE_DELEGATE_FLAG)) {
                  continue;
               }
               nbr.addVelJacobian (M, node, sm, sk, false);
            }
         }
      }
   }

   public void addPosJacobian(
      SparseNumberedBlockMatrix M, double s) {

      if (!myStressesValidP || !myStiffnessesValidP) {
         updateStressAndStiffness();
      }
      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d node = myNodes.get(i);
         if (node.getLocalSolveIndex() != -1) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(node)) {
               nbr.addPosJacobian (M, node, -s);
            }
            // used for soft nodal-based incompressibilty:
            for (FemNodeNeighbor nbr : getIndirectNeighbors(node)) {
               if (nbr.getNode ().checkFlag (StiffnessMatrixUtil.EDGE_DELEGATE_FLAG)) {
                  continue;
               }
               nbr.addPosJacobian (M, node, -s);
            }
         }
      }
      // System.out.println ("symmetric=" + mySolveMatrix.isSymmetric(1e-6));
   }
}

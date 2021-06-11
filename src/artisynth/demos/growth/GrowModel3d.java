package artisynth.demos.growth;

import artisynth.core.femmodels.FemDeformedPoint;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.ShellElement3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.render.Renderer;
import maspack.render.Renderer.DrawMode;
import maspack.util.DataBuffer;

/** 
 * Extension of FemModel3d to account for growth. 
 *  
 * Currently only contains modifications for aesthetics. 
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
                  int nodeGlobalIdx = Integer.parseInt ( 
                     gEle.getNodes ()[en].getName ().substring (8) );    
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
   
   protected FemDeformedPoint createFemDeformedPoint() {
      return new GrowDeformedPoint();
   }
}

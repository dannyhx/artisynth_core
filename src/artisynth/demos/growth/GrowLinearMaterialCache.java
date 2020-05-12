package artisynth.demos.growth;

import artisynth.core.femmodels.FemDeformedPoint;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemUtilities;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.LinearMaterialCache;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.materials.FemMaterial;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/** Growth extension of LinearMaterialCache. Used, but currently doesn't have 
 *  any modifications. */
public class GrowLinearMaterialCache extends LinearMaterialCache {

   public GrowLinearMaterialCache (FemElement3dBase e) {
      super (e);
   }

   protected GrowDeformedPoint createDeformedPoint() {
      return new GrowDeformedPoint();
   }
  
   public void clearInitialStiffness() {
      for (int i=0; i<nnodes*nnodes; i++) {
         K00[i].setZero();
      }
      for (int i=0; i<nnodes; ++i) {
         f0[i].setZero();
      }
      if (hasShellData()) {
         for (int i=0; i<nnodes*nnodes; i++) {
            K01[i].setZero();
            K10[i].setZero();
            K11[i].setZero();
         }
         for (int i=0; i<nnodes; ++i) {
            f1[i].setZero();
         }
      }
   }
   
   public void addInitialShellStiffness (
   ShellElement3d e, FemMaterial mat, double weight) {
      
      FemDeformedPoint dpnt = createDeformedPoint();
      FemNode3d[] nodes = e.getNodes();
      
      // compute stiffness matrix
      Matrix6d D = new Matrix6d();
      SymmetricMatrix3d stress = new SymmetricMatrix3d();
      IntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();
      
      int nump = e.numPlanarIntegrationPoints();
      for (int k=0; k<ipnts.length; k++) {
         IntegrationPoint3d pt = ipnts[k];
         IntegrationData3d dt = idata[k];
         RotationMatrix3d R = null; // used if element has prestrain
         
         dpnt.setFromRestPoint (pt, dt, RotationMatrix3d.IDENTITY, e, k%nump);

         double dv0 = dt.getDetJ0()*weight*pt.getWeight();
         double t = pt.getCoords().z;

         Matrix3d Q = dt.getFrame() == null ? Matrix3d.IDENTITY : dt.getFrame();

         mat.computeStressAndTangent (stress, D, dpnt, Q, 0.0, null);
         VectorNd Ns = pt.getShapeWeights ();
         Vector3d[] dNs = pt.getGNs();
         for (int i = 0; i < nodes.length; i++) {
            // normally stress will be zero, unless there is prestrain ...
            double iN = Ns.get(i);
            Vector3d idN = dNs[i];
            
            FemUtilities.addShellStressForce(
               f0[i], f1[i], stress, t, dv0, iN, idN.x, idN.y, dt.getInvJ0());
            
            for (int j = 0; j < nodes.length; j++) {
               double jN = Ns.get(j);
               Vector3d jdN = dNs[j];
               // XXX should presumably use stress instead of
               // SymmetricMatrix3d.ZERO, but results are unstable
               FemUtilities.addShellMaterialStiffness (
                  K00[i*nnodes+j], K01[i*nnodes+j],
                  K10[i*nnodes+j], K11[i*nnodes+j],
                  iN, jN, idN, jdN, dv0, t,
                  dt.getInvJ0(), SymmetricMatrix3d.ZERO, D);
            }
         }
      }      
     
      // initial RHS
      Vector3d tmp0 = new Vector3d();
      Vector3d tmp1 = new Vector3d();
      for (int i = 0; i < nodes.length; i++) {
         tmp0.setZero();
         tmp1.setZero();
         for (int j=0; j<nodes.length; j++) {
            Vector3d pos = nodes[j].getRestPosition();
            Vector3d backPos = nodes[j].getBackRestPosition();
            mulAddK (i, j, tmp0, tmp1, pos, backPos);
         }
         f0[i].sub (tmp0, f0[i]);
         f1[i].sub (tmp1, f1[i]);
      }
   }
   

}

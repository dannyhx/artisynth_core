package artisynth.demos.growth.models.ts;

import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.demos.growth.remesh.RemeshOps.OpRv;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix3d;

public abstract class ThinShellBase {
   
   protected FemModel3d mModel;
   protected PolygonalMesh mMesh; 
   
   public ThinShellBase(FemModel3d model, PolygonalMesh mesh) {
      this.mModel = model;
      this.mMesh = mesh;
   }
   
   public abstract void setMaterialProperties(
      double youngsModulus, double poissonsRatio, double thickness);
   
   public abstract void addForceAndStiffness();
   
   // Useful if shell implementation has its own advance strategy.
   public void advance() {
      
   }
   
   /** Remeshing **/
   
   public void remeshPreOp() {}
   public void remeshPostOp(boolean isEleModified) {}
   
   /** Remeshing Operations **/
   
   public void remeshRemoveFacePreOp(Face face, OpRv opRv) {}
   
   /* --- Morphogen2Growth --- */
   
   public void applyGrowthTensorToEle(
      FemElement3dBase ele, boolean isBendingMorphogenHack, 
      Matrix3d fixedBendingStrain) {};
      
   public void unapplyGrowthTensors() {};
   
   /* --- Plasticity --- */
   
   public void useResidualPlasticStrain() {};
}

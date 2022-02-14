package artisynth.demos.growth.models.ts;

import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.demos.growth.models.ts.chen.DiscreteShell;
import artisynth.demos.growth.models.ts.narain.NarainShell;
import artisynth.demos.growth.remesh.RemeshOps.OpRv;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix3d;

public class ThinShellAux {
   public ThinShellType mType;
   protected ThinShellBase mTS;
   
   public ThinShellAux(ThinShellType type, FemModel3d model, PolygonalMesh mesh) {
      this.mType = type; 
      
      if (type == ThinShellType.NARAIN) {
         this.mTS = new NarainShell(model, mesh);
      } 
      else if (type == ThinShellType.CHEN) {
         this.mTS = new DiscreteShell(model, mesh);
      } else {
         throw new UnsupportedOperationException("Unimplemented");
      }
   }
   
   public void setMaterialProperties(
      double youngsModulus, double poissonsRatio, double thickness) 
   {
      this.mTS.setMaterialProperties (youngsModulus, poissonsRatio, thickness);
   }
   
   public void addForceAndStiffness() {
      this.mTS.addForceAndStiffness ();
   }   
   
   public void advance() {
      this.mTS.advance ();
   }
   
   /** Remeshing **/
   
   public void remeshPreOp() {
      this.mTS.remeshPreOp ();
   }
   
   public void remeshPostOp(boolean isEleModified) {
      this.mTS.remeshPostOp (isEleModified);
   }
   
   /** Remeshing Operations **/
   
   public void remeshRemoveFacePreOp(Face face, OpRv opRv) {
      this.mTS.remeshRemoveFacePreOp (face, opRv);
   }

   /* --- Morphogen2Growth --- */
   
   public void applyGrowthTensorToEle(FemElement3dBase ele, 
      boolean isBendingMorphogenHack, Matrix3d fixedBendingStrain) 
   {
      this.mTS.applyGrowthTensorToEle (
         ele, isBendingMorphogenHack, fixedBendingStrain);
   }
      
   public void unapplyGrowthTensors() {
      this.mTS.unapplyGrowthTensors ();
   };
   
   /* --- Plasticity --- */
   
   public void useResidualPlasticStrain() {
      this.mTS.useResidualPlasticStrain ();
   };
}

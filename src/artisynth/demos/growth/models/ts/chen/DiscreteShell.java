package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.FemModel3d;
import artisynth.demos.growth.models.ts.ThinShellBase;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;

public class DiscreteShell extends ThinShellBase {
   
   protected DiscreteShellMaterial Material;
   
   public DiscreteShell(FemModel3d model, PolygonalMesh mesh) {
     super(model, mesh);
     this.setMaterialProperties (0, 0, 0);
   }
   


   @Override
   public void setMaterialProperties (
      double youngsModulus, double poissonsRatio, double thickness) {
      this.Material = new StVKMaterial();
   }

   @Override
   public void addStretchingForceAndStiffness () {
   }

   @Override
   public void addBendingForceAndStiffness () {
      // TODO Auto-generated method stub
   }
   
   
}

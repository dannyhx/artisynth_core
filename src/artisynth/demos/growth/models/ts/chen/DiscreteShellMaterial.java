package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.FemModel3d;
import maspack.geometry.Face;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public abstract class DiscreteShellMaterial {
   public abstract double stretchingEnergy(
      MeshConnectivity MC, 
      FemModel3d model,
      RestState rs, 
      int f,
      VectorNd derivative, 
      MatrixNd hessian   
   );
   
   public abstract double bendingEnergy(
      FemModel3d model,
      VectorNd extraDOFs,
      MeshConnectivity MC,
      RestState rs, 
      Face face,
      int f,
      int numExtraDOFs,
      VectorNd derivative, 
      MatrixNd hessian
   );
}

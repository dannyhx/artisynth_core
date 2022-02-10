package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.ShellElement3d;
import maspack.geometry.Face;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public abstract class DiscreteShellMaterial {
   public abstract double stretchingEnergy(
      ShellElement3d ele, 
      RestState rs, 
      int f,
      VectorNd derivative, 
      MatrixNd hessian   
   );
   
   public abstract double bendingEnergy(
      FemModel3d model,
      ShellElement3d ele, 
      RestState rs, 
      Face face,
      int f,
      int numExtraDOFs,
      VectorNd derivative, 
      MatrixNd hessian
   );
}

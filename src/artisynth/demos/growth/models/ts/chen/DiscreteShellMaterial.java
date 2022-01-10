package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.ShellElement3d;
import maspack.matrix.MatrixNd;

public abstract class DiscreteShellMaterial {
   public class StretchingEnergyRv {
      double Result;
      MatrixNd Derivative;  // 1x9 
      MatrixNd Hessian;     // 9x9
   }
   
   public abstract StretchingEnergyRv stretchingEnergy(ShellElement3d ele, RestState rs, int f);
}

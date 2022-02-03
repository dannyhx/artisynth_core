package artisynth.demos.growth.models.ts.chen;

import artisynth.core.femmodels.ShellElement3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public abstract class DiscreteShellMaterial {
   public class StretchingEnergyRv {
      double Result;
      VectorNd Derivative;  // 1x9 
      MatrixNd Hessian;     // 9x9
   }
   
   public abstract StretchingEnergyRv stretchingEnergy(
      ShellElement3d ele, 
      RestState rs, 
      int f,
      VectorNd derivative, 
      MatrixNd hessian   
   );
   
   public class BendingEnergyRv {
      double Result;
      VectorNd Derivative;  
      MatrixNd Hessian;     
   }
}

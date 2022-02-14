package artisynth.demos.growth.models.ts.chen;

import java.util.ArrayList;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.ShellElement3d;
import maspack.matrix.Matrix2d;

/**
 * RestState.h :: MonolayerRestState 
 */
public class MonolayerRestState extends RestState {
   public ArrayList<Double> thicknesses;
   
   // First fundamental form, in barycentric coordinates for each mesh face.
   public ArrayList<Matrix2d> abars;
   
   // Second fundamental form, in barycentric coordinates for each mesh face.
   public ArrayList<Matrix2d> bbars;

   public MonolayerRestState(FemModel3d model, double thickness) {
      int nfaces = model.numShellElements ();
      
      this.thicknesses = new ArrayList<Double>(nfaces);
      this.abars = new ArrayList<Matrix2d>(nfaces);
      this.bbars = new ArrayList<Matrix2d>(nfaces);
      
      for (int f = 0; f < model.numShellElements (); f++) {
         ShellElement3d ele = model.getShellElement (f);
         
         this.thicknesses.add (thickness);
         
         // main.cpp::runSimulation
         Matrix2d I = GeometryDerivative.firstFundamentalForm (ele, null, null);
         this.abars.add (I);
         
         // Flat.
         this.bbars.add (new Matrix2d());
      }
   }
}
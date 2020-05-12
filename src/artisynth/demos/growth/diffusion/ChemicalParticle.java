package artisynth.demos.growth.diffusion;

import artisynth.core.mechmodels.Particle;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;

/** A particle that has its own chemicals. */
public class ChemicalParticle extends Particle {
   
   /** Chemical concentration for each chemical. */
   protected VectorNd mChems;
   
   public ChemicalParticle(double mass, Point3d pt, int numChems) {
      super(mass, pt);
      mChems = new VectorNd(numChems);
   }
   
   public void setChems(VectorNd chems) {
      mChems = chems;
   }
   
   public double getChem0() {
      return mChems.get (0);
   }
   
   public void setChem0(double amt) {
      mChems.set (0, amt);
   }
   
   public double getChem1() {
      if (mChems.size() < 2) {
         return 0;
      }
      
      return mChems.get (1);
   }
   
   public void setChem1(double amt) {
      mChems.set (1, amt);
   }
   
   public double getChem(int chemIdx) {
      return mChems.get (chemIdx);
   }

   public void setChem (int chemIdx, double amt) {
      mChems.set (chemIdx, amt);
   }

   public static PropertyList myProps =
      new PropertyList (ChemicalParticle.class, Particle.class);

   static {
      myProps.add ("chem0 * *", "Concentration of 1st chemical", 0);
      myProps.add ("chem1 * *", "Concentration of 2nd chemical", 0);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   
}

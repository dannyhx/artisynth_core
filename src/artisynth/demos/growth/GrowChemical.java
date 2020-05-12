package artisynth.demos.growth;

public enum GrowChemical {
   PAR(0),
   PER(1),
   NOR(2);
   public final static int NUM_TYPES = 3;
   
   /**
    * Corresponding index of given growth morphogen.
    */
   public final int mIdx;
   
   private GrowChemical(int idx) {
      mIdx = idx;
   }
}

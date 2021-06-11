package artisynth.demos.growth;

import artisynth.core.femmodels.BackNode3d;
import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;

/** Extension of FemNode3d to account for growth. */
public class GrowNode3d extends FemNode3d {
   
   public boolean mIsMembrane = false;
   
   // --- Backup of the world-space. --- //
   
   public Point3d m_WS_front_pos = new Point3d();
   public Point3d m_WS_front_restPos = new Point3d();
   public Vector3d m_WS_front_vel = new Vector3d();
   
   public Point3d m_WS_back_pos = new Point3d();
   public Point3d m_WS_back_restPos = new Point3d();
   public Vector3d m_WS_back_vel = new Vector3d();
   
   // --- Velocity associated with plasticity. --- //
   
   public Vector3d m_ES_front_vel = new Vector3d();
   public Vector3d m_ES_back_vel = new Vector3d();
   
   // --- Other --- //
  
   /** Concentration of each morphogen type. */
   public VectorNd mChems;
   
   /** If true, this node has a permanent morphogen concentration. */
   public boolean mIsMorphogenSrc = false;
   
   /** If true, morphogen is always zeroed at this node. */
   public boolean mIsNoMorphogenZone = false;
   
   
   
   
   public GrowNode3d() {
      super();
      setDirectorActive(true);
   }
   
   public GrowNode3d(boolean isMembrane) {
      super();
      mIsMembrane = isMembrane;
      
      if (!mIsMembrane)
         setDirectorActive(true);
   }
   
   public GrowNode3d (Point3d p, VectorNd chems) {
      super(p);
      mChems = chems;
      
      setDirectorActive(true);
   }

   public GrowNode3d (Point3d p, VectorNd chems, boolean isMembrane) {
      super(p);
      mChems = chems;
      mIsMembrane = isMembrane;
      
      if (!mIsMembrane)
         setDirectorActive(true);
   }
   
   public GrowNode3d (double x, double y, double z, VectorNd chems) {
      super(x,y,z); 
      mChems = chems;
      
      if (!mIsMembrane)
         setDirectorActive(true);
   }
   

   
   /* --- Properties --- */
   
   public static PropertyList myProps =
   new PropertyList (GrowNode3d.class, FemNode3d.class);

   static {
      myProps.add ("chem0 * *", "Concentration of 1st chemical", 0);
      myProps.add ("chem1 * *", "Concentration of 2nd chemical", 0);
      myProps.add ("chem2 * *", "Concentration of 3rd chemical", 0);
      myProps.add ("chem3 * *", "Concentration of 4th chemical", 0);
      myProps.add ("isMorphogenSrc * *", "", false);
      myProps.add ("isNoMorphogenZone * *", "", false);
   }
   
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   
   public double getChem0() { return mChems.get (0); }
   public void setChem0(double amt) { mChems.set (0, amt); }
   public double getChem1() { return mChems.get (1); }
   public void setChem1(double amt) { mChems.set (1, amt); }
   public double getChem2() { return mChems.get (2); }
   public void setChem2(double amt) { mChems.set (2, amt); }
   public double getChem3() { return mChems.get (3); }
   public void setChem3(double amt) { mChems.set (3, amt); }  
   
   public boolean getIsMorphogenSrc() { return mIsMorphogenSrc; }
   public void setIsMorphogenSrc(boolean val) { mIsMorphogenSrc = val; }   
   
   public boolean getIsNoMorphogenZone() { return mIsNoMorphogenZone; }
   public void setIsNoMorphogenZone(boolean val) { mIsNoMorphogenZone = val; } 
   
   /* --- Setters and Getters --- */
   
   public double getGrowChem(GrowChemical chemType) {
      return mChems.get (chemType.mIdx);
   }
   
   public void setGrowChemsZero() {
      mChems.set (GrowChemical.PAR.mIdx, 0);
      mChems.set (GrowChemical.PER.mIdx, 0);
      mChems.set (GrowChemical.NOR.mIdx, 0);
   }
   
   

   /* --- Director --- */
   
   /**
    * Overridden such that backnode isn't removed when its element(s) is being
    * disconnected.
    */
   protected void setDirectorActive (boolean active) {
      BackNode3d backNode = getBackNode();
      super.setDirectorActive (active);
      
      if (active == false) {
         myBackNode = backNode;
      }
   }
   
   /**
    * Modified version where normal contribution from each adjacent element
    * is scaled via angle instead of area. This produces a more even result,
    * and is essential when remeshing an asymmetrical mesh into its
    * symmetrical form.
    */
   public void computeRestDirector (Vector3d dir) {
      double thickness = 0;
      int ecnt = 0;

      for (FemElement e : this.getAdjacentShellElements ()) {
         ShellElement3d se = (ShellElement3d)e;

         Vector3d d = new Vector3d();
         se.computeRestNodeNormal (d, this);
         
         // Scale by angle rather than area
         double scale = getAngle(this, (ShellTriElement)se);
         
         dir.scaledAdd (scale, d);
         thickness += se.getDefaultThickness();
         ecnt++;
      }
      if (ecnt > 0) {
         dir.normalize();
         dir.scale (thickness/ecnt);         
      }
   }
   
   /**
    * Get the angle (in radians) at node A relative to a 3-node element.
    */
   protected double getAngle(FemNode3d A, ShellTriElement ele) {
      FemNode3d B = null;
      FemNode3d C = null;
      for (FemNode3d node : ele.getNodes ()) {
         if (node.getNumber () != A.getNumber ()) {
            if (B == null) 
               B = node; 
            else 
               C = node;
         }
      }
      
      Vector3d BA = new Vector3d(B.getRestPosition ()).sub (A.getRestPosition ());
      Vector3d CA = new Vector3d(C.getRestPosition ()).sub (A.getRestPosition ());
      
      return BA.angle (CA);
   }
}

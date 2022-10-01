package artisynth.demos.growth;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.WedgeElement;
import maspack.matrix.Matrix3d;

public class GrowWedgeElement extends WedgeElement implements GrowElementBase {

   /** Primary direction of growth. */
   protected PolarityElementAux mPolAux = new PolarityElementAux ();

   /* --- Constructor --- */

   public GrowWedgeElement (GrowNode3d p0, GrowNode3d p1, GrowNode3d p2,
   GrowNode3d p3, GrowNode3d p4, GrowNode3d p5) {
      super (p0, p1, p2, p3, p4, p5);

      // Force GrowNode3d[]
      setNodes (p0, p1, p2, p3, p4, p5);

      Matrix3d Fg = new Matrix3d ();
      Fg.setIdentity ();
      setPlasticDeformation (Fg);
   }

   /* --- Plastic Embedding --- */

   public void useResidualPlasticStrain () {
      GrowElementBase.useResidualPlasticStrain_df (this);
   }

   /* --- Nodes --- */

   /**
    * We cannot override ShellTriElement() to initialize myNodes as GrowNode[],
    * so use this instead.
    */
   public void setNodes (
      FemNode3d p0, FemNode3d p1, FemNode3d p2, FemNode3d p3, FemNode3d p4,
      FemNode3d p5) {
      myNodes =
         new GrowNode3d[] { (GrowNode3d)p0, (GrowNode3d)p1, (GrowNode3d)p2,
                            (GrowNode3d)p3, (GrowNode3d)p4, (GrowNode3d)p5 };
      super.setNodes (myNodes);
   }

   public GrowNode3d[] getNodes () {
      return (GrowNode3d[])super.getNodes ();
   }

   /* --- Stiffness Warper 3d --- */

   public GrowStiffnessWarper3d createStiffnessWarper () {
      return GrowElementBase.createStiffnessWarper_df (this);
   }

   /* --- Integration Data --- */

   /**
    * Overridden to ensure warping point is a GrowIntegrationData3d rather than
    * the plain IntegrationData3d.
    */
   public GrowIntegrationData3d getWarpingData () {
      return GrowElementBase.getWarpingData_df (this);
   }

   public GrowIntegrationData3d[] getIntegrationData () {
      return (GrowIntegrationData3d[])super.getIntegrationData ();
   }

   public GrowIntegrationData3d[] doGetIntegrationData () {
      return GrowElementBase.doGetIntegrationData_df (this);
   }

   /* --- Volume --- */

   /**
    * Compute the volume of the element. Necessary to update its mass.
    *
    * When computing the rest volume, the rest volume is scaled by the plastic
    * strain. This is ensure that the mass still increases, even if the plastic
    * strain cannot be "relaxed".
    */
   public double _computeVolume (boolean isRest) {
      return GrowElementBase._computeVolume_df (this, isRest);
   }

   /* --- Misc Methods --- */

   public GrowDeformedPoint createDeformedPoint () {
      return GrowElementBase.createDeformedPoint_df ();
   }

   /* --- Accessors --- */

   public PolarityElementAux getPolAux () {
      return this.mPolAux;
   }
}

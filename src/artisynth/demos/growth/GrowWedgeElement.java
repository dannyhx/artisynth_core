package artisynth.demos.growth;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.WedgeElement;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector3d;

public class GrowWedgeElement extends WedgeElement implements GrowElementBase {

   /** Primary direction of growth. */
   public Vector3d mPolDir = new Vector3d(0,1,0);
   
   /** 3 directions of growth. */
   protected Matrix3d mFrame = new Matrix3d();
   
   /** Growth tensor. Contains magnitude of growth for each node. 
    * (numNodes x 3dof). */
   protected MatrixNd mElementGrowthTensor;
   
   /** Rotated growth tensor. (numNodes x 6). Each row contains a 
    * symmetrical 3x3 plastic strain matrix. */
   protected MatrixNd mRotatedElementGrowthStrains;
   
   /** Transformed variation of the rotated growth tensor where each column 
    *  contains a symmetrical 3x3 plastic strain matrix for a given integration 
    *  point of the element. (6 x numIntegPts). */
   protected MatrixNd mStrainAtIntegPts;
   
   /** Matrix representation of the strain at each edge. */
   protected Matrix3d mBendStrain;
   
   
   
   
   /* --- Constructor --- */

   public GrowWedgeElement (GrowNode3d p0, GrowNode3d p1,
   GrowNode3d p2, GrowNode3d p3, GrowNode3d p4, GrowNode3d p5) {
      super(p0, p1, p2, p3, p4, p5);
      
      // Force GrowNode3d[]
      setNodes(p0, p1, p2, p3, p4, p5);
      
      Matrix3d Fg = new Matrix3d();
      Fg.setIdentity ();
      setPlasticDeformation( Fg );
   }
   
   
   /* --- Plastic Embedding --- */

   public void useResidualPlasticStrain() {
      GrowElementBase.useResidualPlasticStrain_df (this);
   }
   
   
   /* --- Nodes --- */
   
   /**
    * We cannot override ShellTriElement() to initialize myNodes as
    * GrowNode[], so use this instead.
    * */
   public void setNodes (FemNode3d p0, FemNode3d p1, FemNode3d p2, FemNode3d p3,
   FemNode3d p4, FemNode3d p5) {
      myNodes = new GrowNode3d[] {
          (GrowNode3d)p0, (GrowNode3d)p1, (GrowNode3d)p2,
          (GrowNode3d)p3, (GrowNode3d)p4, (GrowNode3d)p5
      };
      super.setNodes (myNodes);
   }
   
   public GrowNode3d[] getNodes() {
      return (GrowNode3d[]) super.getNodes ();
   }
   
   
   /* --- Stiffness Warper 3d --- */
   
   public GrowStiffnessWarper3d createStiffnessWarper () {
      return GrowElementBase.createStiffnessWarper_df (this);
   }
   
   
   
   /* --- Integration Data --- */
   
   /** Overridden to ensure warping point is a GrowIntegrationData3d rather than
    *  the plain IntegrationData3d. */
   public GrowIntegrationData3d getWarpingData() {
      return GrowElementBase.getWarpingData_df (this);
   }
   
   public GrowIntegrationData3d[] getIntegrationData() {
      return (GrowIntegrationData3d[])super.getIntegrationData ();
   }
   
   public GrowIntegrationData3d[] doGetIntegrationData() {
      return GrowElementBase.doGetIntegrationData_df (this);
   }
   
   
   
   /* --- Volume --- */

   /** Compute the volume of the element. Necessary to update its mass. 
    *
    * When computing the rest volume, the rest volume is scaled by the 
    * plastic strain. This is ensure that the mass still increases, even if the 
    * plastic strain cannot be "relaxed".
    */
   public double _computeVolume (boolean isRest) {
      return GrowElementBase._computeVolume_df (this, isRest);
   }
   
   
   
   /* --- Misc Methods --- */
   
   public GrowDeformedPoint createDeformedPoint() {
      return GrowElementBase.createDeformedPoint_df ();
   }
   
   
   
   /* --- Accessors --- */
   
   public Vector3d getPolDir () {
      return mPolDir;
   }

   public void setPolDir (Vector3d mPolDir) {
      this.mPolDir = mPolDir;
   }

   public Matrix3d getFrame () {
      return mFrame;
   }

   public void setFrame (Matrix3d mFrame) {
      this.mFrame = mFrame;
   }

   public MatrixNd getElementGrowthTensor () {
      return mElementGrowthTensor;
   }

   public void setElementGrowthTensor (MatrixNd mElementGrowthTensor) {
      this.mElementGrowthTensor = mElementGrowthTensor;
   }

   public MatrixNd getRotatedElementGrowthStrains () {
      return mRotatedElementGrowthStrains;
   }

   public void setRotatedElementGrowthStrains (
      MatrixNd mRotatedElementGrowthStrains) {
      this.mRotatedElementGrowthStrains = mRotatedElementGrowthStrains;
   }

   public MatrixNd getStrainAtIntegPts () {
      return mStrainAtIntegPts;
   }

   public void setStrainAtIntegPts (MatrixNd mStrainAtIntegPts) {
      this.mStrainAtIntegPts = mStrainAtIntegPts;
   }

   public Matrix3d getBendStrain () {
      return mBendStrain;
   }

   public void setBendStrain (Matrix3d mBendStrain) {
      this.mBendStrain = mBendStrain;
   }
}

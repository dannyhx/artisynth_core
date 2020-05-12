package artisynth.demos.growth;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.demos.growth.util.MathUtil;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/** 
 * Solid-shell (or simply referred as shell) modified to accommodate growth. 
 * 
 * Attributes are with respect to Morphogen2GrowthTensor.java.
 */
public class GrowTriElement extends ShellTriElement {
   
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
 
   
   
   
   /* --- Constructor --- */
   
   public GrowTriElement (GrowNode3d p0, GrowNode3d p1,
   GrowNode3d p2, double thickness) {
      super(p0, p1, p2, thickness);
      
      Matrix3d Fg = new Matrix3d();
      Fg.setIdentity ();
      setPlasticDeformation( Fg );
   }
   
   /**
    * We cannot override ShellTriElement() to initialize myNodes as
    * GrowNode[], so use this instead.
    * */
   public void setNodes (FemNode3d p0, FemNode3d p1, FemNode3d p2) {
      myNodes = new GrowNode3d[myNodeCoords.length/3];
      super.setNodes (p0, p1, p2);
   }
   

   
   
   
   /* --- Widgets --- */
   
   public static PropertyList myProps =
   new PropertyList (GrowTriElement.class, ShellTriElement.class);

   static {
      // These properties are for debugging purposing only in the element UI.
      myProps.add ("inelasticFMaxNorm", "", 1);
      myProps.add ("volume", "", 1);
      myProps.add ("area", "", 1);
      myProps.add ("centroid", "", new Point3d());
      myProps.add ("node0", "", new Point3d());
      myProps.add ("node1", "", new Point3d());
      myProps.add ("node2", "", new Point3d());
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public double getVolume() {
      this.computeRestVolumes ();
      return this.myRestVolume;
   }

   public double getArea() {
      return MathUtil.area (
        this.myNodes[0].getPosition (),
        this.myNodes[1].getPosition (),
        this.myNodes[2].getPosition ()
      );
   }
   
   public double getInelasticFMaxNorm() { 
      double max = 0;
      for (int k = 0; k < numIntegrationPoints (); k++) {
         GrowIntegrationData3d gid = (GrowIntegrationData3d)getIntegrationData ()[k];
         max = Math.max (max, gid.getFp ().maxNorm ());
      }
      return max;
   }
   
   public Point3d getCentroid() {
      return MathUtil.centroid (
         this.myNodes[0].getPosition (),
         this.myNodes[1].getPosition (),
         this.myNodes[2].getPosition ()
      );
   }
   
   public Point3d getNode0() {
      return this.myNodes[0].getPosition ();
   }
   
   public Point3d getNode1() {
      return this.myNodes[1].getPosition ();
   }
   
   public Point3d getNode2() {
      return this.myNodes[2].getPosition ();
   }
   
   public void setInelasticFMaxNorm(double val) {}
   public void setVolume(double val) {};
   public void setArea(double val) {};
   public void setCentroid(Point3d val) {};
   public void setNode0(Point3d val) {};
   public void setNode1(Point3d val) {};
   public void setNode2(Point3d val) {};
   
   
   /* --- Plastic Embedding --- */
   
   /**
    * Compute the residual plastic strain.
    * 
    * See section 3.7 in the thesis.
    * 
    * Assuming linear material:   F_expected = F_occurred + F_residual
    * where
    *   F_expected = The amount of growth to occur.
    *   F_occurred = The amount of growth that actually occurred.
    *   F_residual = The amount of remaining growth to occur.
    *   
    * @precond
    * Reference configuration is set to the old reference configuration.
    * World configuration is set to the new reference configuration.
    */
   protected void useResidualPlasticStrain() {
      SVDecomposition3d SVD = new SVDecomposition3d();
      
      Matrix3d F_occurred = new Matrix3d();
      Matrix3d F_expected = new Matrix3d();
      
      // Symmetrical factors.
      Matrix3d P_occurred = new Matrix3d();
      Matrix3d P_expected = new Matrix3d();
      
      // Rotational factors.
      RotationMatrix3d R_occurred = new RotationMatrix3d();
      RotationMatrix3d R_expected = new RotationMatrix3d();
      
      // Debugging only.
      Matrix3d S_occurred = new Matrix3d();
      Matrix3d S_expected = new Matrix3d();
      
      IntegrationPoint3d[] integPts = null;
      IntegrationData3d[] integDts = null;
      
      integPts = getIntegrationPoints ();
      integDts = getIntegrationData ();


      for (int k = 0; k < integPts.length; k++) {
         GrowIntegrationData3d gid = (GrowIntegrationData3d)integDts[k];
         
         // Compute amount of deformation actually occurred, between
         // t0 and t1.
         Matrix3d J = new Matrix3d();
         integPts[k].computeJacobian (J, getNodes ());
         integDts[k].computeInverseRestJacobian (integPts[k], getNodes ());   
         F_occurred.mul (J, integDts[k].getInvJ0 ());
         
         // Extract symmetrical factor of deformation actually occurred
         SVD.polarDecomposition(R_occurred, P_occurred, F_occurred);
         
         // Extract symmetrical factor of expected deformation
         F_expected = gid.getFp ();
         SVD.polarDecomposition (R_expected, P_expected, F_expected);

         // Remaining deformation to expect. 
         // As F_occurred goes up, F_residual decreases
         
         Matrix3d P_residual = new Matrix3d();
         
         // Linear
         P_residual.sub (P_expected, P_occurred);
         P_residual.add (Matrix3d.IDENTITY);
         
         // Non-linear   // TODO
//         P_residual.mulInverseRight (P_expected, P_occurred);
//         P_residual.mul (R_occurred, P_residual);
         
         // Symmetric testing.
         // Doesn't make much of a difference. Numerically, numbers
         // are very similar.
         S_occurred.setSymmetric (F_occurred);
         S_expected.setSymmetric (F_expected);
         P_residual.sub (S_expected, S_occurred);
         P_residual.add (Matrix3d.IDENTITY);
         
         // Debug
         boolean isExpectedStrainExceed = (P_expected.maxNorm() > 1.95);
         boolean isResidualStrainExceed = (P_residual.maxNorm() > 1.95);
         boolean isOccurredStrainExceed = (P_occurred.maxNorm() > 1.95);
         
         if (isExpectedStrainExceed ||
             isResidualStrainExceed || 
             isOccurredStrainExceed)
         {
            System.out.printf ("Warning: Linear limit exceed: Exp(%b), "+
              "Res(%b), Occ(%b).\n", isExpectedStrainExceed,
              isResidualStrainExceed, isOccurredStrainExceed);
         }
         
         gid.setFp ( P_residual );
      }
   }
   
   
   
   /* --- Nodes --- */
   
   public GrowNode3d[] getNodes() {
      return (GrowNode3d[])myNodes;
   }
   
   
   
   /* --- Stiffness Warper 3d --- */
   
   protected GrowStiffnessWarper3d createStiffnessWarper () {
      return new GrowStiffnessWarper3d (this);
   }
   
   
   
   /* --- Integration Data --- */
   
   /** Overridden to ensure warping point is a GrowIntegrationData3d rather than
    *  the plain IntegrationData3d. */
   public GrowIntegrationData3d getWarpingData() {
      IntegrationData3d wdata = myWarpingData;
      if (wdata == null) {
         int numPnts = getIntegrationPoints().length;
         if (numPnts == 1) {
            // then integration and warping points/data are the same
            wdata = getIntegrationData()[0];
         }
         else {
            wdata = new GrowIntegrationData3d();
            wdata.computeInverseRestJacobian (getWarpingPoint(), myNodes);
         }
         myWarpingData = wdata;
      }
      return (GrowIntegrationData3d)wdata;
   }
   
   public GrowIntegrationData3d[] getIntegrationData() {
      return (GrowIntegrationData3d[])super.getIntegrationData ();
   }
   
   protected GrowIntegrationData3d[] doGetIntegrationData() {
      IntegrationData3d[] idata = myIntegrationData;
      if (idata == null) {
         int numPnts = numIntegrationPoints();
         idata = new GrowIntegrationData3d[numPnts];
         for (int i=0; i<numPnts; i++) {
            idata[i] = new GrowIntegrationData3d();
         }
         myIntegrationData = idata;
      }
      return (GrowIntegrationData3d[])idata;
   }
   
   
   
   /* --- Volume --- */

   /** Compute the volume of the element. Necessary to update its mass. 
    *
    * When computing the rest volume, the rest volume is scaled by the 
    * plastic strain. This is ensure that the mass still increases, even if the 
    * plastic strain cannot be "relaxed".
    */
   public double _computeVolume (boolean isRest) {
      double vol = 0;

      // For each integration point...
      IntegrationPoint3d[] ipnts = getIntegrationPoints ();
      IntegrationData3d[] idata = getIntegrationData ();
      for (int i = 0; i < ipnts.length; i++) {
         GrowIntegrationData3d gid = (GrowIntegrationData3d)idata[i];
         
         double detJ;
         if (isRest) {
            detJ = idata[i].getDetJ0();
         }
         else {
            detJ = ipnts[i].computeJacobianDeterminant(getNodes());
         }
         
         // Scale by plasticity
         detJ *= gid.getFp ().determinant ();

         vol += detJ*ipnts[i].getWeight ();
      }

      return vol;
   }
   
   
   
   /* --- Misc Methods --- */
   
   public GrowDeformedPoint createDeformedPoint() {
      return new GrowDeformedPoint();
   }
}

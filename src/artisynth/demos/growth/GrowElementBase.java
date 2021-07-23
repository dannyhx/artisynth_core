package artisynth.demos.growth;

import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector3d;

public interface GrowElementBase {
   
   /* --- Growth --- */
   
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
   public void useResidualPlasticStrain();
   public static void useResidualPlasticStrain_df(FemElement3dBase _this) {
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
      
      integPts = _this.getIntegrationPoints ();
      integDts = _this.getIntegrationData ();


      for (int k = 0; k < integPts.length; k++) {
         GrowIntegrationData3d gid = (GrowIntegrationData3d)integDts[k];
         
         // Compute amount of deformation actually occurred, between
         // t0 and t1.
         Matrix3d J = new Matrix3d();
         integPts[k].computeJacobian (J, _this.getNodes ());
         integDts[k].computeInverseRestJacobian (integPts[k], _this.getNodes ());   
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
   
   public GrowNode3d[] getNodes();
   
   /* --- Stiffness Warper 3d --- */
   
   public GrowStiffnessWarper3d createStiffnessWarper ();
   public static GrowStiffnessWarper3d createStiffnessWarper_df (FemElement3dBase _this) {
      return new GrowStiffnessWarper3d (_this);
   }
   
   /* --- Integration Data --- */
   
   /** Overridden to ensure warping point is a GrowIntegrationData3d rather than
    *  the plain IntegrationData3d. */
   public GrowIntegrationData3d getWarpingData();
   public static GrowIntegrationData3d getWarpingData_df(FemElement3dBase _this) {
      IntegrationData3d wdata = _this.myWarpingData;
      if (wdata == null) {
         int numPnts = _this.getIntegrationPoints().length;
         if (numPnts == 1) {
            // then integration and warping points/data are the same
            wdata = _this.getIntegrationData()[0];
         }
         else {
            wdata = new GrowIntegrationData3d();
            wdata.computeInverseRestJacobian (_this.getWarpingPoint(), _this.getNodes ());
         }
         _this.myWarpingData = wdata;
      }
      return (GrowIntegrationData3d)wdata;
   }
   
   public GrowIntegrationData3d[] getIntegrationData();
   
   public GrowIntegrationData3d[] doGetIntegrationData();
   public static GrowIntegrationData3d[] doGetIntegrationData_df(FemElement3dBase _this) {
      IntegrationData3d[] idata = _this.myIntegrationData;
      if (idata == null) {
         int numPnts = _this.numIntegrationPoints();
         idata = new GrowIntegrationData3d[numPnts];
         for (int i=0; i<numPnts; i++) {
            idata[i] = new GrowIntegrationData3d();
         }
         _this.myIntegrationData = idata;
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
   public double _computeVolume (boolean isRest);
   public static double _computeVolume_df (FemElement3dBase _this, boolean isRest) {
      double vol = 0;

      // For each integration point...
      IntegrationPoint3d[] ipnts = _this.getIntegrationPoints ();
      IntegrationData3d[] idata = _this.getIntegrationData ();
      for (int i = 0; i < ipnts.length; i++) {
         GrowIntegrationData3d gid = (GrowIntegrationData3d)idata[i];
         
         double detJ;
         if (isRest) {
            detJ = idata[i].getDetJ0();
         }
         else {
            detJ = ipnts[i].computeJacobianDeterminant(_this.getNodes());
         }
         
         // Scale by plasticity
         detJ *= gid.getFp ().determinant ();

         vol += detJ*ipnts[i].getWeight ();
      }

      return vol;
   }
   
   /* --- Misc Methods --- */
   
   public GrowDeformedPoint createDeformedPoint();
   public static GrowDeformedPoint createDeformedPoint_df() {
      return new GrowDeformedPoint();
   }
   
   
   /* --- Accessors --- */
   
   public Vector3d getPolDir ();

   public void setPolDir (Vector3d mPolDir);

   public Matrix3d getFrame ();

   public void setFrame (Matrix3d mFrame);

   public MatrixNd getElementGrowthTensor ();
   
   public void setElementGrowthTensor (MatrixNd mElementGrowthTensor);

   public MatrixNd getRotatedElementGrowthStrains ();

   public void setRotatedElementGrowthStrains (
      MatrixNd mRotatedElementGrowthStrains);

   public MatrixNd getStrainAtIntegPts ();

   public void setStrainAtIntegPts (MatrixNd mStrainAtIntegPts);

   public Matrix3d getBendStrain ();

   public void setBendStrain (Matrix3d mBendStrain);
}

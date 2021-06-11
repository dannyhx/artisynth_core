package artisynth.demos.growth;

import artisynth.core.femmodels.FemDeformedPoint;
import artisynth.core.femmodels.FemElement.ElementClass;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.demos.growth.util.HingeUtil;
import maspack.matrix.Matrix3d;
import maspack.matrix.RotationMatrix3d;

/** 
 * Extension of FemDeformedPoint to account for growth. 
 * 
 * FemDeformPoint is basically a utility that operates on its given 
 * integration point, such as computing the deformation gradient.
 * */
public class GrowDeformedPoint extends FemDeformedPoint {
   
   public void setFromIntegrationPoint (
   IntegrationPoint3d ipnt, IntegrationData3d idat,
   RotationMatrix3d R, FemElement3dBase elem, int idx) {
      
      GrowTriElement gEle = (GrowTriElement)elem;
      GrowIntegrationData3d gid = (GrowIntegrationData3d)idat;
      
      // Sanity
      idat.computeInverseRestJacobian (ipnt, gEle.getNodes ());
      
      super.setFromIntegrationPoint (ipnt, idat, R, elem, idx);
      
      myF.mulInverse (gid.getFp ());  
      myDetF = myF.determinant ();
      
      // DAN21
      if (elem.getElementClass () == ElementClass.MEMBRANE) {
         Matrix3d E = HingeUtil.computeStrain ((ShellTriElement)elem);
         E.setSymmetric (E);
         myF.setIdentity ();
         myF.add(E); 
         myDetF = myF.determinant ();
      }
   }

   public void setFromRestPoint (
   IntegrationPoint3d ipnt, IntegrationData3d idat,
   RotationMatrix3d R, FemElement3dBase elem, int idx) {
      
      GrowIntegrationData3d gid = (GrowIntegrationData3d)idat;

      super.setFromRestPoint (ipnt, idat, R, elem, idx);

      // E = plastic strain = Fp - I
      Matrix3d E = new Matrix3d();
      E.setSymmetric ( gid.getFp () );
      E.m00 -= 1;
      E.m11 -= 1;
      E.m22 -= 1;
      
      // F = Fe Fp = I Fp
      myF.sub (Matrix3d.IDENTITY, E);
      myDetF = myF.determinant ();
   }
}

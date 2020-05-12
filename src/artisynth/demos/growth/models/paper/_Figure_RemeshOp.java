package artisynth.demos.growth.models.paper;

import java.awt.Color;

import artisynth.demos.growth.models.base.GrowDemo;
import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.HalfEdge;
import maspack.matrix.Point3d;
import maspack.render.Renderer.Shading;

// artisynth.models.plants.growth.paperdemos._RemeshFigure
public class _Figure_RemeshOp extends GrowDemo {

   protected void build_pre() {
      super.build_pre ();
      
      mEnableDiffusion = false;
      mEnableGrowth = false;
      mEnableRemesh = false;
      mEnablePlasticEmbedding = false;
      mEnableCollisionHandling = false;
      mMaintainResidualStrain = false;
      
      mMeshXDiv = 13;
      mMeshYDiv = 13;
   }
   
   protected void build_modelSkeleton() {
      mMesh[0] = MeshUtil.createHexagonalTriRectangle (1, 1, 14, 14, false);
      
//      RigidTransform3d X = new RigidTransform3d();
//      X.mulRotZ (Math.toRadians (90));
//      mMesh[0].transform (X);
   }
   
   protected void build_femRendering() {
      super.build_femRendering ();
      mFemModel[0].getSurfaceMesh ().getRenderProps().setShading (Shading.FLAT);
      
      mFemModel[0].getRenderProps ().setLineColor (Color.BLACK);
      mFemModel[0].getRenderProps ().setLineWidth (2);
   }

   public void advanceCustom(double t0, double t1, int flags) {  
      getMainViewer().setEye (new Point3d(0.5, 0.5, 0.70));
      getMainViewer().setCenter (new Point3d(0.5, 0.5, 0));
      
      super.advanceCustom (t0, t1, flags);
   }
   
   protected void customLogic() {
      HalfEdge edge = MeshUtil.getEdge (108, 109, mMesh[0]);
      mRemesher.mSizingField.computeVertexSizingFields ();
//      mRemesher.bisectEdge (edge);
      mRemesher.flip (edge);
//      mRemesher.collapseEdge (edge, mMesh[0].getVertex (109), false);
   }
}

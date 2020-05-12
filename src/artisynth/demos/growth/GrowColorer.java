package artisynth.demos.growth;

import java.awt.Color;

import artisynth.core.femmodels.FemNode3d;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.ColorInterpolation;
import maspack.render.Renderer.ColorMixing;
import maspack.render.Renderer.FaceStyle;

/** 
 *  Handles coloring the surface of the growth model.
 *  
 *  The color can correspond to the amount of plastic strain or morphogen
 *  concentration present.
 */
public class GrowColorer {
   /** Lowest value in the plastic strain color bar. */
   protected final double mMinPlasticStrainColorBarRange = 1;
   
   /** Highest value in the plastic strain color bar.*/
   public double mMaxPlasticStrainColorBarRange = 1.10;
   
   /** Lowest value in the morphogen color bar. */
   protected final double mMinMorphogenColorBarRange = 0;
   
   /** Highest value in the morphogen color bar.*/
   protected final double mMaxMorphogenColorBarRange = 1;
   
   /** RGB color for areas with lowest morphogen value. */
   public Vector3d zeroMorphogenRGB = new Vector3d(0,1,0);
   
   /** RGB color for areas with highest morphogen value. */
   public Vector3d maxMorphogenRGB = new Vector3d(1,0,1);
   
   protected GrowModel3d mFemModel;
   protected RootModel mRootModel;
   protected PolygonalMesh mSurfaceMesh;
   
   protected ColorBar mColorBar;

   /**
    * Handles the visualization of the amount and localization of plastic strain
    * or morphogen concentration on a FEM model.
    * 
    * @param root
    * Root model that contains the FE model. It will hold the color map chart.
    */
   public GrowColorer (
   RootModel root, GrowModel3d femModel, boolean showColorBar) {
      setTarget(femModel);

      mColorBar = new ColorBar ();
      mColorBar.setNumberFormat ("%.5f"); // 5 decimal places.
      mColorBar.populateLabels (
         mMinPlasticStrainColorBarRange, mMaxPlasticStrainColorBarRange, 10 /*ticks*/);
      mColorBar.setColorMap (mFemModel.getColorMap ());
      mColorBar.setLocation (-80, 0.05, 20, 0.9);
      if (showColorBar)
         root.addRenderable (mColorBar);
      mColorBar.setColorMap (mFemModel.getColorMap ());
   }
   
   public void setTarget(GrowModel3d femModel) {
      mFemModel = femModel;
   }
   
   /**
    * Update the colors, corresponding to the amount of plastic strain.
    */
   public void computePlasticStrainColors () {
      // When coloring vtx-by-vtx, cannot give different colors for
      // front and back faces.
      RenderProps.setFaceStyle (mFemModel.getSurfaceMeshComp (), FaceStyle.FRONT);
      mSurfaceMesh = mFemModel.getSurfaceMesh ();  // Latest surface mesh
      mSurfaceMesh.setVertexColoringEnabled ();
      mColorBar.updateLabels (mMinPlasticStrainColorBarRange, mMaxPlasticStrainColorBarRange);
      
      Matrix3d[] nodalFps = mFemModel.getNodalPlasticDeformationGradient ();
      double[] rgb = new double[3];  
      for (int n = 0; n < mFemModel.numNodes (); n++) {
         FemNode3d node = mFemModel.getNode (n);
         
         // Convert the plastic deformation gradient into a scalar.
         double scalar = fpToScalar (nodalFps[n]);
         
         mColorBar.getColor (scalar, rgb);
         mSurfaceMesh.setColor (mFemModel.getSurfaceVertex (node).getIndex (), 
            rgb[0], rgb[1], rgb[2], 1);
      }  
   }
   
   /**
    * Update the colors, corresponding to the amount of morphogen.
    */
   public void computeMorphogenColors () {
      // When coloring vtx-by-vtx, cannot give different colors for 
      // front and back faces.
      mSurfaceMesh = mFemModel.getSurfaceMesh ();  // Latest surface mesh

      mSurfaceMesh.setVertexColoringEnabled ();
      mSurfaceMesh.setColorInterpolation (ColorInterpolation.RGB);
      mSurfaceMesh.setVertexColorMixing (ColorMixing.REPLACE);
      mSurfaceMesh.setColorsFixed (false);
      
      mColorBar.updateLabels (
         mMinMorphogenColorBarRange, mMaxMorphogenColorBarRange);
      
      double maxMorph = 0;
      for (FemNode3d node : mFemModel.getNodes ()) {
         maxMorph = Math.max ( ((GrowNode3d)node).getChem3 (), maxMorph);
      }
      
      for (FemNode3d node : mFemModel.getNodes ()) {
         GrowNode3d gNode = (GrowNode3d)node;
         double nodeMorph = gNode.getChem3 ();
         
         Vector3d nodeMorphogenRGB = interpolateRGB (
            zeroMorphogenRGB, maxMorphogenRGB, nodeMorph/maxMorph);
         
         mSurfaceMesh.setColor (mFemModel.getSurfaceVertex (node).getIndex (),
            nodeMorphogenRGB.x, nodeMorphogenRGB.y, nodeMorphogenRGB.z, 1);
      }
      
      mFemModel.getRenderProps ().setLineColor (Color.WHITE);
      mSurfaceMesh.getRenderProps ().setFaceStyle (FaceStyle.FRONT_AND_BACK);
      mSurfaceMesh.getRenderProps ().setBackColor (Color.CYAN);
      
   }
   
   /**
    * Convert a plastic deformation gradient into a scalar value that's within
    * the color bar range.
    */
   protected double fpToScalar(Matrix3d Fp) {
      return Fp.determinant ();
   }
   
   /**
    * Convert a morphogen concentration into a scalar value that's within the 
    * color bar range.
    */
   protected double morphogenToScalar(double morphogen) {
      return Math.min(1.5, morphogen);
   }
   
   /**
    * Turn off any surface coloring.
    */
   public void toggleOff() {
      RenderProps.setFaceStyle (
         mFemModel.getSurfaceMeshComp (), FaceStyle.FRONT_AND_BACK);
      mSurfaceMesh = mFemModel.getSurfaceMesh ();
      mSurfaceMesh.clearColors ();
   }
   
   /** Get an interpolated RGB value between two given RGB values. */
   public Vector3d interpolateRGB(Vector3d A, Vector3d B, double pctB) {
      Vector3d C = new Vector3d();
      
      C.scaledAdd (1-pctB, A);
      C.scaledAdd (pctB   ,B);
      
      return C;
   }
}

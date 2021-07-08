package artisynth.demos.growth.models.base;

import java.awt.Color;

import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import artisynth.demos.growth.thinshell.EdgeDataMap;
import artisynth.demos.growth.thinshell.ThinShellAux;
import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;
import maspack.widgets.DoubleFieldSlider;
import maspack.widgets.LabeledComponentBase;

/**
 * Square patch of triangular solid-shell elements, subjected to gravity. 
 * Some nodes will be held in-place to demonstrate shell bending forces.
 * 
 * This is a base class for the growth demo.
 */
public class ShellPatch extends RootModel {
   
   /* --- Bodies --- */
   
   /** Number of models to simulate. */
   protected int M = 1;
   
   /** Mesh for each model. Allows access to mesh traversal tools. */
   protected PolygonalMesh[] mMesh = null;
   
   /** Models to simulate. */
   protected FemModel3d[] mFemModel = null;
   
   /** Container for the simulated models. */
   protected MechModel mMechModel = null;

   /* --- Model Shape --- */
   
   /** Width of square patch of shell elements. */
   public double mMeshX = 10;    
   
   /** Length of square patch of shell elements. */
   public double mMeshY = 10;
   
   /** Number of shell elements per row. */
   public int mMeshXDiv = 10;
   
   /** Number of shell elements per column. */
   public int mMeshYDiv = 10;
   
   /* --- FEM physical properties --- */
   
   /** Use membrane? */
   protected boolean m_isMembrane = false;
   
   /** Overall density of shell patch. */
   protected double m_density = 100;
   
   /** Generic particle velocity damping. */
   protected static double m_particleDamping = 1.0;
   
   /** Generic stiffness damping. 0 for water-like, 100 for aluminium-like. */
   protected static double m_stiffnessDamping = 0.05;              
   
   /** Affects bending strength. */
   protected static double m_shellThickness = 0.1;
   
   /** Material stiffness. */
   protected double m_youngsModulus = 500000;
   
   /** */
   protected double m_poissonsRatio = 0.33;
   
   /* --- Rendering --- */
   
   public class RenderConfig {
      public Color mFrontMeshColor = Color.GREEN;
      public Color mRearMeshColor = Color.CYAN;
      public Color mNodeColor = Color.WHITE;
      public double mNodeRadius = 0.02;
      public double mDirectorLen = 0;
      public SurfaceRender mSurfaceRender = SurfaceRender.Shaded;
      public Shading mShading = Shading.FLAT;
      public Color mMeshEdgeColor = Color.BLUE;
      public Color mBackgroundColor = Color.WHITE;
      public int mLineWidth = 1;
      public boolean mDrawEdges = false;
   }
   
   protected static RenderConfig mRendCfg = null;
   
   /*--- Environment --- */

   protected Vector3d mGravity = new Vector3d(0, 0, -9.81);
   
   /*--- UI --- */
   
   protected ControlPanel mPanel;
   
   /* --- Misc --- */
   
   // Number of times initialized 
   protected int mNumInitialized = 0;
  
   
   
   public void initialize (double t) {
      // On startup: build(), init()
      // On restart: init()
      mNumInitialized++;
      if (mNumInitialized > 1) {
         clear ();
         build (null);
      }
      System.out.println ("ShellPatch.initialize()");
      super.initialize (t);
   }

   public void build (String[] args) {
      System.out.println ("ShellPatch.build()");
      
      build_pre();
      build_modelSkeleton();
      build_modelStructure();
      build_modelProperties();
      build_utils();
      build_renderConfig();
      build_femRendering();
      build_assembleUI();
      build_addUI();
      build_post();
   }    
   
   /**
    * Initialize and override your scalar/tensor attributes here.
    */
   protected void build_pre() {
      
   }
   
   /**
    * Create a mesh, which will be used as a template for building 
    * the FEM model.
    */
   protected void build_modelSkeleton() {
      mMesh = new PolygonalMesh[M];
      for (int m=0; m<M; m++) { 
         mMesh[m] = MeshFactory.createPlane(mMeshX, mMeshY, mMeshXDiv, mMeshYDiv);
      }
   }
   
   /**
    * Assemble the FEM model.
    */
   protected void build_modelStructure() {
      mMechModel = new MechModel ("mech");
      
      mFemModel = new FemModel3d[M];
      for (int m=0; m<M; m++) { 
         mFemModel[m] = createFemModel();

         // Create a node for each mesh vertex
         for (int v = 0; v < mMesh[m].numVertices (); v++) {
            Vertex3d vtx = mMesh[m].getVertex (v);
            FemNode3d node = createNode(vtx.getPosition ());
            node.setName ("MyNode_#" + v);
            mFemModel[m].addNode (node);
         }
         
         // Create an element for each mesh face
         for (int f = 0; f < mMesh[m].numFaces (); f++) {
            Face face = mMesh[m].getFace (f);
            int[] vtxIdxs = face.getVertexIndices ();
   
            FemNode3d n0 = mFemModel[m].getNode( vtxIdxs[0] );
            FemNode3d n1 = mFemModel[m].getNode( vtxIdxs[1] );
            FemNode3d n2 = mFemModel[m].getNode( vtxIdxs[2] );
   
            ShellTriElement ele = createElement(n0, n1, n2, m_shellThickness);
            ele.setName ("MyEle_#" + f);
            mFemModel[m].addShellElement(ele);
         }
         
         if (this.m_isMembrane) {
            mFemModel[m].myEdgeDataMap = EdgeDataMap.createFromMesh (mMesh[m]);
            mFemModel[m].myThinShellAux = new ThinShellAux(mFemModel[m], mMesh[m]);
            mFemModel[m].myThinShellAux.setAltMaterial (
               m_youngsModulus, m_poissonsRatio, m_shellThickness);
         }

         
         mMechModel.addModel (mFemModel[m]);
      }
      
      addModel (mMechModel);
   }
   
   /**
    * Configure the FEM model.
    */
   protected void build_modelProperties() {
      for (int m=0; m<M; m++) { 
         mFemModel[m].setMaterial (
            new LinearMaterial(m_youngsModulus, m_poissonsRatio));
         mFemModel[m].setStiffnessDamping (m_stiffnessDamping);
         mFemModel[m].setGravity (mGravity);
         mFemModel[m].setDensity (m_density);
         mFemModel[m].setParticleDamping (m_particleDamping);
   
         // Hold some nodes in-place
         for (int n = 0; n < mFemModel[m].numNodes (); n++) {
            FemNode3d node = mFemModel[m].getNode (n);
            node.setRenderProps( node.createRenderProps() );
            if (shouldBeFrozen(n)) {
               node.setDynamic (false);
            }
         }
      }
   }
   
   /**
    * Initialize objects, such as those that require the FEM model
    * to be already assembled at this point.
    */
   protected void build_utils() {
      
   }
   
   protected void build_renderConfig() {
      if (mRendCfg == null) {
         mRendCfg = new RenderConfig();
      }
   }
   
   protected void build_femRendering() {
      for (int m=0; m<M; m++) { 
         mFemModel[m].setSurfaceRendering (mRendCfg.mSurfaceRender);
         RenderProps.setFaceColor (mFemModel[m], mRendCfg.mFrontMeshColor);
         RenderProps.setBackColor (mFemModel[m], mRendCfg.mRearMeshColor);
         RenderProps.setPointColor (mFemModel[m], mRendCfg.mNodeColor);
         RenderProps.setShininess (
            mFemModel[m], mFemModel[m].getRenderProps().getShininess() * 10);
         RenderProps.setVisible (mFemModel[m], true);
         RenderProps.setFaceStyle (mFemModel[m], Renderer.FaceStyle.FRONT_AND_BACK);
         RenderProps.setPointStyle (mFemModel[m].getNodes(), 
                                    Renderer.PointStyle.SPHERE);
         RenderProps.setPointRadius (mFemModel[m].getNodes(), mRendCfg.mNodeRadius);
         mFemModel[m].setDirectorRenderLen (mRendCfg.mDirectorLen);
         mFemModel[m].getRenderProps ().setLineWidth (mRendCfg.mLineWidth);
         getMainViewer ().setBackgroundColor (mRendCfg.mBackgroundColor);
         RenderProps.setShading (mFemModel[m], mRendCfg.mShading);
         RenderProps.setDrawEdges (mFemModel[m], mRendCfg.mDrawEdges);
      }
   }
   
   /**
    * Build the user panel.
    */
   protected void build_assembleUI() {
      // Add control panel

      mPanel = new ControlPanel("shellPatchControlPanel");
      mPanel.addWidget (this, "shellThickness", m_shellThickness, 10);
      mPanel.addWidget (mFemModel[0], "particleDamping");
      mPanel.addWidget (mFemModel[0], "stiffnessDamping", 0, 2000);
      mPanel.addWidget (mFemModel[0], "density", 10, 1000);
      mPanel.addWidget (mFemModel[0], "gravity");
      mPanel.addWidget (mFemModel[0], "directorRenderLen");
      mPanel.addWidget (mFemModel[0], "material");
      LabeledComponentBase comp = mPanel.addWidget (mFemModel[0], "material.YoungsModulus");
      ((DoubleFieldSlider)comp).setSliderRange (1, 100000);
   }
   
   /**
    * Add panel to control panel.
    */
   protected void build_addUI() {
      addControlPanel(mPanel);
   }
   
   /**
    * Final touches to build.
    */
   protected void build_post() {

   }
   
   /* --- Helper methods, which can overridden, for building --- */
   
   protected FemModel3d createFemModel() {
      return new FemModel3d();
   }
   
   protected FemNode3d createNode(Point3d pt) {
      return new FemNode3d(pt);
   }
   
   protected ShellTriElement createElement(FemNode3d n0, FemNode3d n1,
   FemNode3d n2, double thickness) {
      ShellTriElement ele = new ShellTriElement(n0, n1, n2, thickness, this.m_isMembrane);
      return ele;
   }
   
   /**
    * Should this node be non-dynamic?
    */
   public boolean shouldBeFrozen(int idx) {
      return (idx <= mMeshXDiv);        // bottom edge nodes
   }

   protected int getIdx(FemModel3d femModel) {
      for (int m=0; m<M; m++) {
         if (mFemModel[m] == femModel) 
            return m;
      }
      return -1;
   }
   
   /* --- Properties --- */
   
   public static PropertyList myProps =
      new PropertyList (ShellPatch.class, RootModel.class);
   
   static {
      myProps.add("shellThickness", "Thickness of each shell element", 
                  m_shellThickness);
   }
   
   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public double getShellThickness() {
      return mFemModel[0].getShellElement (0).getDefaultThickness ();
   }
   
   public void setShellThickness(double newThickness) {
      for (ShellElement3d ele : mFemModel[0].getShellElements ()) {
         ele.setDefaultThickness (newThickness);
      }
   }
   
}

package artisynth.demos.growth.models.base;

import java.awt.Color;
import java.util.LinkedHashMap;
import java.util.LinkedList;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionBehavior.Method;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionManager.ColliderType;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.demos.growth.GrowColorer;
import artisynth.demos.growth.GrowLinearMaterial;
import artisynth.demos.growth.GrowModel3d;
import artisynth.demos.growth.GrowNode3d;
import artisynth.demos.growth.GrowRemesher;
import artisynth.demos.growth.GrowTriElement;
import artisynth.demos.growth.Morphogen2GrowthTensor;
import artisynth.demos.growth.PlasticEmbedder;
import artisynth.demos.growth.collision.CollisionDetector;
import artisynth.demos.growth.collision.SweptMeshInfo;
import artisynth.demos.growth.diffusion.Diffusion;
import artisynth.demos.growth.diffusion.MeshChemicals;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

/**
 * Demo that can perform growth experiments. 
 * 
 * In order to reproduce the extended demos exactly, multi-threading needs to be 
 * disabled by using these command-line arguments:
 * 
 *    -disableHybridSolves
 *    -numSolverThreads 1
 * 
 * DEV NOTES:
 *   - Polarity direction is set explicitly before each growth step.
 *     
 *   - ContinuousCollider.myUnilaterals = masterCC; is now in effect in 
 *     MechSystemSolver.java
 *     Remesh will now actually interpolate the unilaterals.
 *     
 *   - Revamped merging of imminent and actual constraints, and their 
 *     interpolation in remesher.
 *     
 *   - When (un)commenting collision code, check GrowDemo.java and 
 *     ShellRemeshOps.java
 */
public class GrowDemo extends ShellPatch {
   
   /* --- Remesher --- */
   
   /** Handles remeshing logic. */
   protected GrowRemesher mRemesher;
   
   /** Remeshing frequency. Currently set to refresh every 3 frames. */
   protected double mRemeshFreq = 0.03;           
   
   /** Maximum number of times to remesh. Mostly for debugging. */
   protected int mNumRemeshRemaining = Integer.MAX_VALUE;
 
   /* --- Sizing Field Parameters for Remesher --- */
   
   /** Weighing for the curvature metrics. Higher weight -> less refinement 
    *  at curved areas. */
   protected static double mRefineAngle = 0.2;
   
   /** Weighing for the strain metrics. Higher weight -> less refinement 
    *  at strained areas. */
   protected static double mRefineCompression = 0.01;
   
   /** Weighing for the velocity metrics. Higher weight -> less refinement 
    * at moving areas. */
   protected static double mRefineVelocity = 1;
   
   /** Minimum edge size. */
   protected static double mSizeMin = 0.05;
   
   /** Maximum edge size. */
   protected static double mSizeMax = mSizeMin*5;
   
   /** Minimum aspect ratio. Should leave at 0.5 for symmetrical. */
   protected static double mAspectMin = 0.5;
   
   /** How much should morphogen concentration influence the mesh resolution? */
   protected static double mRefineMorphogen = 0;  // Disabled.
   
   /* --- Diffusion --- */
   
   /** Handles diffusion of morphogen. */
   protected Diffusion mDiffusion;
   
   /** Scaler for the diffusion time step. 
    * 
    * Diffusion timestep := (t1-t0)*mDiffusionTimestepScale
    */
   protected double mDiffusionTimestepScale = 0.01;   
   
   /* --- Morphogen --- */

   /** Chemicals that are associated with the nodes. Each array index 
    *  corresponds to a particular model. */
   protected MeshChemicals[] mMeshChems;
   
   /** Number of different morphogen types. (PAR, PER, NOR, DF). DF is a 
    *  special diffusible morphogen that is essentially converted into PAR and 
    *  PER when absorbed, allowing us to perform a single diffusion simulation 
    *  for both PAR and PER. */
   protected int mNumChemTypes = 4;
   
   /** Fraction of morphogen (DF) converted into its activated form 
       (PAR, PER) per timestep. */
   protected double mChemCvtRate = 0.001;
   
   /** Which morphogen to diffuse? (DF). */
   protected int mChemTypeToDiffuse = 3;
   
   /** Should PAR morphogen be generated whenever morphogen DF is converted? */
   protected boolean mIsActivatePAR = true;
   
   /** Should PER morphogen be generated whenever morphogen DF is converted? */
   protected boolean mIsActivatePER = true;
   
   /** Should NOR morphogen be generated whenever morphogen DF is converted? */
   protected boolean mIsActivateNOR = false;
   
   /* --- Growth --- */
   
   /** Growth direction of PAR morphogen.
    *  PER morphogen growth direction is the cross-product of the 
    *  element's normal and mPolDir. */
   protected static Vector3d mPolDir = new Vector3d(0,1,0);
   
   /* --- Reference-space mesh (for display only) --- */
   
   /** Mesh that represents the reference configuration of the model. */
   protected PolygonalMesh[] mRefMesh;
   
   /** Rigid body that contains the reference mesh. */
   protected RigidBody[] mRefRB;
   
   /** Where should the reference mesh should be placed, relative to the 
    *  model in world-space. */
   protected Vector3d mRefMeshOffset = new Vector3d(0,0,-1);
   
   /** Should the reference mesh be displayed? */
   protected boolean mIsShowRefMesh = true;
   
   /* --- Morphogen-to-Plastic-Strain and Plastic Embedding --- */
   
   protected Morphogen2GrowthTensor mMorphogen2GrowthTensor;
   protected PlasticEmbedder mPlasticEmbedder;
   
   /* --- Collision --- */
   
   /** Collision settings between each pair of models, including self 
    *  collisions. */
   protected CollisionBehavior[] mCsnBeh;
   
   /** Minimum distance that the features should be pushed apart. */
   protected double mPenetrationTol = -1e-2;    
   
   /** Friction magnitude scale. */
   protected double mFriction = 0;
   
   /** Detect and repel nearby features? */
   protected boolean mEnableProximityDetection = true;
   
   /** Detect and resolve colliding features? */
   protected boolean mEnableContinuousDetection = true;
   
   /** Resolve colliding features, one impact-zone at a time? */
   protected boolean mEnableImpactZoneDetection = true;
   
   /** Maximum number of continuous collision handling loops. */
   protected int mMaxContinuousIters = 5;
   
   /** Enable self collisions? */
   protected static boolean mEnableSelfCollision = true;
   
   /* --- Incision --- */
   
   /** When a incision occurs at a node (which is replaced with two nodes), 
    *  much should the two nodes be pushed apart? */
   protected double mSepScaleForCutNodes = 0.05;
   
   /* --- Render --- */
   
   /** How should the model be displayed? 
    * 
    * DEFAULT    :: Edges and vertices shown.
    * MORPHOLOGY :: Smooth with no visible discretization.
    * TOPOLOGY   :: Edges shown with no shaders.  
    * */
   protected enum RenderMode {DEFAULT, MORPHOLOGY, TOPOLOGY}
   protected static RenderMode mRenderMode = RenderMode.DEFAULT;
   
   /** Configurations for each RenderMode. */
   protected static LinkedHashMap<RenderMode, RenderConfig> mRendCfgPresets;
   
   /* --- Morphogen concentration and plastic strain visualization. --- */
   
   /** Handles coloring the model, based on morphogen concentration or plastic 
    *  strain magnitude. */
   protected GrowColorer mGrowColorer;
   
   /** Plastic deformation gradients with a determinant that exceeds this is 
    *  considered "high" magnitude. */
   protected static double mMaxPlasticStrainColorBarRange = 1.1;
   
   /** Show the color bar? */
   protected boolean mShowColorBar = true;
   
   public enum SurfaceColor { DEFAULT, PLASTIC_STRAIN, MORPHOGEN }
   
   /** Color the surface according to the plastic strain or morphogen 
    *  concentration? If no coloring, use DEFAULT. */
   protected static SurfaceColor mSurfaceColor = SurfaceColor.DEFAULT; 
   
   /* --- UI Controls --- */
   
   protected static boolean mEnableDiffusion = true;
   protected static boolean mEnableGrowth = true;
   protected static boolean mEnableRemesh = true;
   protected static boolean mEnablePlasticEmbedding = true; 
   protected static boolean mEnableCollisionHandling = true;
   
   /** If true, do not discard the residual strain. */
   protected static boolean mMaintainResidualStrain = true;
   
   /** When toggled to true, currently selected nodes are incised. Resets back
    *  to false after incision. */
   protected static boolean mCutSelectedNodes = false;
   
   protected static boolean mCustom = false;
   
   /* --- Camera --- */
   
   /** Starting camera direction. */
   protected Point3d mCameraEye;
   
   /** Starting camera position. */
   protected Point3d mCameraCenter;
   
   /* --- Color nodes according to the specific morphogen concentration. --- */
   
   public enum NodeColor { 
      PAR(0), PER(1), NOR(2), DF(3);
      private final int value;
      private NodeColor(int value) { this.value = value; }
      public int getValue() { return value; }   
   }
   
   /** Nodes will be colored according to this morphogen concentration. */
   protected static NodeColor mNodeColor = NodeColor.DF;
   
 
   
   
   
   protected void build_pre() {
      super.build_pre();

      mGravity = new Vector3d(0,0,0);
      
      // Geometry
      mMeshX = 1;
      mMeshY = 1;
      mMeshXDiv = 5;  
      mMeshYDiv = 5;
      
      // Remesh 
      mRefineAngle = 0.2;
      mRefineCompression = 0.01;
      mRefineVelocity = 1;
      mSizeMin = 0.05;    // 0.05
      mSizeMax = mSizeMin*5;
      mAspectMin = 1;
      
      // Material
      m_shellThickness = 0.001;
      m_youngsModulus = 1e3;
   }
   
   protected void build_modelSkeleton() {
      super.build_modelSkeleton ();
   }

   protected void build_modelStructure() {
      super.build_modelStructure ();
      
      // Create the displayable reference mesh.
      mRefMesh = new PolygonalMesh[M];
      mRefRB = new RigidBody[M];
      for (int m=0; m<M; m++) {
         mRefMesh[m] = new PolygonalMesh();
         mRefRB[m] = new RigidBody();
         mRefRB[m].setSurfaceMesh (mRefMesh[m]);
         mRefRB[m].setDynamic (false);
         mRefRB[m].getRenderProps().setFaceStyle(FaceStyle.FRONT_AND_BACK);
         mRefRB[m].getRenderProps().setBackColor(Color.PINK);
         mRefRB[m].getRenderProps().setEdgeColor(Color.BLACK);
         mRefRB[m].getRenderProps().setDrawEdges(true);
         mRefRB[m].getRenderProps().setAlpha (1.0);
      }          
   }
   
   protected void build_modelProperties() {
      super.build_modelProperties();
      
      for (int m=0; m<M; m++) {
         mFemModel[m].setMaterial (
            new GrowLinearMaterial(m_youngsModulus, m_poissonsRatio));
      }
   }
   
   protected void build_utils() {
      
      // For each model...
      mMeshChems = new MeshChemicals[M];
      for (int m=0; m<M; m++) {
         // Create a per-node morphogen stash.
         mMeshChems[m] = new MeshChemicals(mFemModel[m].numNodes (), mNumChemTypes);
         
         // Pass morphogen references to each node.
         for (int n = 0; n < mFemModel[m].numNodes (); n++) {
            GrowNode3d gNode = (GrowNode3d)mFemModel[m].getNode (n);
            gNode.mChems = mMeshChems[m].getVtxChems (n);
         }
      }
      
      // Initialize the components of growth.
      
      mDiffusion = new Diffusion(mMesh[0], mMeshChems[0], 
         new int[] {mChemTypeToDiffuse});
      mMorphogen2GrowthTensor = new Morphogen2GrowthTensor(
         (GrowModel3d)mFemModel[0], mMesh[0]);
      mRemesher = new GrowRemesher(mMesh[0], mFemModel[0], 
         mSizeMin, mSizeMax, mAspectMin, mRefineAngle, mRefineCompression,
         mRefineVelocity, mRefineMorphogen, mMeshChems[0]);
      mGrowColorer = new GrowColorer(this, (GrowModel3d)mFemModel[0], mShowColorBar);
      
      mPlasticEmbedder = new PlasticEmbedder();
      mPlasticEmbedder.setTarget (mFemModel[0], mMesh[0]);
   }
   
   protected void build_renderConfig() {   
      // Default
      
      RenderConfig defaultCfg = new RenderConfig();
      defaultCfg.mNodeRadius = 0.005;  
      defaultCfg.mDirectorLen = 1;
      mRendCfgPresets = new LinkedHashMap<RenderMode, RenderConfig>();
      mRendCfgPresets.put (RenderMode.DEFAULT, defaultCfg);
      
      // Morphology
      
      RenderConfig morphCfg = new RenderConfig();
      morphCfg.mNodeRadius = 0;
      morphCfg.mDirectorLen = 0;
      morphCfg.mShading = Shading.SMOOTH;
      morphCfg.mLineWidth = 0;
      mRendCfgPresets.put (RenderMode.MORPHOLOGY, morphCfg);
      
      // Topology 
      
      RenderConfig topologyCfg = new RenderConfig();
      topologyCfg.mFrontMeshColor = Color.LIGHT_GRAY;
      topologyCfg.mNodeRadius = 0;
      topologyCfg.mDirectorLen = 0;
      topologyCfg.mShading = Shading.NONE;
      topologyCfg.mLineWidth = 1;
      mRendCfgPresets.put (RenderMode.TOPOLOGY, topologyCfg);
      
      // Use 
      
      mRendCfg = mRendCfgPresets.get (mRenderMode);
   }
   
   protected void build_assembleUI() {
      mPanel = new ControlPanel("cp");
      
      mPanel.addWidget (this, "enableDiffusion");
      mPanel.addWidget (this, "enableGrowth");
      mPanel.addWidget (this, "enableRemesh");
      mPanel.addWidget (this, "enablePlasticEmbedding");
      mPanel.addWidget (this, "enableCollisionHandling");
      mPanel.addWidget (this, "enableSelfCollision");
      mPanel.addWidget (this, "maintainResidualStrain");

      mPanel.addWidget (this, "surfaceColor");
      mPanel.addWidget (this, "nodeColor");
      mPanel.addWidget (this, "renderMode");
      
      mPanel.addWidget (this, "SF_refineAngle");
      mPanel.addWidget (this, "SF_refineCompression");
      mPanel.addWidget (this, "SF_refineVelocity");
      mPanel.addWidget (this, "SF_sizeMin");
      mPanel.addWidget (this, "SF_sizeMax");
      mPanel.addWidget (this, "SF_aspectMin");
      
      mPanel.addWidget (this, "polDir");
      
      mPanel.addWidget (this, "maxPlasticStrainColorBarRange");
      
      mPanel.addWidget (this, "cutSelectedNodes");
      mPanel.addWidget (this, "custom");
   }
   
   protected void build_post() {
      mCsnBeh = null;
   }
   
   /* --- Helper methods, which can overridden, for building --- */
   
   protected FemModel3d createFemModel() {
      return new GrowModel3d();
   }
   
   protected GrowNode3d createNode(Point3d pt) {
      return new GrowNode3d(pt, new VectorNd(mNumChemTypes), m_isMembrane);
   }
   
   protected GrowTriElement createElement(FemNode3d n0, FemNode3d n1,
   FemNode3d n2, double thickness)
   {
      return new GrowTriElement(
         (GrowNode3d)n0, 
         (GrowNode3d)n1,
         (GrowNode3d)n2, thickness, m_isMembrane);
   }
   
   public boolean shouldBeFrozen(int idx) {
      return false;
   }
   
   
   /* --- Properties (to hook up with Artisynth's UI system) --- */
   
   public static PropertyList myProps =
      new PropertyList (GrowDemo.class, ShellPatch.class);

   static {
      myProps.add("enableDiffusion", "", mEnableDiffusion);
      myProps.add("enableGrowth", "", mEnableGrowth);
      myProps.add("enableRemesh", "", mEnableRemesh);
      myProps.add("enablePlasticEmbedding", "", mEnablePlasticEmbedding);
      myProps.add("enableCollisionHandling", "", mEnableCollisionHandling);
      myProps.add("enableSelfCollision", "", mEnableSelfCollision);
      myProps.add("maintainResidualStrain", "", mMaintainResidualStrain);

      myProps.add("renderMode", "", mRenderMode);
      myProps.add("surfaceColor", "", mSurfaceColor);
      myProps.add("nodeColor", "", mNodeColor);
      
      myProps.add("SF_refineAngle", "", mRefineAngle);
      myProps.add("SF_refineCompression", "", mRefineCompression);
      myProps.add("SF_refineVelocity", "", mRefineVelocity);
      myProps.add("SF_sizeMin", "", mSizeMin);
      myProps.add("SF_sizeMax", "", mSizeMax);
      myProps.add("SF_aspectMin", "", mAspectMin);
      
      myProps.add("polDir", "", mPolDir);
      
      myProps.add ("maxPlasticStrainColorBarRange", "", mMaxPlasticStrainColorBarRange, "[1,2]");
      
      myProps.add("cutSelectedNodes", "", mCutSelectedNodes);
      myProps.add("custom", "", mCustom);
   }
   
   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public boolean getEnableDiffusion() { return mEnableDiffusion; }
   public void setEnableDiffusion(boolean val) { mEnableDiffusion = val; }
   public boolean getEnableGrowth() { return mEnableGrowth; }
   public void setEnableGrowth(boolean val) { mEnableGrowth = val; }
   public boolean getEnableRemesh() { return mEnableRemesh; }
   public void setEnableRemesh(boolean val) { mEnableRemesh = val; }
   public boolean getEnablePlasticEmbedding() { return mEnablePlasticEmbedding; }
   public void setEnablePlasticEmbedding(boolean val) { mEnablePlasticEmbedding = val; }
   public boolean getEnableCollisionHandling() { return mEnableCollisionHandling; }
   public void setEnableCollisionHandling(boolean val) { mEnableCollisionHandling = val; }
   public boolean getEnableSelfCollision() { return mEnableSelfCollision; }
   public void setEnableSelfCollision(boolean val) { mEnableSelfCollision = val; }
   public boolean getMaintainResidualStrain() { return mMaintainResidualStrain; }
   public void setMaintainResidualStrain(boolean val) { mMaintainResidualStrain = val; }
   
   public RenderMode getRenderMode() { return mRenderMode; }
   public void setRenderMode(RenderMode val) { 
      mRenderMode = val;
      mRendCfg = mRendCfgPresets.get (mRenderMode);
      build_femRendering ();
   }
   public NodeColor getNodeColor() { return mNodeColor; }
   public void setNodeColor(NodeColor val) { mNodeColor = val; }
   public SurfaceColor getSurfaceColor() { return mSurfaceColor; }
   public void setSurfaceColor(SurfaceColor val) { mSurfaceColor = val; }
   
   public double getSF_refineAngle() { return mRefineAngle; }
   public void   setSF_refineAngle(double val) { mRefineAngle = val; }
   public double getSF_refineCompression() { return mRefineCompression; }
   public void   setSF_refineCompression(double val) { mRefineCompression = val; }
   public double getSF_refineVelocity() { return mRefineVelocity; }
   public void   setSF_refineVelocity(double val) { mRefineVelocity = val; }
   
   public double getSF_sizeMin () { return mSizeMin; }
   public void   setSF_sizeMin (double val) { mSizeMin = val; }
   public double getSF_sizeMax () { return mSizeMax; }
   public void   setSF_sizeMax (double val) { mSizeMax = val; }
   public double getSF_aspectMin () { return mAspectMin; }
   public void   setSF_aspectMin (double val) { mAspectMin = val; }
   
   public Vector3d getPolDir () { return mPolDir; }
   public void   setPolDir (Vector3d val) { mPolDir.set(val); }
   
   public double getMaxPlasticStrainColorBarRange () { return mMaxPlasticStrainColorBarRange; }
   public void   setMaxPlasticStrainColorBarRange (double val) { 
      mMaxPlasticStrainColorBarRange = val;
      mGrowColorer.mMaxPlasticStrainColorBarRange = mMaxPlasticStrainColorBarRange;
   }
   
   public boolean getCutSelectedNodes() {return mCutSelectedNodes;}
   public void setCutSelectedNodes(boolean val) {
      mCutSelectedNodes = false;
      inciseSelectedNodes();
   }
   public boolean getCustom() { return mCustom; }
   public void setCustom(boolean val) { mCustom = val; }
  

   
   /* --- Time Step Logic --- */
   
   /** Number of simulated seconds before simulation is paused. */
   protected double mPauseEveryInterval = 999.00;     
   
   /** Time at the beginning of the simulation. */
   protected long mStartTime = System.currentTimeMillis ();
   
   /** Time it took to simulate the previous time step. */
   protected long mPrvElapsedTime = 0;
   
   /** Time it took to simulate the current time step. */
   protected long mCurElapsedTime = 0;
   
   public void advanceCustom(double t0, double t1, int flags) {
      if (t0 < 0.01) {
         if (mCameraEye != null) {
            Main.getMain ().getViewer ().setEye (mCameraEye);
            Main.getMain ().getViewer ().setCenter (mCameraCenter);
         }
         
         Main.getMain ().getViewer ().setBackgroundColor (mRendCfg.mBackgroundColor);
      }
      
      // Pause the simulation if so.
      if (MathUtil.compare (t0 % mPauseEveryInterval, 0) == 0 && t0 != 0) {
         setStopRequest (true);
      }
      
      // Setup the collision behavior for each pair of models.
      toggleCollisionBehavior(mEnableCollisionHandling);
      
      if (mIsShowRefMesh) 
         updateReferenceMesh();
      
      if (mCustom) {
         customLogic();
         mCustom = false;
      }
      
      // Used to measure the time to simulate each component (e.g. diffusion).
      double start = 0;
      double end = 0;
      
      // Handle diffusion.
      if (mEnableDiffusion) {
         start = System.currentTimeMillis ();
         for (int m=0; m<M; m++) {
            if (! mFemModel[m].getDynamicsEnabled ()) 
               continue;
            
            mDiffusion.setTarget (mMesh[m], mMeshChems[m]);
            mDiffusion.advance ((t1-t0)*mDiffusionTimestepScale);
         }
         customPostDiffusion();
         end = System.currentTimeMillis ();
         System.out.println ("Diffusion solve (ms): " + (end-start));
      }
      
      // Handle remeshing.
      if (MathUtil.compare (t0 % mRemeshFreq, 0) == 0 && mEnableRemesh &&
          mNumRemeshRemaining > 0 && t0 > 0.01) 
      {
         start = System.currentTimeMillis ();
         
         mRemesher.mSizingField.updateParameters (mSizeMin, mSizeMax, mAspectMin,
            mRefineAngle, mRefineCompression, mRefineVelocity);
         
         for (int m=0; m<M; m++) {
            if (! mFemModel[m].getDynamicsEnabled ()) 
               continue;
            
            mRemesher.setTarget (mMesh[m], mFemModel[m], mMeshChems[m]);
            
            // Ensure constraints are interpolated during remeshing.
            // CSNCMT
            mRemesher.setConstraints ( CollisionDetector.myCCAgg );
            
            mRemesher.remesh ();   
            ShellUtil.invalidateFem (mFemModel[m]);
            
            // Make sure collision detection uses updated surface mesh.
            // CSNCMT
            CollisionManager colMgr = mMechModel.getCollisionManager ();
            CollisionDetector contCldr = colMgr.getContinuousCollider ();
            if (contCldr != null && mRemesher.isEleModified)
               contCldr.rebuildSweptMeshInfo (mFemModel[m].getMeshComp (0));
         }

         mNumRemeshRemaining--;
         System.out.printf ("Remesh occurred. Eles: %d, Nodes: %d \n", 
            getNumShellElements(), getNumNodes());

         end = System.currentTimeMillis ();
         System.out.println ("Remesh solve (ms): " + (end-start));
      }
      
      // Convert fraction of morphogen into plastic strain.
      if (mEnableGrowth) {
         start = System.currentTimeMillis ();
         
         for (int m=0; m<M; m++) {
            if (! mFemModel[m].getDynamicsEnabled ()) 
               continue;
            
            mMorphogen2GrowthTensor.setTarget (mFemModel[m]);
            
            if (! mMaintainResidualStrain)
               mMorphogen2GrowthTensor.unapplyGrowthTensors ();
            
            mMorphogen2GrowthTensor.updatePolarityDirection(mPolDir);
            mMorphogen2GrowthTensor.activateFractionOfMorphogen (mChemCvtRate, 
               mIsActivatePAR, mIsActivatePER, mIsActivateNOR);
            mMorphogen2GrowthTensor.computeGrowthTensors ();
            mMorphogen2GrowthTensor.applyGrowthTensors (); 
            mMorphogen2GrowthTensor.clearActivatedMorphogen ();   
         }
         
         end = System.currentTimeMillis ();
         System.out.println ("Tensor solve (ms): " + (end-start));
      }
      
      // Plastic Embedding
      if (mEnablePlasticEmbedding) {
         start = System.currentTimeMillis ();
         
         for (int m=0; m<M; m++) {
            if (! mFemModel[m].getDynamicsEnabled ()) 
               continue;
            
            // Use this instead if simulating both reference and world-space.
   //         mPlasticEmbedder.setTarget (mFemModel);
   //         mPlasticEmbedder.advance (t0, t1);
   //         mPlasticEmbedder.preadvance (); 
            
            mPlasticEmbedder.setTarget (mFemModel[m]);
            mPlasticEmbedder.copyWorldSpaceToReferenceSpace ();
         }
         
         end = System.currentTimeMillis ();
         System.out.println ("PlasticEmbedding solve (ms): " + (end-start));
      }
      else {
//         mMorphogen2GrowthTensor.unapplyGrowthTensors ();
//         mPlasticEmbedder.backupWorldSpace ();
//         mPlasticEmbedder.worldSpaceMode ();
      }
      
      updateNodesColor();
      
      if (mSurfaceColor == SurfaceColor.PLASTIC_STRAIN) 
         mGrowColorer.computePlasticStrainColors ();
      else if (mSurfaceColor == SurfaceColor.MORPHOGEN) 
         mGrowColorer.computeMorphogenColors ();
      else if (mGrowColorer != null)
         mGrowColorer.toggleOff ();
      
      mCurElapsedTime = System.currentTimeMillis () - mStartTime;
      System.out.println ("Time Elapsed (ms): " + mCurElapsedTime);
      System.out.println ("Time step solve (ms): " + 
         (mCurElapsedTime - mPrvElapsedTime));
      mPrvElapsedTime = mCurElapsedTime;
   }
   
   public StepAdjustment advance (double t0, double t1, int flags) {  
      advanceCustom(t0, t1, flags);
      return super.advance(t0, t1, flags);
   }

   
   /* --- Helper Functions --- */
   
   /** Sync the reference mesh with the reference-space of the model. */
   protected void updateReferenceMesh() {
      for (int m=0; m<M; m++) {
         mRefMesh[m].clear ();
         
         // Copy nodes
         for (int v = 0; v < mMesh[m].numVertices (); v++) {
            FemNode3d node = mFemModel[m].getNode (v);
            GrowNode3d gNode = (GrowNode3d)node;
            
            Point3d plasticPt = new Point3d(gNode.getRestPosition ());
           
            Vertex3d copyVtx = new Vertex3d( plasticPt );
            copyVtx.getPosition().add (mRefMeshOffset);
            copyVtx.setIndex (v);
            mRefMesh[m].addVertex (copyVtx);
         }
        
         // Copy faces
         for (Face face : mMesh[m].getFaces ()) {
            mRefMesh[m].addFace (face.getVertexIndices ());
         }
      }
   }
   
   /** Update the color of the nodes, typically according to the morphogen (DF) 
    *  concentration. */
   protected void updateNodesColor() {
      for (int m=0; m<M; m++) {
         for (FemNode3d node : mFemModel[m].getNodes ()) {
            double amt = ((GrowNode3d)node).mChems.get (mNodeColor.getValue ());
            float amtPct = (float)convertAmtToColorSaturation(amt);
            RenderProps.setPointColor(node, Color.getHSBColor (1.f, amtPct, 1.f));
         }
      }
   }
   
   /** Convert the given morphogen concentration to a fraction. */
   protected double convertAmtToColorSaturation(double amt) {
      double max = 1;
      return (amt > max) ? 1 :
             (amt < 0)   ? 0 :
                    amt / max;
   }
   
   /** Turn on or off collision handling. */
   protected void toggleCollisionBehavior(boolean isEnable) {
      // CSNCMT
      if (isEnable && mCsnBeh == null) {
         CollisionManager colMgr = mMechModel.getCollisionManager ();
         colMgr.setColliderType (ColliderType.CONTINUOUS);
         
         mCsnBeh = new CollisionBehavior[M*M];
         
         int c = 0;
         for (int m0=0; m0<M; m0++) {
            for (int m1=0; m1<M; m1++) {
                
               // Don't enable this collision pair if it's a self collision &&
               // self collisions are disabled.
               boolean isEnableM01 = !(m0==m1 && !mEnableSelfCollision);
               
               mCsnBeh[c] = new CollisionBehavior(true, 1);
               mCsnBeh[c].setCollidablePair (mFemModel[m0], mFemModel[m1]);
               mCsnBeh[c].setMethod (Method.VERTEX_EDGE_PENETRATION);
               mCsnBeh[c].setColliderType (ColliderType.CONTINUOUS);
               mCsnBeh[c].setPenetrationTol (mPenetrationTol); 
               mCsnBeh[c].setFriction (mFriction);
               mCsnBeh[c].setEnabled (isEnableM01);
               colMgr.setBehavior (mFemModel[m0], mFemModel[m1], mCsnBeh[c]);
               c++;
            }
         }
      }
      else if (mCsnBeh != null) {
         int c = 0;
         for (int m0=0; m0<M; m0++) {
            for (int m1=0; m1<M; m1++) {
               boolean isEnableM0M1 = !(m0==m1 && !mEnableSelfCollision);
               mCsnBeh[c].setEnabled (isEnable && isEnableM0M1);
               c++;
            }
         }
      }
      
      SweptMeshInfo.myMargin = Math.abs(mPenetrationTol)/2.0 + 1e-6; 
      CollisionDetector.myClothThickness = Math.abs (mPenetrationTol);
      
      CollisionDetector.mySpaceElipson = 1e-12;  // Dec if penetration occurring
      CollisionDetector.myEnableProximityDetection = mEnableProximityDetection;
      CollisionDetector.myEnableContinuousDetection = mEnableContinuousDetection;
      CollisionDetector.myEnableImpactZoneDetection = mEnableImpactZoneDetection;
      
      // After 5 iterations, use 1/2 impulse
      CollisionDetector.myMaxNumActualIters = mMaxContinuousIters;
   }
   

   /** Incise the currently selected nodes. */
   protected void inciseSelectedNodes() {
      System.out.println ("Performing incision...");

      LinkedList<FemNode3d> selectedNodes = new LinkedList<FemNode3d>();

      for (ModelComponent comp : 
           Main.getMain ().getSelectionManager ().getCurrentSelection ()) 
      {
         if (comp instanceof FemNode3d) {
            FemNode3d node = (FemNode3d)comp;
            System.out.printf ("Selected node to cut: %s\n", node.getName ());
            selectedNodes.add (node);
         }
      }
      
      if (selectedNodes.size () == 0) 
         return;
      
      FemModel3d femModel = (FemModel3d)selectedNodes.get (0)
         .getParent ().getParent ();
      int m = getIdx(femModel);
      
      mRemesher.mSizingField.updateParameters (mSizeMin, mSizeMax, mAspectMin,
         mRefineAngle, mRefineCompression, mRefineVelocity);
      mRemesher.setTarget (mMesh[m], mFemModel[m], mMeshChems[m]);
      
      // Ensure constraints are interpolated during remeshing.
      // CSNCMT
      mRemesher.setConstraints ( CollisionDetector.myCCAgg );
      
      mRemesher.mSepScaleForCutNodes = mSepScaleForCutNodes;
      mRemesher.cutNodes (selectedNodes);
      
      ShellUtil.invalidateFem (mFemModel[m]);
      
      // Make sure collision detection uses updated surface mesh.
      // CSNCMT
      CollisionManager colMgr = mMechModel.getCollisionManager ();
      CollisionDetector contCldr = colMgr.getContinuousCollider ();
      if (contCldr != null)
         contCldr.rebuildSweptMeshInfo (mFemModel[m].getMeshComp (0));
      
      if (mSurfaceColor == SurfaceColor.PLASTIC_STRAIN) 
         mGrowColorer.computePlasticStrainColors ();
      else if (mSurfaceColor == SurfaceColor.MORPHOGEN) 
         mGrowColorer.computeMorphogenColors ();
      else if (mGrowColorer != null)
         mGrowColorer.toggleOff ();
   }

   /* --- Miscellaneous. --- */
   
   protected int getNumShellElements() {
      int sum = 0;
      for (int m=0; m<M; m++) {
         sum += mFemModel[m].numShellElements ();
      }
      return sum;
   }
   
   protected int getNumNodes() {
      int sum = 0;
      for (int m=0; m<M; m++) {
         sum += mFemModel[m].numNodes ();
      }
      return sum;
   }
   
   protected void customLogic() {
      
   }
   
   protected void customPostDiffusion() {
      
   }
   
}

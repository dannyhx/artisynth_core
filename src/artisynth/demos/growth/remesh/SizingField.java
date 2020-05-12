package artisynth.demos.growth.remesh;

import static java.lang.Math.max;
import static java.lang.Math.pow;

import java.util.ArrayList;
import java.util.HashMap;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.ShellElement3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.demos.growth.util.MathUtil;
import artisynth.demos.growth.util.ShellUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.EigenDecomposition;
import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix2x3;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x2;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/**
 * Measures the amount of refinement needed for a given model.
 * 
 * The sizing field (measured at face/element and later interpolated to each
 * node) is a quantity of needed refinement.
 * 
 * Implementation details:
 * 
 * After element-based sizing field is computed:
 *   - Eigenvalues are clamped between 1/sizeMax^2 to 1/sizeMin^2
 *   - Minimum eigenvalue is set to max(eigenValues) * aspectMin^2
 *   
 * Edge will not collapse if:
 *   - New face rest area < Old face rest area AND new face rest area < 0.1*mSizeMin^2
 *   - New face rest aspect ratio < mAspectMin
 *   
 * sizeMin :: maximum s
 * sizeMax :: minimum s
 */
public class SizingField {

   protected final boolean DEBUG = false;
   
   protected PolygonalMesh mMesh;
   protected FemModel3d mFemModel;
   
   /** Sizing field of each element. */
   public ArrayList<Matrix3d> mEleSFs;
   
   /** Sizing field of each node. Interpolated from elements' sizing fields. */
   public ArrayList<Matrix3d> mNodeSFs;
   
   // Adjust the weights of the metrics. Increasing the weights have an 
   // effect of decreasing the sizing field.
   
   protected double mRefineAngle;
   protected double mRefineCompression;
   protected double mRefineVelocity;
   
   /** Set the minimum sizing field. */
   public double mSizeMin;
   
   /** Set the maximum sizing field. */ 
   protected double mSizeMax;
   
   /** Targeted ratio between the minimum and maximum edge length of a
    *  triangle. 0.5 is recommended to preserve symmetry.  */
   public double mAspectMin;
   
   // Optional: A specific resolution can be requested at marked elements.
   // If an element contains at least one node flagged with a FLAG_NODE_UNIFORM 
   // or FLAG_NODE_RESOLVE_MAX, then the diagonal entries of the element's 
   // sizing field will be overridden:
   //    FLAG_NODE_UNIFORM     -> mSizeUniform
   //    FLAG_NODE_RESOLVE_MAX -> mSizeMin
   // This optional feature has not been used.
   
   protected double mSizeUniform;
   
   /** Nodes flagged with this will have their elements' sizing field be 
    *  locked to mSizeUniform. */
   protected static final int FLAG_NODE_UNIFORM;
   
   /** Nodes flagged with this will have their elements' sizing field be 
    *  locked to mSizeMin. */
   protected static final int FLAG_NODE_RESOLVE_MAX;
   
   
   
   static {
      FLAG_NODE_UNIFORM = ModelComponentBase.createTempFlag ();
      FLAG_NODE_RESOLVE_MAX = ModelComponentBase.createTempFlag ();
   }
   
   public SizingField(PolygonalMesh mesh, FemModel3d femModel) {
      setTarget(mesh, femModel);
   
      // Paper folding settings:  fold.json
      mRefineAngle = 0.2;       // 0.2
      mRefineCompression = 1;   // 0.1e-3
      mSizeMin = 0.05;          // 3e-3
      mSizeMax = mSizeMin*2.5;  // 100e-3
      mAspectMin = 0.2;         // 0.2
      
      mRefineVelocity = 1;      // 1
      
      mEleSFs = new ArrayList<Matrix3d>(mFemModel.numShellElements ());
      mNodeSFs = new ArrayList<Matrix3d>(mFemModel.numNodes ());
   }
   
   public SizingField(PolygonalMesh mesh, FemModel3d femModel, double sizeMin,
   double sizeMax, double aspectMin, double refineAngle, double refineComp, 
   double refineVelocity) {
      setTarget(mesh, femModel);
      
      mRefineAngle = refineAngle;       
      mRefineCompression = refineComp;  
      mSizeMin = sizeMin;     
      mSizeMax = sizeMax;   
      mAspectMin = aspectMin;
      
      mRefineVelocity = refineVelocity;
      
      mEleSFs = new ArrayList<Matrix3d>(mFemModel.numShellElements ());
      mNodeSFs = new ArrayList<Matrix3d>(mFemModel.numNodes ());
   }
   
   public void setTarget(PolygonalMesh mesh, FemModel3d femModel) {
      mMesh = mesh;
      mFemModel = femModel;
   }
   
   public void updateParameters(double sizeMin, double sizeMax, double aspectMin,
   double refineAngle, double refineComp, double refineVelocity) {
      mRefineAngle = refineAngle;       
      mRefineCompression = refineComp;  
      mSizeMin = sizeMin;     
      mSizeMax = sizeMax;   
      mAspectMin = aspectMin;
      
      mRefineVelocity = refineVelocity;
   }
   
   
   
   /** 
    * Calculate the sizing field of each vertex/node.  
    */
   public void computeVertexSizingFields() {
      mEleSFs.clear ();
      mNodeSFs.clear ();
      
      // Calculate sizing field of each face/element.
      for (int f = 0; f < mMesh.numFaces (); f++) {
         Face face = mMesh.getFace (f);
         mEleSFs.add (createFaceSizingField(face));
      }
      
      // Mapping from element addresses to their indices in mEleSFs.
      HashMap<ShellElement3d,Integer> e2iMap = new HashMap<ShellElement3d,Integer>();
      for (int e = 0; e < mFemModel.numShellElements (); e++) {
         e2iMap.put (mFemModel.getShellElement (e) , e);
      }
      
      // Interpolate the sizing fields to the nodes.
      for (int n = 0; n < mFemModel.numNodes (); n++) {
         FemNode3d node = mFemModel.getNode (n);
         
         Matrix3d nodeSF = new Matrix3d();
         double areaSum = 0;
         
         for (ShellElement3d adjEle : node.getAdjacentShellElements ()) {
            double eleArea = ShellUtil.area (adjEle.getNodes (), true /*isRest*/); // <MS>
           
            int e = e2iMap.get (adjEle);
            Matrix3d adjEleSF = mEleSFs.get (e);
           
            nodeSF.scaledAdd (eleArea, adjEleSF );
            areaSum += eleArea;
         }
         
         nodeSF.scale (1.0 / areaSum);
         mNodeSFs.add (nodeSF);
      }
   }
   
   public Matrix3d getVertexSF(int i) {
      return mNodeSFs.get (i);
   }
   
   public void addVertexSF(Matrix3d vSF) {
      mNodeSFs.add (new Matrix3d());
   }
   
   
   /* --- Protected 2nd functions --- */
   
   /**
    * Compute the amount of remeshing refinement needed at a particular face.
    * 
    * Amount of refinement needed is increased by:
    *   - 1st curvature metric :: depends on dihedral angles.
    *   - 2nd curvature metric :: depends on vertex normals.
    *   - Velocity metric.
    *   - Buckling tendency metric :: Relies of Fe*Fp in compression metric
    *       Essentially a modified version of the buckling suppression metric
    *       that promotes buckling/refinement instead.
    *   - SKIP: Obstruction
    *   - SKIP: Fracture
    * 
    * Follows dynamicremesh.cpp :: compute_face_sizing().
    *   Element's rest position == <PS>
    *   Element's world position == <WS>
    * 
    * @param restFace
    * Face to be measured, and having a corresponding shell element.
    * 
    * @return
    * 3x3 sizing field matrix of the face.
    */
   protected Matrix3d createFaceSizingField(Face restFace) {
      ShellTriElement ele = (ShellTriElement)mFemModel.getShellElement (restFace.idx);
      FemNode3d[] nodes = ele.getNodes ();
      IntegrationPoint3d ipnt = ele.getWarpingPoint ();
      IntegrationData3d idat = ele.getWarpingData ();
      
      idat.computeInverseRestJacobian (ipnt, ele.getNodes ());
      Matrix3d invJ0 = idat.getInvJ0 ();
      
      // Compute basis matrix that can project a vector/matrix into the plane
      // of the element.
      
      Matrix3d base = localBase(ShellUtil.getNormal (ele, true));    // <MS>
      
      Matrix3x2 UV = new Matrix3x2(); // Basis matrix with 3rd column discarded.
      UV.set(new double[] {
         base.m00, base.m01,
         base.m10, base.m11,  
         base.m20, base.m21});
      Matrix2x3 UV_t = new Matrix2x3();
      UV_t.transpose (UV);
       
      // 1st curvature metric :: depends on dihedral angles.
      
      Matrix2d Sw1 = projectedCurvature(restFace,   // <WS>
         ShellUtil.area(nodes, true /*isRest*/), 
         false /*isPlasticSpace*/, UV_t);
      
      Matrix2d Mcurvw1 = new Matrix2d();
      Mcurvw1.mulTransposeLeft (Sw1, Sw1);   // M' M is a method to make M 
                                             // symmetric -> ensures 
                                             // eigenvec/val real.
      Mcurvw1.scale ( 1.0 / pow (mRefineAngle, 2) );
      
      // 2nd curvature metric :: depends on vertex normals.
            
      Matrix3d Sw2 = new Matrix3d();
      Matrix3d _nrm_J = ShellUtil.createCustomJacobian (ipnt, nodes,
                                      new Vector3d[] {
                                        ShellUtil.getNormal (nodes[0], false), 
                                        ShellUtil.getNormal (nodes[1], false), 
                                        ShellUtil.getNormal (nodes[2], false)},
                                      null);
      Sw2.mul (_nrm_J, invJ0);
      
      MatrixNd Mcurvw2 = null;
      Matrix3d _Sw2t_Sw2 = new Matrix3d();
      _Sw2t_Sw2.mulTransposeLeft (Sw2, Sw2);
      Mcurvw2 = MathUtil.mul (UV_t, _Sw2t_Sw2);
      Mcurvw2 = MathUtil.mul (Mcurvw2, UV);
      Mcurvw2.scale ( 1.0 / pow (mRefineAngle, 2) );
     
      // Velocity metric.
      
      Matrix3d V = new Matrix3d();
      Matrix3d _vel_J = ShellUtil.createCustomJacobian (ipnt, nodes,
                                       new Vector3d[] {
                                        nodes[0].getVelocity (),
                                        nodes[1].getVelocity (),
                                        nodes[2].getVelocity ()},
                                       null);
      V.mul (_vel_J, invJ0);

      MatrixNd Mvel = null;
      Matrix3d _Vt_V = new Matrix3d();
      _Vt_V.mulTransposeLeft (V, V);
      Mvel = MathUtil.mul (UV_t, _Vt_V);
      Mvel = MathUtil.mul (Mvel, UV);
      Mvel.scale ( 1.0 / pow(mRefineVelocity, 2) );
      
      // Compression metric.
      
      // deformation_gradient<WS>():  computes gradient * residual stretch
      Matrix3d _F = new Matrix3d();  
      ipnt.computeGradient (_F, nodes, invJ0);   // Fe
      _F.mul ( ShellUtil.idatAvgFp (ele) );      // Fp

      Matrix2d Mcomp = compressionMetric (
         _F, _Sw2t_Sw2, UV_t, UV, mRefineCompression);
   
      // Mcomp can be replaced with a pure elastic strain metric and 
      // pure plastic strain metric approach:
//      
//      Matrix3d Fe = new Matrix3d();  
//      ipnt.computeGradient (Fe, nodes, invJ0);
//      Fe.sub (Matrix3d.IDENTITY);
//      MathUtil.abs (Fe);
//      
//      Matrix3d FeT_Fe = new Matrix3d();
//      FeT_Fe.mulTransposeLeft (Fe, Fe);
//      
//      MatrixNd Me = null;
//      Me = MathUtil.mul (UV_t, FeT_Fe);
//      Me = MathUtil.mul (Me, UV);
//      Me.scale ( 1.0 / pow(mRefineCompression, 2) );
//      
//      // Plastic strain metric.
//      
//      Matrix3d Fp = ShellUtil.idatAvgFp (ele);
//      Fp.sub (Matrix3d.IDENTITY);
//      MathUtil.abs (Fp);
//      
//      Matrix3d FpT_Fp = new Matrix3d();
//      FpT_Fp.mulTransposeLeft (Fp, Fp);
//      
//      MatrixNd Mp = null;
//      Mp = MathUtil.mul (UV_t, FpT_Fp);
//      Mp = MathUtil.mul (Mp, UV);
//      Mp.scale ( 1.0 / pow(mRefineCompression, 2) );
//
//      !!! Me and Mp are added to s below. 
      
      // Assumption:  combine_tensors == false (in original, it's true)
      Matrix2d s = new Matrix2d();
      s.add (Mcurvw1);          
      s.add (Mcurvw2);
      s.add (Mvel);
      s.add (Mcomp);
      
      // Specific resolution request.
      for (int i = 0; i < 3; i++) {
         if (nodes[i].checkFlag (FLAG_NODE_UNIFORM)) {
            double diagVal = 1.0 / pow (mSizeUniform, 2);
            s = new Matrix2d();
            s.setDiagonal (new double[] {diagVal,diagVal});
         }
         if (nodes[i].checkFlag (FLAG_NODE_RESOLVE_MAX)) {
            double diagVal = 1.0 / pow (mSizeMin, 2);
            s = new Matrix2d();
            s.setDiagonal (new double[] {diagVal,diagVal});
         }
      }
      
      if (DEBUG && restFace.idx == 0) {
         System.out.println ();
         System.out.println ("curvw1: " + MathUtil.normF (Mcurvw1));
         System.out.println ("curvw2: " + MathUtil.normF (Mcurvw2));
         System.out.println ("vel: " + MathUtil.normF (Mvel));
         System.out.println ("comp: " + MathUtil.normF (Mcomp));
         System.out.println ("s: " + MathUtil.normF (s));
      }
      
      // Decompose sizing field.
      EigenDecomposition eig = new EigenDecomposition();
      eig.factor (s);
      
      // Retrieve the eigen values and vectors
      VectorNd eigVals = new VectorNd(2);
      Matrix2d eigVecs = new Matrix2d();
      eig.get (eigVals, null, eigVecs);
      
      double[] eigValsBuf = eigVals.getBuffer ();
      
      for (int i = 0; i < 2; i++) {
         eigValsBuf[i] = MathUtil.clamp (
            eigValsBuf[i],
            1.0 / pow (mSizeMax, 2),
            1.0 / pow (mSizeMin, 2));
      }
      double lmax = max(eigValsBuf[0], eigValsBuf[1]);
      double lmin = lmax * pow (mAspectMin, 2);
      
      for (int i = 0; i < 2; i++) {
         if (eigValsBuf[i] < lmin) {
            eigValsBuf[i] = lmin;
         }
      }
      
      // Reassemble matrix from its eigenvectors and modified eigenvalues
      s.set (eigVecs);
      s.mul ( new Matrix2d(   
         eigValsBuf[0], 0, 
         0, eigValsBuf[1]));
      s.mulTranspose (eigVecs);
      
      // Reproject to 3D
      MatrixNd rv = MathUtil.mul (UV, s);
      rv = MathUtil.mul (rv, UV_t);
      return new Matrix3d(rv.getBuffer ());
   }
   
   /* --- Protected 3rd functions --- */
   
   /** Compute a 3x3 basis matrix that can project a vector or matrix into
    *  the plane of the element. */
   protected static Matrix3d localBase(Vector3d normal) {
      // geometry.cpp local_base
      
      Vector3d u =
         (normal.dot (Vector3d.X_UNIT) > normal.dot (Vector3d.Z_UNIT))
         ? new Vector3d(Vector3d.Z_UNIT) 
         : new Vector3d(Vector3d.X_UNIT);
      
      double uDotNormal = u.dot (normal);
      u.sub (new Vector3d(normal).scale (uDotNormal));
      u.normalize ();
      
      Vector3d v = new Vector3d().cross (normal, u);
       
      Matrix3d uvn = new Matrix3d();
      uvn.setColumns (u, v, normal);
      return uvn;
   }
   
   /** Compute the curavture metric, projected into 2D space. 
    * 
    * Curvature is measured using the hinge-angle of each edge of the given
    * face.
    */
   protected Matrix2d projectedCurvature(Face face, double msArea,
   boolean isPlasticSpace, Matrix2x3 base) {
      if (DEBUG && msArea <= 0) {
         System.out.println ("projectedCurvature() :: non-positive refArea");
      }
      
      Matrix2d s = new Matrix2d();
      
      // For each edge of the face...
      for (int e = 0; e < 3; e++) {
         HalfEdge edge = face.getEdge (e);

         // TODO: Assuming head <- tail. In original, PREV-NEXT
         Vector3d edgeVec3d = new Vector3d().sub ( 
            mFemModel.getNode( edge.head.getIndex () ).getRestPosition (),
            mFemModel.getNode( edge.tail.getIndex () ).getRestPosition ()
         );
         
         VectorNd e_mat = new VectorNd(2);
         base.mul (e_mat /*vr*/, new VectorNd(edgeVec3d));
         
         VectorNd t_mat = new VectorNd(e_mat);
         t_mat.normalize ();
         
         t_mat.set (new double[] {              // ::perp()
           -t_mat.get (1), t_mat.get (0) });
        
         double theta = dihedralAngle(edge, isPlasticSpace);    // <s>
         
         Matrix2d scaled_t_mat_outerProd = 
            MathUtil.outerProduct2x2 (t_mat.getBuffer (), t_mat.getBuffer ());
         scaled_t_mat_outerProd.scale (0.5 * theta * e_mat.norm ());
         
         s.sub (scaled_t_mat_outerProd);
      }
      
      s.scale ( 1.0 / msArea );
      return s;
   }
   
   /** Get the dihedral angle formed by two intersecting faces.
     *
     * Vector along the intersection is specified by p0 and p1.
     * Plane normals are n0 and n1.
     */
   public double dihedralAngle(Vector3d p0, Vector3d p1, Vector3d n0, Vector3d n1) {
      // geometry.cpp dihedral_angle
      
      Vector3d e = new Vector3d().sub (p1,p0).normalize ();
      if ( MathUtil.compare (e.normSquared (),0) == 0 ) {
         return 0;
      }
      if ( MathUtil.compare (n0.normSquared (),0) == 0 ||
           MathUtil.compare (n1.normSquared (),0) == 0 ) {
         return 0;
      }
      
      double cosine = n0.dot (n1);
      double sine = e.dot ( new Vector3d(n0).cross (n1) );
      double theta = Math.atan2 (sine, cosine);
      
      return theta;
   }
   
   /** 
    * Get the dihedral angle formed between two connected faces. 
    * 
    * @param edge 
    * The common edge of the two connected faces. 
    */
   public double dihedralAngle(HalfEdge edge, boolean isPlasticSpace) {
      // geometry.cpp dihedral_angle
      
      if (edge.getOppositeFace () == null) {
         return 0;
      }
      
      FemNode3d hNode = mFemModel.getNode (edge.head.getIndex ());
      FemNode3d tNode = mFemModel.getNode (edge.tail.getIndex ());
      
      // TODO: Assuming head <- tail
      Vector3d e = new Vector3d();
      if (isPlasticSpace) {
         e.sub (hNode.getRestPosition (), tNode.getRestPosition ());
      }
      else {
         e.sub (hNode.getPosition (), tNode.getPosition ());
      }
      e.normalize ();
      
      if ( MathUtil.compare (e.normSquared (),0) == 0 ) {
         return 0;
      } 
      
      // Face normals
      
      ShellTriElement ele = (ShellTriElement)mFemModel.getShellElement (
         edge.getFace ().idx);
      ShellTriElement oppEle = (ShellTriElement)mFemModel.getShellElement (
         edge.getOppositeFace ().idx);
      
      Vector3d n0 = null;
      Vector3d n1 = null;
      if (isPlasticSpace) {
         n0 = ShellUtil.getNormal (ele, true);
         n1 = ShellUtil.getNormal (oppEle, true);
      }
      else {
         n0 = ShellUtil.getNormal (ele, false);
         n1 = ShellUtil.getNormal (oppEle, false);
      }
      
      if ( MathUtil.compare (n0.normSquared (),0) == 0 || 
           MathUtil.compare (n1.normSquared (),0) == 0) {
         return 0;
      }
           
      Vector3d n0_x_n1 = new Vector3d().cross (n0, n1);
           
      double cosine = n0.dot (n1);
      double sine = e.dot (n0_x_n1);
      double theta = Math.atan2 (sine, cosine);
      
      return theta;
   }
   
   /** 
    * Buckling tendency metric, which is essentially a modified version of the
    * buckling suppression metric that promotes buckling/refinement instead.
    * 
    * @param F
    * Elastic and plastic deformation gradient multiplied together (Fe * Fp).
    * 
    * @param S2 
    * 2nd curvature metric, in symmetric form (i.e. S' S). 
    * Within 3D space (i.e. 3x3 matrix). 
    * 
    * @param UV_t 
    * Transpose of UV.
    * 
    * @param UV
    * 2D basis matrix (i.e. 3rd column discarded)
    * 
    * @param c
    * Weight of the metric.
    */
   protected Matrix2d compressionMetric(Matrix3d F, Matrix3d S2, Matrix2x3 UV_t, 
   Matrix3x2 UV, double c) {
      // 3x3 = F' F 
      Matrix3d G = new Matrix3d();
      G.mulTransposeLeft (F, F);    
      G.sub (Matrix3d.IDENTITY);    // F - I = linear strain
      
      // Use absolute to make refinement occur for either elastic compression or 
      // plastic stretching. Use negative to "trick" the original 
      // buckling suppression metric into promoting buckling/refinement instead.
      MathUtil.negAbs (G);
      
      // 2x2 = UV' F' F UV 
      MatrixNd e = new MatrixNd(UV_t);    
      e = MathUtil.mul (e, G);
      e = MathUtil.mul (e, UV);
      
      // 2x2 = UV' (F' F)' (F' F) UV 
      MatrixNd e2 = new MatrixNd(UV_t);
      e2 = MathUtil.mul (e2, MathUtil.transpose (G));
      e2 = MathUtil.mul (e2, G);
      e2 = MathUtil.mul (e2, UV);
      
      // 2x2 = UV' S' S UV
      MatrixNd S = new MatrixNd(UV_t);
      S = MathUtil.mul (S, S2);
      S = MathUtil.mul (S, UV);
      S = MathUtil.perp2x2 (S);
      
      // 2x2 
      MatrixNd D = new MatrixNd(e2);     // (F' F)' (F' F)
      D.scaledAdd (-4*c*c, S);           // TODO * (rib_stiffness = 1)
      
      // 2x2
      Matrix2d rv = new Matrix2d();
      rv.set (e);                      // TODO on or off?   F' F
      rv.negate ();                    // TODO on or off?   
      rv.add ( MathUtil.sqrt (D) );    // (F' F)' (F' F) + Curvature
      rv.set ( MathUtil.getPositive (rv) );
      rv.scale ( 1.0 / (2*c*c) );
      
      return rv;
   }
}

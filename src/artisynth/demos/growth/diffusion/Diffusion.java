package artisynth.demos.growth.diffusion;

import java.util.ArrayList;

import artisynth.demos.growth.util.MeshUtil;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix;
import maspack.matrix.SparseMatrixCell;
import maspack.matrix.SparseMatrixNd;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.solvers.PardisoSolver;

/**
 * Simulate diffusion (i.e. spreading) of chemicals across a mesh. 
 * 
 * Details can be found in the course notes:
 * "Laplace-Beltrami: The swiss army knife of geometry processing."
 */
public class Diffusion {
   
   /** Surface mesh that diffusion will occur on. */
   protected PolygonalMesh mMesh;
   
   /** Collection of chemicals to be diffused. */
   protected MeshChemicals mMeshChems;
   
   /** Which chemical types are to be diffused? */
   protected ArrayList<Integer> mChemTypesToDiffuse;
   
   /* --- Experimental. Should leave them false. --- */
   
   protected boolean mUseRobinCondition = false;
   protected boolean mUseSparseArea = false;
   
   /* --- Cache --- */
   
   /** Surface area of mesh. */
   protected double mSumFaceAreas;
   
   /** Area of each face. */
   protected VectorNd mFaceAreas;
   
   /** Area associated with each vertex. */
   protected VectorNd mVtxAreas;
   
   /** Laplcian matrix. */
   protected SparseMatrixNd mL;
   
   /** Mass matrix. */
   protected SparseMatrixNd mA;
   
   /* --- Linear ax=b solver --- */
   
   protected PardisoSolver mLinearSolver;
   protected final double EPSILON = 1e-12;
   
   /* --- End-User Functions --- */
   
   public Diffusion(PolygonalMesh mesh, MeshChemicals meshChems) {
      setTarget(mesh, meshChems);
      
      mFaceAreas = new VectorNd();
      mL = new SparseMatrixNd(mesh.numVertices (), mesh.numVertices ());
      
      if (mUseSparseArea)
         mA = new SparseMatrixNd(mesh.numVertices (), mesh.numVertices ());
      else 
         mVtxAreas = new VectorNd();
      
      mLinearSolver = new PardisoSolver();
      
      int numChems = mMeshChems.getNumChemTypes ();
      mChemTypesToDiffuse = new ArrayList<Integer>();
      for (int c = 0; c < numChems; c++) {
         mChemTypesToDiffuse.add (c);
      }
   }

   public Diffusion(PolygonalMesh mesh, MeshChemicals meshChems,
   int[] chemTypesToDiffuse) {
      this(mesh, meshChems);
      mChemTypesToDiffuse = new ArrayList<Integer>();
      for (int c : chemTypesToDiffuse) {
         mChemTypesToDiffuse.add (c);
      }
   }
   
   public void setTarget(PolygonalMesh mesh, MeshChemicals meshChems) {
      mMesh = mesh;
      mMeshChems = meshChems;
   }
   
   /** Simulate diffusion across a small time step. */
   public void advance(double deltaTime) {
      computeFaceAreas();
      if (! mUseSparseArea)
         computeVertexAreas();
      computeL(mUseSparseArea);
      diffuse(deltaTime);
   }
   
   /* --- Primary Functions --- */

   /** Calculate the area of each face. */
   protected void computeFaceAreas( ) {
      // Clear face area
      mFaceAreas.setSize (mMesh.numFaces ());
      mSumFaceAreas = 0;
      
      // Face area
      double[] faceAreasBuf = mFaceAreas.getBuffer ();
      for (int f = 0; f < mMesh.numFaces (); f++) {
         faceAreasBuf[f] = mMesh.getFace (f).computeArea ();
         mSumFaceAreas += faceAreasBuf[f];
      }
   }
   
   /** Calculate the area associated with each vertex. Vertex area is equal to 
    *  1/3 of the summed area of the vertex's adjacent faces. */
   protected void computeVertexAreas() {
      // Clear vertex area
      mVtxAreas.setSize (mMesh.numVertices ());
      for (int v = 0; v < mVtxAreas.size(); v++) {
         mVtxAreas.setZero ();
      }
      
      // Compute vertex area
      double[] faceAreasBuf = mFaceAreas.getBuffer ();
      double[] vtxAreasBuf = mVtxAreas.getBuffer ();
      for (int f = 0; f < mMesh.numFaces(); f++) {
         Face face = mMesh.getFace (f);
         for (Vertex3d vtx : face.getTriVertices ()) {
            int v = vtx.getIndex ();
            vtxAreasBuf[v] += faceAreasBuf[f];
         }
      }
      
      // Scale vertex area
      for (int v = 0; v < vtxAreasBuf.length; v++) {
         vtxAreasBuf[v] /= 3.0;
      }
   }

   /** Construct the Laplacian matrix. */
   protected void computeL(boolean isComputeSparseArea) {
      // https://libigl.github.io/tutorial/#laplacian
      
      // Reset mLc
      if (mL.rowSize () != mMesh.numVertices ()) {
         mL.setSize (mMesh.numVertices (), mMesh.numVertices ());
      }
      mL.setZero ();
      
      // Reset mA
      if (isComputeSparseArea) {
         if (mA.rowSize () != mMesh.numVertices ()) 
            mA.setSize (mMesh.numVertices (), mMesh.numVertices ());
         mA.setZero ();
      }
      double[] faceAreasBuf = mFaceAreas.getBuffer ();

      for (Face face : mMesh.getFaces()) {
         HalfEdge he = face.firstHalfEdge();
         do {
            
            int i = he.head.getIndex();
            int j = he.tail.getIndex();
            
            // If this (i,j) half edge has corresponding half edge, only let 
            // the half edge, where i < j, handle the edge cotan computation.
            if (he.opposite != null && i > j) {
               he = he.getNext ();
               continue;
            }

            double edgeCotan = 0.5 * getEdgeCotan(he);
            
            if (Double.isInfinite (edgeCotan)) {
               System.out.println ("Detected infinite edgeCotan.");
            }
            
            getCell(mL, i, j).value += edgeCotan;
            getCell(mL, j, i).value += edgeCotan;
            
            getCell(mL, i, i).value -= edgeCotan + EPSILON;
            getCell(mL, j, j).value -= edgeCotan + EPSILON;
           
            if (isComputeSparseArea) {
               double edgeArea = faceAreasBuf[he.getFace ().idx];
               if (he.opposite != null) edgeArea += faceAreasBuf[he.getOppositeFace ().idx];
               edgeArea /= 12;          // But 6 used for (i,i),(j,j) in lec notes
               
               getCell(mA, i, j).value += edgeArea;
               getCell(mA, j, i).value += edgeArea;
               
               getCell(mA, i, i).value += edgeArea;   
               getCell(mA, j, j).value += edgeArea;   
            }

            he = he.getNext ();
         } while (he != face.firstHalfEdge());
      }
   }
   
   
   
   
   /**
    * Perform a single time step of diffusion, for each chemical type.
    * 
    * Solves (A - t*L)u1 = A*u0
    */
   protected void diffuse(double deltaTime) {
      int numVtxs = mMesh.numVertices();
      double[] vtxAreasBuf = (! mUseSparseArea) ? mVtxAreas.getBuffer () : null;
      

      double t = deltaTime;
      mL.scale (-t);    // - t*L
      
      if (mUseSparseArea) {
         mL.add (mA);
      }
      else {
         for (int d = 0; d < numVtxs; d++) {
            getCell(mL, d, d).value += vtxAreasBuf[d];   // A - t*L 
         }
      }
      
      mLinearSolver.analyze (mL, mL.rowSize(), Matrix.SPD);
      mLinearSolver.factor ();
      
      // For each chemical type
      for (int chemType : mChemTypesToDiffuse) {
         // Initial chemical amounts before diffusion.
         VectorNd u0 = new VectorNd(numVtxs);
         for (int v = 0; v < numVtxs; v++) {
            double u0_v = mMeshChems.getVtxChem (v, chemType);
            if (! mUseSparseArea) u0_v *= vtxAreasBuf[v];
            
            u0.set (v, u0_v);   // A*u0
         }
         
         if (mUseSparseArea)
            u0.mul (mA, u0);
         
         VectorNd u1 = new VectorNd(numVtxs);
         
         // Solve for u1
         mLinearSolver.solve (u1, u0);
         
         if (mUseRobinCondition) {
            VectorNd u1_dirichlet = new VectorNd(numVtxs);
            
            for (Vertex3d vtx : mMesh.getVertices ()) {
               if (MeshUtil.isBoundaryVtx (vtx)) {
                  int v = vtx.getIndex();
                  // Modify A matrix
                  for (int c = 0; c < mMesh.numVertices (); c++) {
                     if (v == c)
                        mL.set (v, c, 1);
                     else
                        mL.set (v, c, 0);
                  }
                  
                  // Modify u0 
                  u0.set (v, 0);
               }
            }
            
            mLinearSolver.analyze (mL, mL.rowSize(), Matrix.SPD);
            mLinearSolver.factor ();
            mLinearSolver.solve(u1_dirichlet, u0);
            
            u1.add (u1_dirichlet);
            u1.scale (0.5);
         }
         
         // Update the chemical amounts with the diffused u1
         for (int v = 0; v < numVtxs; v++) {
            double conc = u1.get (v);
            
            if (conc < 0) {
               System.out.printf ("diffuse(): Negative concentration detected "+
                 "at node %d. Forcing conc 0.0.\n", v);
               
               conc = 0;
            }
            
            mMeshChems.setVtxChem (v, chemType, conc);
         }
      }
   }
   
   /* --- Secondary Functions --- */
   
   /** Calculate cot(alpha) + cot(beta) for a given edge, where alpha and beta
    *  are the angles that are opposite of the edge. */
   protected double getEdgeCotan(HalfEdge edge) {
      double cotAlpha = 0;
      double cotBeta = 0;
      
      // Compute cot_alpha
      
      Vector3d vi = edge.head.pnt;
      Vector3d vj = edge.tail.pnt;
      
      Vector3d va = edge.getNext ().head.pnt;   // Vertex 'a', opposite of edge
      
      Vector3d via = new Vector3d().sub(vi,va);
      Vector3d vja = new Vector3d().sub(vj,va);
      
      cotAlpha = via.dot(vja) / new Vector3d().cross(via, vja).norm();
      
      // Compute cot_beta
      
      if (edge.opposite != null) {
         Vector3d vb = edge.opposite.getNext ().head.pnt;
         
         Vector3d vib = new Vector3d().sub(vi,vb);
         Vector3d vjb = new Vector3d().sub(vj,vb);
         
         cotBeta = vib.dot(vjb) / new Vector3d().cross(vib,vjb).norm();
      }

      return (cotAlpha + cotBeta);
   }
   
   /* --- Util --- */
   
   protected int getOppVtxIdx(Vertex3d curVtx, HalfEdge edge) {
      if (edge.head.getIndex () == curVtx.getIndex ()) {
         return edge.tail.getIndex ();
      }
      else {
         return edge.head.getIndex ();
      }
   }
   
   protected SparseMatrixCell getCell(SparseMatrixNd mtx, int i, int j) {
      SparseMatrixCell cell = mtx.getCell (i, j);
      if (cell == null) {
         mtx.setZero(i, j);
         cell = mtx.getCell (i, j);
         if (cell == null) {
            System.out.println ("ERROR");
         }
      }
      return cell;
   }
   
}

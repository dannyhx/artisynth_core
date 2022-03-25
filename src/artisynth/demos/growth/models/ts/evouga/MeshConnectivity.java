package artisynth.demos.growth.models.ts.evouga;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Matrix2d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

public class MeshConnectivity {
   
   protected PolygonalMesh mesh;
   
   /** F[f][local_v] = v */
   public int[][] F;
   
   /** FE[f][local_e] = e */
   public int[][] FE;
   
   /** FEorient[f][local_e] = 0 if local_e half-edge belongs to f. 
    * Otherwise, 1 if half-edge belongs to opposite f. */
   public int[][] FEorient;
   
   /** EV[e][0,1] = vertex of e */
   public int[][] EV;
   
   /** EF[e][0,1] = face of e */
   public int[][] EF;
   
   /** EOpp[f][0,1] = vertex that's opposite of e */ 
   public int[][] EOpp;
   
   public final Vector2d d = new Vector2d(0,1);
   public Matrix2d[] T;
   
   public MeshConnectivity(PolygonalMesh mesh) {
      this.mesh = mesh;
      this.refresh ();
   }
   
   public void refresh() {
      int nfaces = mesh.numFaces ();
      
      F = new int[nfaces][3];
      T = new Matrix2d[nfaces];
      for (int f = 0; f < nfaces; f++) {
         Face face = mesh.getFace (f);
         F[f] = face.getVertexIndices ();
         
         T[f] = new Matrix2d();
      }
      
      // edgeFaces[edgeStr][0,1 halfEdge] = f
      LinkedHashMap<String, int[]> edgeFaces = new LinkedHashMap<String, int[]>();
      
      for (int f = 0; f < nfaces; f++) {
         for (int j = 0; j < 3; j++) {
            int v0 = F[f][(j+1) % 3];
            int v1 = F[f][(j+2) % 3];
            
            int idx = 0;
            
            if (v0 > v1) {
               int v0_tmp = v0;
               v0 = v1;
               v1 = v0_tmp;
               
               idx = 1; // Other half-edge
            }
            
            String p = this.edgeStr (v0, v1);
            
            if (edgeFaces.get (p) == null) {
               int[] hef = new int[2];
               hef[idx] = f;
               hef[1-idx] = -1;  // To be populated when iterate to other half-edge, if exists.
               edgeFaces.put (p, hef);
            } else {
               edgeFaces.get (p)[idx] = f;
            }
         }
      }
      
      int nedges = edgeFaces.size ();
      
      FE = new int[nfaces][3];
      FEorient = new int[nfaces][3];
      EV = new int[nedges][2];
      EF = new int[nedges][2];
      EOpp = new int[nedges][2];
      
      // Indices assigned to each full-edge.
      HashMap<String, Integer> edgeIndices = new HashMap<String, Integer>();
      
      int idx = 0;
      for (Map.Entry<String, int[]> entry : edgeFaces.entrySet()) {
         String edgeStr = entry.getKey ();
         int[] hef = entry.getValue ();  // Faces adjacent to the edge.
         
         int[] edgeVtxPair = this.edgeStrToIndices (edgeStr);
         
         edgeIndices.put (edgeStr, idx);
         EV[idx][0] = edgeVtxPair[0];
         EV[idx][1] = edgeVtxPair[1];
         EF[idx][0] = hef[0];   // Face index.
         EF[idx][1] = hef[1];   // Opposite face index.
         idx++;
      }
      
      for (int f = 0; f < nfaces; f++) {
         for (int j = 0; j < 3; j++) {
            int v0 = F[f][(j+1)%3];
            int v1 = F[f][(j+2)%3];
            
            if (v0 > v1) {
               int v0_tmp = v0;
               v0 = v1;
               v1 = v0_tmp;
            }
            
            String edgeStr = this.edgeStr (v0, v1);
            FE[f][j] = edgeIndices.get (edgeStr);
         }
      }
      
      for (int e = 0; e < nedges; e++) {
         for (int j = 0; j < 2; j++) {
            EOpp[e][j] = oppositeVertex(e, j);
         }
      }
      
      for (int f = 0; f < nfaces; f++) {
         for (int j = 0; j < 3; j++) {
            // Full-edge index.
            int edge = FE[f][j];
            
            if (EF[edge][0] == f) {
               FEorient[f][j] = 0;
            } else {
               FEorient[f][j] = 1;
            }
         }
      }
      
   }
   
   /////////////////////
   
   public int vertexOppositeFaceEdge(int f, int vertidx) {
      int edge = FE[f][vertidx];
      int edgeorient = FEorient[f][vertidx];
      return EOpp[edge][1 - edgeorient];
   }
   
   
   /////////////////////
   
   protected int oppositeVertexIndex(int edge, int faceIdx) {
      int f = EF[edge][faceIdx];
      if (f == -1) {
         return -1;
      }
      
      // For vertex of the face.
      for (int j = 0; j < 3; j++) {
         // Select the vertex that isn't part of the edge.
         if (F[f][j] != EV[edge][0] && F[f][j] != EV[edge][1]) {
            return j;
         }
      }
      
      // unreachable.
      return -1;
   }
   
   protected int oppositeVertex(int edge, int faceIdx) {
      int f = EF[edge][faceIdx];
      int local_oppVtxIdx = oppositeVertexIndex(edge, faceIdx);  
      if (local_oppVtxIdx == -1) {
         return -1;
      }
      return F[f][local_oppVtxIdx];  // To global vertex index.
   }
   
   /////////////////////
   
   protected String edgeStr(int a, int b) {
      return String.format ("%d-%d", a, b);
   }
   
   protected int[] edgeStrToIndices(String s) {
      String[] tokens = s.split ("-");
      
      return new int[] {
         Integer.parseInt (tokens[0]), 
         Integer.parseInt (tokens[1]),
      };
   }
   
   ////////////////////////
   
   public int numEdges() {
      return EV.length;
   }
}

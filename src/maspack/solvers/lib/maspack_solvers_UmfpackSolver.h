/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class maspack_solvers_UmfpackSolver */

#ifndef _Included_maspack_solvers_UmfpackSolver
#define _Included_maspack_solvers_UmfpackSolver
#ifdef __cplusplus
extern "C" {
#endif
#undef maspack_solvers_UmfpackSolver_INIT_UNKNOWN
#define maspack_solvers_UmfpackSolver_INIT_UNKNOWN 0L
#undef maspack_solvers_UmfpackSolver_INIT_OK
#define maspack_solvers_UmfpackSolver_INIT_OK 2L
#undef maspack_solvers_UmfpackSolver_ERR_CANT_LOAD_LIBRARIES
#define maspack_solvers_UmfpackSolver_ERR_CANT_LOAD_LIBRARIES -1000L
/*
 * Class:     maspack_solvers_UmfpackSolver
 * Method:    umfpack_di_symbolic
 * Signature: (II[I[I[D[J[D[D)I
 */
JNIEXPORT jint JNICALL Java_maspack_solvers_UmfpackSolver_umfpack_1di_1symbolic
  (JNIEnv *, jobject, jint, jint, jintArray, jintArray, jdoubleArray, jlongArray, jdoubleArray, jdoubleArray);

/*
 * Class:     maspack_solvers_UmfpackSolver
 * Method:    umfpack_di_numeric
 * Signature: ([I[I[D[J[J[D[D)I
 */
JNIEXPORT jint JNICALL Java_maspack_solvers_UmfpackSolver_umfpack_1di_1numeric
  (JNIEnv *, jobject, jintArray, jintArray, jdoubleArray, jlongArray, jlongArray, jdoubleArray, jdoubleArray);

/*
 * Class:     maspack_solvers_UmfpackSolver
 * Method:    umfpack_di_solve
 * Signature: (I[I[I[D[D[D[J[D[D)I
 */
JNIEXPORT jint JNICALL Java_maspack_solvers_UmfpackSolver_umfpack_1di_1solve
  (JNIEnv *, jobject, jint, jintArray, jintArray, jdoubleArray, jdoubleArray, jdoubleArray, jlongArray, jdoubleArray, jdoubleArray);

/*
 * Class:     maspack_solvers_UmfpackSolver
 * Method:    umfpack_di_free_symbolic
 * Signature: ([J)V
 */
JNIEXPORT void JNICALL Java_maspack_solvers_UmfpackSolver_umfpack_1di_1free_1symbolic
  (JNIEnv *, jobject, jlongArray);

/*
 * Class:     maspack_solvers_UmfpackSolver
 * Method:    umfpack_di_free_numeric
 * Signature: ([J)V
 */
JNIEXPORT void JNICALL Java_maspack_solvers_UmfpackSolver_umfpack_1di_1free_1numeric
  (JNIEnv *, jobject, jlongArray);

/*
 * Class:     maspack_solvers_UmfpackSolver
 * Method:    umfpack_di_defaults
 * Signature: ([D)V
 */
JNIEXPORT void JNICALL Java_maspack_solvers_UmfpackSolver_umfpack_1di_1defaults
  (JNIEnv *, jobject, jdoubleArray);

#ifdef __cplusplus
}
#endif
#endif

#-------------------------------------------------
#
# Project created by QtCreator 2018-08-25T00:22:41
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = SlamTestBed
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    glwidget.cpp \
    logo.cpp \
    window.cpp \
    tetrahedron.cpp \
    winslam.cpp \
    dialogscalatraslarota.cpp \
    Point3D.cpp \
    transformador2/Transformador.cpp \
    GeneratorPCA/GeneratorPCA.cpp \
    ModuloEscala/FindScala.cpp \
    AjusteTiempo/AjusteTiempo.cpp \
    Interpolator/Interpolator.cpp \
    datadialogscalatraslarota.cpp \
    dialogshowestimated.cpp \
    datadialogshowestimated.cpp \
    dialogmessage.cpp \
    Properties/ReaderProperties.cpp \
    Registrador/Registrador.cpp \
    Statistics/Statistics.cpp \
    dialogparameters.cpp \
    datadialogparameters.cpp \
    configuration.cpp

HEADERS += \
        mainwindow.h \
    glwidget.h \
    logo.h \
    window.h \
    winslam.h \
    tetrahedron.h \
    dialogscalatraslarota.h \
    transformador2\transformador.h \
    transformador2\Point3D.h \
    Point3D.h \
    transformador2/Transformador.h \
    Registrador/Registrador.h \
    GeneratorPCA/GeneratorPCA.h \
    ModuloEscala/FindScala.h \
    AjusteTiempo/AjusteTiempo.h \
    Interpolator/Interpolator.h \
    datadialogscalatraslarota.h \
    dialogshowestimated.h \
    datadialogshowestimated.h \
    Statistics/Statistics.h \
    dialogmessage.h \
    Properties/ReaderProperties.h \
    dialogparameters.h \
    datadialogparameters.h \
    configuration.h \
    RegistradorRansac/Eigen/src/Cholesky/LDLT.h \
    RegistradorRansac/Eigen/src/Cholesky/LLT.h \
    RegistradorRansac/Eigen/src/Cholesky/LLT_LAPACKE.h \
    RegistradorRansac/Eigen/src/CholmodSupport/CholmodSupport.h \
    RegistradorRansac/Eigen/src/Core/arch/AltiVec/Complex.h \
    RegistradorRansac/Eigen/src/Core/arch/AltiVec/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/arch/AltiVec/PacketMath.h \
    RegistradorRansac/Eigen/src/Core/arch/AVX/Complex.h \
    RegistradorRansac/Eigen/src/Core/arch/AVX/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/arch/AVX/PacketMath.h \
    RegistradorRansac/Eigen/src/Core/arch/AVX/TypeCasting.h \
    RegistradorRansac/Eigen/src/Core/arch/AVX512/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/arch/AVX512/PacketMath.h \
    RegistradorRansac/Eigen/src/Core/arch/CUDA/Complex.h \
    RegistradorRansac/Eigen/src/Core/arch/CUDA/Half.h \
    RegistradorRansac/Eigen/src/Core/arch/CUDA/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/arch/CUDA/PacketMath.h \
    RegistradorRansac/Eigen/src/Core/arch/CUDA/PacketMathHalf.h \
    RegistradorRansac/Eigen/src/Core/arch/CUDA/TypeCasting.h \
    RegistradorRansac/Eigen/src/Core/arch/Default/Settings.h \
    RegistradorRansac/Eigen/src/Core/arch/NEON/Complex.h \
    RegistradorRansac/Eigen/src/Core/arch/NEON/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/arch/NEON/PacketMath.h \
    RegistradorRansac/Eigen/src/Core/arch/SSE/Complex.h \
    RegistradorRansac/Eigen/src/Core/arch/SSE/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/arch/SSE/PacketMath.h \
    RegistradorRansac/Eigen/src/Core/arch/SSE/TypeCasting.h \
    RegistradorRansac/Eigen/src/Core/arch/ZVector/Complex.h \
    RegistradorRansac/Eigen/src/Core/arch/ZVector/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/arch/ZVector/PacketMath.h \
    RegistradorRansac/Eigen/src/Core/functors/AssignmentFunctors.h \
    RegistradorRansac/Eigen/src/Core/functors/BinaryFunctors.h \
    RegistradorRansac/Eigen/src/Core/functors/NullaryFunctors.h \
    RegistradorRansac/Eigen/src/Core/functors/StlFunctors.h \
    RegistradorRansac/Eigen/src/Core/functors/TernaryFunctors.h \
    RegistradorRansac/Eigen/src/Core/functors/UnaryFunctors.h \
    RegistradorRansac/Eigen/src/Core/products/GeneralBlockPanelKernel.h \
    RegistradorRansac/Eigen/src/Core/products/GeneralMatrixMatrix.h \
    RegistradorRansac/Eigen/src/Core/products/GeneralMatrixMatrix_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
    RegistradorRansac/Eigen/src/Core/products/GeneralMatrixMatrixTriangular_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/GeneralMatrixVector.h \
    RegistradorRansac/Eigen/src/Core/products/GeneralMatrixVector_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/Parallelizer.h \
    RegistradorRansac/Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
    RegistradorRansac/Eigen/src/Core/products/SelfadjointMatrixMatrix_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/SelfadjointMatrixVector.h \
    RegistradorRansac/Eigen/src/Core/products/SelfadjointMatrixVector_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/SelfadjointProduct.h \
    RegistradorRansac/Eigen/src/Core/products/SelfadjointRank2Update.h \
    RegistradorRansac/Eigen/src/Core/products/TriangularMatrixMatrix.h \
    RegistradorRansac/Eigen/src/Core/products/TriangularMatrixMatrix_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/TriangularMatrixVector.h \
    RegistradorRansac/Eigen/src/Core/products/TriangularMatrixVector_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/TriangularSolverMatrix.h \
    RegistradorRansac/Eigen/src/Core/products/TriangularSolverMatrix_BLAS.h \
    RegistradorRansac/Eigen/src/Core/products/TriangularSolverVector.h \
    RegistradorRansac/Eigen/src/Core/util/BlasUtil.h \
    RegistradorRansac/Eigen/src/Core/util/Constants.h \
    RegistradorRansac/Eigen/src/Core/util/DisableStupidWarnings.h \
    RegistradorRansac/Eigen/src/Core/util/ForwardDeclarations.h \
    RegistradorRansac/Eigen/src/Core/util/Macros.h \
    RegistradorRansac/Eigen/src/Core/util/Memory.h \
    RegistradorRansac/Eigen/src/Core/util/Meta.h \
    RegistradorRansac/Eigen/src/Core/util/MKL_support.h \
    RegistradorRansac/Eigen/src/Core/util/NonMPL2.h \
    RegistradorRansac/Eigen/src/Core/util/ReenableStupidWarnings.h \
    RegistradorRansac/Eigen/src/Core/util/StaticAssert.h \
    RegistradorRansac/Eigen/src/Core/util/XprHelper.h \
    RegistradorRansac/Eigen/src/Core/Array.h \
    RegistradorRansac/Eigen/src/Core/ArrayBase.h \
    RegistradorRansac/Eigen/src/Core/ArrayWrapper.h \
    RegistradorRansac/Eigen/src/Core/Assign.h \
    RegistradorRansac/Eigen/src/Core/Assign_MKL.h \
    RegistradorRansac/Eigen/src/Core/AssignEvaluator.h \
    RegistradorRansac/Eigen/src/Core/BandMatrix.h \
    RegistradorRansac/Eigen/src/Core/Block.h \
    RegistradorRansac/Eigen/src/Core/BooleanRedux.h \
    RegistradorRansac/Eigen/src/Core/CommaInitializer.h \
    RegistradorRansac/Eigen/src/Core/ConditionEstimator.h \
    RegistradorRansac/Eigen/src/Core/CoreEvaluators.h \
    RegistradorRansac/Eigen/src/Core/CoreIterators.h \
    RegistradorRansac/Eigen/src/Core/CwiseBinaryOp.h \
    RegistradorRansac/Eigen/src/Core/CwiseNullaryOp.h \
    RegistradorRansac/Eigen/src/Core/CwiseTernaryOp.h \
    RegistradorRansac/Eigen/src/Core/CwiseUnaryOp.h \
    RegistradorRansac/Eigen/src/Core/CwiseUnaryView.h \
    RegistradorRansac/Eigen/src/Core/DenseBase.h \
    RegistradorRansac/Eigen/src/Core/DenseCoeffsBase.h \
    RegistradorRansac/Eigen/src/Core/DenseStorage.h \
    RegistradorRansac/Eigen/src/Core/Diagonal.h \
    RegistradorRansac/Eigen/src/Core/DiagonalMatrix.h \
    RegistradorRansac/Eigen/src/Core/DiagonalProduct.h \
    RegistradorRansac/Eigen/src/Core/Dot.h \
    RegistradorRansac/Eigen/src/Core/EigenBase.h \
    RegistradorRansac/Eigen/src/Core/ForceAlignedAccess.h \
    RegistradorRansac/Eigen/src/Core/Fuzzy.h \
    RegistradorRansac/Eigen/src/Core/GeneralProduct.h \
    RegistradorRansac/Eigen/src/Core/GenericPacketMath.h \
    RegistradorRansac/Eigen/src/Core/GlobalFunctions.h \
    RegistradorRansac/Eigen/src/Core/Inverse.h \
    RegistradorRansac/Eigen/src/Core/IO.h \
    RegistradorRansac/Eigen/src/Core/Map.h \
    RegistradorRansac/Eigen/src/Core/MapBase.h \
    RegistradorRansac/Eigen/src/Core/MathFunctions.h \
    RegistradorRansac/Eigen/src/Core/MathFunctionsImpl.h \
    RegistradorRansac/Eigen/src/Core/Matrix.h \
    RegistradorRansac/Eigen/src/Core/MatrixBase.h \
    RegistradorRansac/Eigen/src/Core/NestByValue.h \
    RegistradorRansac/Eigen/src/Core/NoAlias.h \
    RegistradorRansac/Eigen/src/Core/NumTraits.h \
    RegistradorRansac/Eigen/src/Core/PermutationMatrix.h \
    RegistradorRansac/Eigen/src/Core/PlainObjectBase.h \
    RegistradorRansac/Eigen/src/Core/Product.h \
    RegistradorRansac/Eigen/src/Core/ProductEvaluators.h \
    RegistradorRansac/Eigen/src/Core/Random.h \
    RegistradorRansac/Eigen/src/Core/Redux.h \
    RegistradorRansac/Eigen/src/Core/Ref.h \
    RegistradorRansac/Eigen/src/Core/Replicate.h \
    RegistradorRansac/Eigen/src/Core/ReturnByValue.h \
    RegistradorRansac/Eigen/src/Core/Reverse.h \
    RegistradorRansac/Eigen/src/Core/Select.h \
    RegistradorRansac/Eigen/src/Core/SelfAdjointView.h \
    RegistradorRansac/Eigen/src/Core/SelfCwiseBinaryOp.h \
    RegistradorRansac/Eigen/src/Core/Solve.h \
    RegistradorRansac/Eigen/src/Core/SolverBase.h \
    RegistradorRansac/Eigen/src/Core/SolveTriangular.h \
    RegistradorRansac/Eigen/src/Core/StableNorm.h \
    RegistradorRansac/Eigen/src/Core/Stride.h \
    RegistradorRansac/Eigen/src/Core/Swap.h \
    RegistradorRansac/Eigen/src/Core/Transpose.h \
    RegistradorRansac/Eigen/src/Core/Transpositions.h \
    RegistradorRansac/Eigen/src/Core/TriangularMatrix.h \
    RegistradorRansac/Eigen/src/Core/VectorBlock.h \
    RegistradorRansac/Eigen/src/Core/VectorwiseOp.h \
    RegistradorRansac/Eigen/src/Core/Visitor.h \
    RegistradorRansac/Eigen/src/Eigenvalues/ComplexEigenSolver.h \
    RegistradorRansac/Eigen/src/Eigenvalues/ComplexSchur.h \
    RegistradorRansac/Eigen/src/Eigenvalues/ComplexSchur_LAPACKE.h \
    RegistradorRansac/Eigen/src/Eigenvalues/EigenSolver.h \
    RegistradorRansac/Eigen/src/Eigenvalues/GeneralizedEigenSolver.h \
    RegistradorRansac/Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
    RegistradorRansac/Eigen/src/Eigenvalues/HessenbergDecomposition.h \
    RegistradorRansac/Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h \
    RegistradorRansac/Eigen/src/Eigenvalues/RealQZ.h \
    RegistradorRansac/Eigen/src/Eigenvalues/RealSchur.h \
    RegistradorRansac/Eigen/src/Eigenvalues/RealSchur_LAPACKE.h \
    RegistradorRansac/Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
    RegistradorRansac/Eigen/src/Eigenvalues/SelfAdjointEigenSolver_LAPACKE.h \
    RegistradorRansac/Eigen/src/Eigenvalues/Tridiagonalization.h \
    RegistradorRansac/Eigen/src/Geometry/arch/Geometry_SSE.h \
    RegistradorRansac/Eigen/src/Geometry/AlignedBox.h \
    RegistradorRansac/Eigen/src/Geometry/AngleAxis.h \
    RegistradorRansac/Eigen/src/Geometry/EulerAngles.h \
    RegistradorRansac/Eigen/src/Geometry/Homogeneous.h \
    RegistradorRansac/Eigen/src/Geometry/Hyperplane.h \
    RegistradorRansac/Eigen/src/Geometry/OrthoMethods.h \
    RegistradorRansac/Eigen/src/Geometry/ParametrizedLine.h \
    RegistradorRansac/Eigen/src/Geometry/Quaternion.h \
    RegistradorRansac/Eigen/src/Geometry/Rotation2D.h \
    RegistradorRansac/Eigen/src/Geometry/RotationBase.h \
    RegistradorRansac/Eigen/src/Geometry/Scaling.h \
    RegistradorRansac/Eigen/src/Geometry/Transform.h \
    RegistradorRansac/Eigen/src/Geometry/Translation.h \
    RegistradorRansac/Eigen/src/Geometry/Umeyama.h \
    RegistradorRansac/Eigen/src/Householder/BlockHouseholder.h \
    RegistradorRansac/Eigen/src/Householder/Householder.h \
    RegistradorRansac/Eigen/src/Householder/HouseholderSequence.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/BasicPreconditioners.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/BiCGSTAB.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/ConjugateGradient.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/IncompleteCholesky.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/IncompleteLUT.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/IterativeSolverBase.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/LeastSquareConjugateGradient.h \
    RegistradorRansac/Eigen/src/IterativeLinearSolvers/SolveWithGuess.h \
    RegistradorRansac/Eigen/src/Jacobi/Jacobi.h \
    RegistradorRansac/Eigen/src/LU/arch/Inverse_SSE.h \
    RegistradorRansac/Eigen/src/LU/Determinant.h \
    RegistradorRansac/Eigen/src/LU/FullPivLU.h \
    RegistradorRansac/Eigen/src/LU/InverseImpl.h \
    RegistradorRansac/Eigen/src/LU/PartialPivLU.h \
    RegistradorRansac/Eigen/src/LU/PartialPivLU_LAPACKE.h \
    RegistradorRansac/Eigen/src/MetisSupport/MetisSupport.h \
    RegistradorRansac/Eigen/src/misc/blas.h \
    RegistradorRansac/Eigen/src/misc/Image.h \
    RegistradorRansac/Eigen/src/misc/Kernel.h \
    RegistradorRansac/Eigen/src/misc/lapack.h \
    RegistradorRansac/Eigen/src/misc/lapacke.h \
    RegistradorRansac/Eigen/src/misc/lapacke_mangling.h \
    RegistradorRansac/Eigen/src/misc/RealSvd2x2.h \
    RegistradorRansac/Eigen/src/OrderingMethods/Amd.h \
    RegistradorRansac/Eigen/src/OrderingMethods/Eigen_Colamd.h \
    RegistradorRansac/Eigen/src/OrderingMethods/Ordering.h \
    RegistradorRansac/Eigen/src/PardisoSupport/PardisoSupport.h \
    RegistradorRansac/Eigen/src/PaStiXSupport/PaStiXSupport.h \
    RegistradorRansac/Eigen/src/plugins/ArrayCwiseBinaryOps.h \
    RegistradorRansac/Eigen/src/plugins/ArrayCwiseUnaryOps.h \
    RegistradorRansac/Eigen/src/plugins/BlockMethods.h \
    RegistradorRansac/Eigen/src/plugins/CommonCwiseBinaryOps.h \
    RegistradorRansac/Eigen/src/plugins/CommonCwiseUnaryOps.h \
    RegistradorRansac/Eigen/src/plugins/MatrixCwiseBinaryOps.h \
    RegistradorRansac/Eigen/src/plugins/MatrixCwiseUnaryOps.h \
    RegistradorRansac/Eigen/src/QR/ColPivHouseholderQR.h \
    RegistradorRansac/Eigen/src/QR/ColPivHouseholderQR_LAPACKE.h \
    RegistradorRansac/Eigen/src/QR/CompleteOrthogonalDecomposition.h \
    RegistradorRansac/Eigen/src/QR/FullPivHouseholderQR.h \
    RegistradorRansac/Eigen/src/QR/HouseholderQR.h \
    RegistradorRansac/Eigen/src/QR/HouseholderQR_LAPACKE.h \
    RegistradorRansac/Eigen/src/SparseCholesky/SimplicialCholesky.h \
    RegistradorRansac/Eigen/src/SparseCholesky/SimplicialCholesky_impl.h \
    RegistradorRansac/Eigen/src/SparseCore/AmbiVector.h \
    RegistradorRansac/Eigen/src/SparseCore/CompressedStorage.h \
    RegistradorRansac/Eigen/src/SparseCore/ConservativeSparseSparseProduct.h \
    RegistradorRansac/Eigen/src/SparseCore/MappedSparseMatrix.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseAssign.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseBlock.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseColEtree.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseCompressedBase.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseCwiseBinaryOp.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseCwiseUnaryOp.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseDenseProduct.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseDiagonalProduct.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseDot.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseFuzzy.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseMap.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseMatrix.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseMatrixBase.h \
    RegistradorRansac/Eigen/src/SparseCore/SparsePermutation.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseProduct.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseRedux.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseRef.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseSelfAdjointView.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseSolverBase.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseSparseProductWithPruning.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseTranspose.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseTriangularView.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseUtil.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseVector.h \
    RegistradorRansac/Eigen/src/SparseCore/SparseView.h \
    RegistradorRansac/Eigen/src/SparseCore/TriangularSolver.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_column_bmod.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_column_dfs.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_copy_to_ucol.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_gemm_kernel.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_heap_relax_snode.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_kernel_bmod.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_Memory.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_panel_bmod.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_panel_dfs.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_pivotL.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_pruneL.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_relax_snode.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_Structs.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_SupernodalMatrix.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLU_Utils.h \
    RegistradorRansac/Eigen/src/SparseLU/SparseLUImpl.h \
    RegistradorRansac/Eigen/src/SparseQR/SparseQR.h \
    RegistradorRansac/Eigen/src/SPQRSupport/SuiteSparseQRSupport.h \
    RegistradorRansac/Eigen/src/StlSupport/details.h \
    RegistradorRansac/Eigen/src/StlSupport/StdDeque.h \
    RegistradorRansac/Eigen/src/StlSupport/StdList.h \
    RegistradorRansac/Eigen/src/StlSupport/StdVector.h \
    RegistradorRansac/Eigen/src/SuperLUSupport/SuperLUSupport.h \
    RegistradorRansac/Eigen/src/SVD/BDCSVD.h \
    RegistradorRansac/Eigen/src/SVD/JacobiSVD.h \
    RegistradorRansac/Eigen/src/SVD/JacobiSVD_LAPACKE.h \
    RegistradorRansac/Eigen/src/SVD/SVDBase.h \
    RegistradorRansac/Eigen/src/SVD/UpperBidiagonalization.h \
    RegistradorRansac/Eigen/src/UmfPackSupport/UmfPackSupport.h

FORMS += \
        mainwindow.ui
INCLUDEPATH += ..\transformador2


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

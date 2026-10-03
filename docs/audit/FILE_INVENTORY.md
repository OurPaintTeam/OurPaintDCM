# Инвентаризация файлов OurPaintDCM

Снимок отслеживаемых файлов на 1 октября 2026 года. Созданные файлы аудита и build/ исключены. Статус использования относится к текущему репозиторию; внешние потребители публичного API не обследованы.

| Файл | Строк | Роль |
|---|---:|---|
| [.clang-format](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/.clang-format) | 29 | Конфигурация / документация |
| [.github/workflows/doc.yml](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/.github/workflows/doc.yml) | 31 | Конфигурация / документация |
| [.github/workflows/workflow.yml](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/.github/workflows/workflow.yml) | 51 | Конфигурация / документация |
| [.gitmodules](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/.gitmodules) | 3 | Конфигурация / документация |
| [CMakeLists.txt](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/CMakeLists.txt) | 59 | Конфигурация / документация |
| [Doxyfile](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/Doxyfile) | 2987 | Конфигурация / документация |
| [LICENSE](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/LICENSE) | 21 | Конфигурация / документация |
| [README.md](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/README.md) | 278 | Конфигурация / документация |
| [app/CMakeLists.txt](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/app/CMakeLists.txt) | 21 | Отдельное консольное приложение |
| [app/main.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/app/main.cpp) | 393 | Отдельное консольное приложение |
| [build.ps1](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/build.ps1) | 25 | Конфигурация / документация |
| [headers/DCMManager.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/DCMManager.h) | 436 | Библиотека DCM |
| [headers/figures/Arc.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/Arc.h) | 41 | Библиотека DCM |
| [headers/figures/Circle.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/Circle.h) | 54 | Библиотека DCM |
| [headers/figures/GeometryDependencyIndex.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/GeometryDependencyIndex.h) | 83 | Библиотека DCM |
| [headers/figures/GeometryGraphBuilder.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/GeometryGraphBuilder.h) | 39 | Библиотека DCM |
| [headers/figures/GeometryStorage.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/GeometryStorage.h) | 432 | Библиотека DCM |
| [headers/figures/Line.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/Line.h) | 49 | Библиотека DCM |
| [headers/figures/Point2D.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/Point2D.h) | 45 | Библиотека DCM |
| [headers/figures/PointBase.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/figures/PointBase.h) | 59 | Библиотека DCM |
| [headers/functions/RequirementFunction.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/functions/RequirementFunction.h) | 256 | Библиотека DCM |
| [headers/functions/RequirementFunctionFactory.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/functions/RequirementFunctionFactory.h) | 114 | Фабрика; вызовы только в тестах |
| [headers/requirements/Components.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/requirements/Components.h) | 85 | Нет ссылок; не компилируется отдельно |
| [headers/requirements/Requirements.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/requirements/Requirements.h) | 467 | Старый API; вызывается тестами, основной solve() обходит его |
| [headers/system/RequirementFunctionSystem.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/system/RequirementFunctionSystem.h) | 88 | Библиотека DCM |
| [headers/system/RequirementSystem.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/system/RequirementSystem.h) | 203 | Библиотека DCM |
| [headers/utils/Enums.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/utils/Enums.h) | 98 | Библиотека DCM |
| [headers/utils/FigureDescriptor.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/utils/FigureDescriptor.h) | 338 | Библиотека DCM |
| [headers/utils/ID.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/utils/ID.h) | 150 | Библиотека DCM |
| [headers/utils/IDGenerator.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/utils/IDGenerator.h) | 81 | Библиотека DCM |
| [headers/utils/RequirementDescriptor.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/headers/utils/RequirementDescriptor.h) | 191 | Библиотека DCM |
| [scripts/run_valgrind_all.sh](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/scripts/run_valgrind_all.sh) | 36 | Конфигурация / документация |
| [src/DCMManager.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/DCMManager.cpp) | 1764 | Библиотека DCM |
| [src/figures/GeometryDependencyIndex.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/figures/GeometryDependencyIndex.cpp) | 84 | Библиотека DCM |
| [src/figures/GeometryGraphBuilder.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/figures/GeometryGraphBuilder.cpp) | 78 | Библиотека DCM |
| [src/figures/GeometryStorage.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/figures/GeometryStorage.cpp) | 523 | Библиотека DCM |
| [src/functions/RequirementFunction.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/functions/RequirementFunction.cpp) | 818 | Библиотека DCM |
| [src/functions/RequirementFunctionFactory.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/functions/RequirementFunctionFactory.cpp) | 189 | Фабрика; вызовы только в тестах |
| [src/requirements/Requirements.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/requirements/Requirements.cpp) | 291 | Старый API; вызывается тестами, основной solve() обходит его |
| [src/system/RequirementFunctionSystem.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/system/RequirementFunctionSystem.cpp) | 98 | Библиотека DCM |
| [src/system/RequirementSystem.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/src/system/RequirementSystem.cpp) | 531 | Библиотека DCM |
| [tests/CMakeLists.txt](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/CMakeLists.txt) | 19 | Тест DCM |
| [tests/DCMManagerGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/DCMManagerGTEST.cpp) | 654 | Тест DCM |
| [tests/DCMManagerSnapshotGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/DCMManagerSnapshotGTEST.cpp) | 94 | Тест DCM |
| [tests/DCMManagerSolveGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/DCMManagerSolveGTEST.cpp) | 444 | Тест DCM |
| [tests/DragOptimizersBenchmarkGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/DragOptimizersBenchmarkGTEST.cpp) | 448 | Исключён из сборки; отдельной цели нет |
| [tests/Figures/ArcGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Figures/ArcGTEST.cpp) | 45 | Тест DCM |
| [tests/Figures/CircleGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Figures/CircleGTEST.cpp) | 51 | Тест DCM |
| [tests/Figures/GeometryStorageBenchmark.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Figures/GeometryStorageBenchmark.cpp) | 142 | Тест DCM |
| [tests/Figures/GeometryStorageGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Figures/GeometryStorageGTEST.cpp) | 321 | Тест DCM |
| [tests/Figures/LineGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Figures/LineGTEST.cpp) | 23 | Тест DCM |
| [tests/Figures/Point2DGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Figures/Point2DGTEST.cpp) | 38 | Тест DCM |
| [tests/Figures/PointBaseGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Figures/PointBaseGTEST.cpp) | 30 | Тест DCM |
| [tests/Requirements/LineCircleDistGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/LineCircleDistGTEST.cpp) | 65 | Тест DCM |
| [tests/Requirements/LineHorizontalGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/LineHorizontalGTEST.cpp) | 50 | Тест DCM |
| [tests/Requirements/LineLineAngleGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/LineLineAngleGTEST.cpp) | 65 | Тест DCM |
| [tests/Requirements/LineLineParallelGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/LineLineParallelGTEST.cpp) | 61 | Тест DCM |
| [tests/Requirements/LineLinePerpendicularGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/LineLinePerpendicularGTEST.cpp) | 62 | Тест DCM |
| [tests/Requirements/LineVerticalGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/LineVerticalGTEST.cpp) | 50 | Тест DCM |
| [tests/Requirements/PointLineDistGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/PointLineDistGTEST.cpp) | 53 | Тест DCM |
| [tests/Requirements/PointOnLineGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/PointOnLineGTEST.cpp) | 101 | Тест DCM |
| [tests/Requirements/PointOnPointGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/PointOnPointGTEST.cpp) | 78 | Тест DCM |
| [tests/Requirements/PointPointDistGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Requirements/PointPointDistGTEST.cpp) | 84 | Тест DCM |
| [tests/System/FixRequirementSystemGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/System/FixRequirementSystemGTEST.cpp) | 274 | Тест DCM |
| [tests/System/RequirementFunctionSystemGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/System/RequirementFunctionSystemGTEST.cpp) | 76 | Тест DCM |
| [tests/System/RequirementSystemGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/System/RequirementSystemGTEST.cpp) | 283 | Тест DCM |
| [tests/Utils/FigureDescriptorGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Utils/FigureDescriptorGTEST.cpp) | 148 | Тест DCM |
| [tests/Utils/IDGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Utils/IDGTEST.cpp) | 59 | Тест DCM |
| [tests/Utils/IDGeneratorGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Utils/IDGeneratorGTEST.cpp) | 45 | Тест DCM |
| [tests/Utils/RequirementDescriptorGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/Utils/RequirementDescriptorGTEST.cpp) | 304 | Тест DCM |
| [tests/function/FixCoordinateFunctionGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/function/FixCoordinateFunctionGTEST.cpp) | 197 | Тест DCM |
| [tests/function/RequirementFunctionFactoryGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/function/RequirementFunctionFactoryGTEST.cpp) | 164 | Тест DCM |
| [tests/function/RequirementFunctionGTEST.cpp](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/tests/function/RequirementFunctionGTEST.cpp) | 145 | Тест DCM |
| [math/.clang-format](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/.clang-format) | 1 | Конфигурация / документация |
| [math/.clang-tidy](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/.clang-tidy) | 1 | Конфигурация / документация |
| [math/.github/workflows/allSituations.yml](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/.github/workflows/allSituations.yml) | 45 | Конфигурация / документация |
| [math/.gitignore](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/.gitignore) | 2 | Конфигурация / документация |
| [math/CMakeLists.txt](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/CMakeLists.txt) | 189 | Конфигурация / документация |
| [math/LICENSE](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/LICENSE) | 21 | Конфигурация / документация |
| [math/README.md](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/README.md) | 2 | Конфигурация / документация |
| [math/headers/core/ErrorFunction.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/core/ErrorFunction.h) | 146 | Математический подмодуль |
| [math/headers/core/Function.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/core/Function.h) | 617 | Математический подмодуль |
| [math/headers/decomposition/QR.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/decomposition/QR.h) | 66 | Математический подмодуль |
| [math/headers/decomposition/SparseQR.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/decomposition/SparseQR.h) | 110 | Математический подмодуль |
| [math/headers/decomposition/materials/CGS .png](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/decomposition/materials/CGS .png) | — | Учебные материалы; ссылок в коде/README нет |
| [math/headers/decomposition/materials/CGS2.png](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/decomposition/materials/CGS2.png) | — | Учебные материалы; ссылок в коде/README нет |
| [math/headers/decomposition/materials/IGS.png](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/decomposition/materials/IGS.png) | — | Учебные материалы; ссылок в коде/README нет |
| [math/headers/decomposition/materials/MGS.png](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/decomposition/materials/MGS.png) | — | Учебные материалы; ссылок в коде/README нет |
| [math/headers/decomposition/ordering/SparseQrColamd.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/decomposition/ordering/SparseQrColamd.h) | 1228 | Математический подмодуль |
| [math/headers/graph/Graph.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/graph/Graph.h) | 591 | Математический подмодуль |
| [math/headers/graph/GraphObjects.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/graph/GraphObjects.h) | 53 | Математический подмодуль |
| [math/headers/graph/InheritanceGraph.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/graph/InheritanceGraph.h) | 64 | Математический подмодуль |
| [math/headers/graph/Politicians.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/graph/Politicians.h) | 20 | Математический подмодуль |
| [math/headers/linear_algebra/Matrix.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/linear_algebra/Matrix.h) | 1372 | Математический подмодуль |
| [math/headers/linear_algebra/SparseMatrix.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/linear_algebra/SparseMatrix.h) | 1218 | Математический подмодуль |
| [math/headers/optimizers/base/Optimizer.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/base/Optimizer.h) | 20 | Математический подмодуль |
| [math/headers/optimizers/eigen/AdamOptimizer.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/eigen/AdamOptimizer.h) | 32 | Математический подмодуль |
| [math/headers/optimizers/eigen/EigenOptimizer.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/eigen/EigenOptimizer.h) | 13 | Математический подмодуль |
| [math/headers/optimizers/eigen/LMForTest.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/eigen/LMForTest.h) | 94 | Математический подмодуль |
| [math/headers/optimizers/eigen/LMWithSparse.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/eigen/LMWithSparse.h) | 141 | Математический подмодуль |
| [math/headers/optimizers/eigen/StochasticGradientOptimizer.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/eigen/StochasticGradientOptimizer.h) | 31 | Математический подмодуль |
| [math/headers/optimizers/matrix/GradientOptimizer.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/matrix/GradientOptimizer.h) | 29 | Математический подмодуль |
| [math/headers/optimizers/matrix/LevenbergMarquardtSolver.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/matrix/LevenbergMarquardtSolver.h) | 39 | Математический подмодуль |
| [math/headers/optimizers/matrix/MatrixOptimizer.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/matrix/MatrixOptimizer.h) | 12 | Математический подмодуль |
| [math/headers/optimizers/matrix/NewtonGaussSolver.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/matrix/NewtonGaussSolver.h) | 26 | Математический подмодуль |
| [math/headers/optimizers/matrix/NewtonOptimizer.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/matrix/NewtonOptimizer.h) | 24 | Математический подмодуль |
| [math/headers/optimizers/matrix/sparse/SparseLevenbergMarquardtSolver.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/optimizers/matrix/sparse/SparseLevenbergMarquardtSolver.h) | 41 | Математический подмодуль |
| [math/headers/tasks/base/Task.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/tasks/base/Task.h) | 17 | Математический подмодуль |
| [math/headers/tasks/eigen/LSMFORLMTask.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/tasks/eigen/LSMFORLMTask.h) | 126 | Математический подмодуль |
| [math/headers/tasks/eigen/TaskEigen.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/tasks/eigen/TaskEigen.h) | 15 | Математический подмодуль |
| [math/headers/tasks/matrix/LSMTask.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/tasks/matrix/LSMTask.h) | 137 | Математический подмодуль |
| [math/headers/tasks/matrix/SparseLSMTask.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/tasks/matrix/SparseLSMTask.h) | 425 | Математический подмодуль |
| [math/headers/tasks/matrix/TaskF.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/tasks/matrix/TaskF.h) | 79 | Математический подмодуль |
| [math/headers/tasks/matrix/TaskMatrix.h](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/headers/tasks/matrix/TaskMatrix.h) | 15 | Математический подмодуль |
| [math/src/core/ErrorFunctions.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/core/ErrorFunctions.cc) | 273 | Математический подмодуль |
| [math/src/core/Function.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/core/Function.cc) | 712 | Математический подмодуль |
| [math/src/decomposition/QR.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/decomposition/QR.cc) | 517 | Математический подмодуль |
| [math/src/decomposition/SparseQR.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/decomposition/SparseQR.cc) | 937 | Математический подмодуль |
| [math/src/optimizers/eigen/AdamOptimizer.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/optimizers/eigen/AdamOptimizer.cc) | 59 | Математический подмодуль |
| [math/src/optimizers/eigen/StochasticGradientOptimizer.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/optimizers/eigen/StochasticGradientOptimizer.cc) | 54 | Математический подмодуль |
| [math/src/optimizers/matrix/GradientOptimizer.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/optimizers/matrix/GradientOptimizer.cc) | 42 | Математический подмодуль |
| [math/src/optimizers/matrix/LevenbergMarquardtSolver.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/optimizers/matrix/LevenbergMarquardtSolver.cc) | 72 | Математический подмодуль |
| [math/src/optimizers/matrix/NewtonGaussSolver.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/optimizers/matrix/NewtonGaussSolver.cc) | 100 | Математический подмодуль |
| [math/src/optimizers/matrix/NewtonOptimizer.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/optimizers/matrix/NewtonOptimizer.cc) | 59 | Математический подмодуль |
| [math/src/optimizers/matrix/sparse/SparseLevenbergMarquardtSolver.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/src/optimizers/matrix/sparse/SparseLevenbergMarquardtSolver.cc) | 132 | Математический подмодуль |
| [math/tests/CMakeLists.txt](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/CMakeLists.txt) | 98 | Тест математического подмодуля |
| [math/tests/core/ErrorFunctionsTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/core/ErrorFunctionsTests.cc) | 167 | Тест математического подмодуля |
| [math/tests/core/FunctionTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/core/FunctionTest.cc) | 1145 | Тест математического подмодуля |
| [math/tests/decomposition/QRPerformanceTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/decomposition/QRPerformanceTests.cc) | 31 | Тест математического подмодуля |
| [math/tests/decomposition/SparseQRPerformanceTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/decomposition/SparseQRPerformanceTests.cc) | 631 | Тест математического подмодуля |
| [math/tests/decomposition/SparseQRTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/decomposition/SparseQRTests.cc) | 570 | Тест математического подмодуля |
| [math/tests/decomposition/SparseQrColamdTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/decomposition/SparseQrColamdTests.cc) | 444 | Тест математического подмодуля |
| [math/tests/decomposition/TestsQR.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/decomposition/TestsQR.cc) | 1549 | Тест математического подмодуля |
| [math/tests/graph/graphgtests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/graph/graphgtests.cc) | 677 | Тест математического подмодуля |
| [math/tests/linear_algebra/SparseMatrixComparisonTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/linear_algebra/SparseMatrixComparisonTests.cc) | 153 | Тест математического подмодуля |
| [math/tests/linear_algebra/SparseMatrixTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/linear_algebra/SparseMatrixTests.cc) | 306 | Тест математического подмодуля |
| [math/tests/linear_algebra/TestsMatrix.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/linear_algebra/TestsMatrix.cc) | 1205 | Тест математического подмодуля |
| [math/tests/optimizers/eigen/AdamOptimizerTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/eigen/AdamOptimizerTest.cc) | 100 | Тест математического подмодуля |
| [math/tests/optimizers/eigen/SGDOptimizerTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/eigen/SGDOptimizerTest.cc) | 73 | Тест математического подмодуля |
| [math/tests/optimizers/matrix/GradientOptimizerTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/matrix/GradientOptimizerTest.cc) | 133 | Тест математического подмодуля |
| [math/tests/optimizers/matrix/LMTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/matrix/LMTest.cc) | 196 | Тест математического подмодуля |
| [math/tests/optimizers/matrix/NewtonGaussSolverTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/matrix/NewtonGaussSolverTests.cc) | 129 | Тест математического подмодуля |
| [math/tests/optimizers/matrix/NewtonOptimizerTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/matrix/NewtonOptimizerTest.cc) | 139 | Тест математического подмодуля |
| [math/tests/optimizers/matrix/OurLMTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/matrix/OurLMTest.cc) | 171 | Тест математического подмодуля |
| [math/tests/optimizers/matrix/SparseLMComparisonPerformanceTests.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/matrix/SparseLMComparisonPerformanceTests.cc) | 314 | Тест математического подмодуля |
| [math/tests/optimizers/matrix/SparseLevenbergMarquardtSolverTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/optimizers/matrix/SparseLevenbergMarquardtSolverTest.cc) | 89 | Тест математического подмодуля |
| [math/tests/tasks/LSMTaskTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/tasks/LSMTaskTest.cc) | 230 | Тест математического подмодуля |
| [math/tests/tasks/TaskTest.cc](C:/Users/Killu/OneDrive/Desktop/ourpaintdcm/math/tests/tasks/TaskTest.cc) | 98 | Тест математического подмодуля |

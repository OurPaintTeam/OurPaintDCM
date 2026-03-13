# OurPaintDCM

`OurPaintDCM` - это C++ библиотека для решения геометрических ограничений в 2D-эскизах CAD. Она хранит геометрию, связывает объекты ограничениями, строит систему уравнений и решает её в разных режимах: глобально, локально по компоненте и интерактивно во время перетаскивания.

Проект ориентирован на сценарий параметрического эскизника. Пользователь создаёт точки, линии, окружности и дуги после этого CAD добавляет геометрические ограничения, решатель решает положение зависимых объектов и приложение получает новую координату

## Что уже есть

### Геометрические примитивы

- `Point2D`
- `Line`
- `Circle`
- `Arc`

Все примитивы можно создавать либо напрямую по координатам, либо через уже существующие опорные точки по их `ID`.

### Поддерживаемые ограничения

Сейчас в высокоуровневом API доступны:

- расстояние между двумя точками;
- совпадение двух точек;
- расстояние от точки до линии;
- принадлежность точки линии;
- расстояние от линии до окружности;
- принадлежность линии окружности;
- параллельность линий;
- перпендикулярность линий;
- фиксированный угол между линиями;
- вертикальность линии;
- горизонтальность линии;
- фиксация точки;
- фиксация линии;
- фиксация окружности.

`LineInCircle` уже присутствует в перечислениях и дескрипторах, но через единый `addRequirement()` в текущей версии ещё не поддержан(в будущем исключена)

### Что умеет менеджер системы

Класс `DCMManager` даёт единый API для:

- добавления и удаления геометрии;
- обновления координат точек и радиусов окружностей;
- добавления, удаления и изменения параметров ограничений;
- разбиения сцены на компоненты связности;
- локального решения только затронутой части эскиза;
- диагностики состояния системы по Якобиану.

### Режимы решения

- `GLOBAL` - решить всю систему целиком;
- `LOCAL` - решить только одну компоненту связности;
- `DRAG` - пересчитывать систему при редактировании геометрии, удобно для интерактивного перетаскивания.

## Сборка

Требования:

- CMake `3.26+`
- компилятор с поддержкой `C++20`
- доступ в интернет при первой сборке для загрузки `Eigen` и `GoogleTest`

Сборка библиотеки:

```bash
git clone --recurse-submodules https://github.com/OurPaintTeam/OurPaintDCM.git
cd OurPaintDCM
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Запуск тестов:

```bash
cd build
ctest --output-on-failure
```

Проект собирает статическую библиотеку `OurPaintDCM`.

## Быстрый старт

Ниже минимальный сценарий: создать две точки, задать между ними расстояние и решить систему.

```cpp
#include "DCMManager.h"

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

int main() {
    DCMManager dcm;

    auto p1 = dcm.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = dcm.addFigure(FigureDescriptor::point(30.0, 0.0));

    dcm.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 50.0));

    dcm.setSolveMode(SolveMode::GLOBAL);
    const bool ok = dcm.solve();

    if (!ok) {
        return 1;
    }

    auto a = dcm.getFigure(p1);
    auto b = dcm.getFigure(p2);

    // Здесь CAD может забрать новые координаты и перерисовать эскиз.
    return 0;
}
```

## Как пользоваться

### 1. Создать геометрию

Можно добавлять объекты по координатам:

```cpp
auto p  = dcm.addFigure(FigureDescriptor::point(10.0, 20.0));
auto l  = dcm.addFigure(FigureDescriptor::line(0.0, 0.0, 100.0, 10.0));
auto c  = dcm.addFigure(FigureDescriptor::circle(40.0, 40.0, 25.0));
auto a  = dcm.addFigure(FigureDescriptor::arc(0.0, 0.0, 10.0, 0.0, 5.0, 5.0));
```

Или собирать составные объекты из уже существующих точек:

```cpp
auto p1 = dcm.addFigure(FigureDescriptor::point(0.0, 0.0));
auto p2 = dcm.addFigure(FigureDescriptor::point(100.0, 0.0));
auto line = dcm.addFigure(FigureDescriptor::line(p1, p2));
```

### 2. Добавить ограничения

Примеры:

```cpp
dcm.addRequirement(RequirementDescriptor::horizontal(line));
dcm.addRequirement(RequirementDescriptor::vertical(otherLine));
dcm.addRequirement(RequirementDescriptor::pointOnLine(p1, line));
dcm.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 100.0));
dcm.addRequirement(RequirementDescriptor::lineLineParallel(line1, line2));
dcm.addRequirement(RequirementDescriptor::fixPoint(p1));
```

### 3. Запустить решение

```cpp
dcm.setSolveMode(SolveMode::GLOBAL);
bool ok = dcm.solve();
```

Если нужно решать только часть эскиза:

```cpp
auto component = dcm.getComponentForFigure(line);
if (component.has_value()) {
    dcm.setSolveMode(SolveMode::LOCAL);
    dcm.solve(component.value());
}
```

### 4. Забрать результат обратно в CAD

После `solve()` обновлённые координаты уже записаны в геометрию внутри `DCMManager`. Их можно получить через `getFigure()`:

```cpp
auto desc = dcm.getFigure(line);
auto p1Id = desc->pointIds[0];
auto p2Id = desc->pointIds[1];

auto p1Desc = dcm.getFigure(p1Id);
auto p2Desc = dcm.getFigure(p2Id);
```

То же касается окружностей и дуг

### 5. Обновлять геометрию при редактировании

Для интерактивного сценария удобно использовать режим `DRAG`:

```cpp
dcm.setSolveMode(SolveMode::DRAG);
dcm.updatePoint(PointUpdateDescriptor(p1, 120.0, 35.0));
```

В этом режиме `updatePoint()` и `updateCircle()` автоматически пытаются пересчитать затронутую компоненту.

## Диагностика системы

Низкоуровневая система ограничений умеет:

- вычислять вектор невязок;
- строить Якобиан;
- вычислять `J^T J`;
- определять состояние системы.

Типичный сценарий диагностики:

```cpp
auto status = dcm.getRequirementSystem().diagnose();
```

Возможные состояния:

- `WELL_CONSTRAINED`
- `UNDER_CONSTRAINED`
- `OVER_CONSTRAINED`
- `SINGULAR_SYSTEM`
- `EMPTY`
- `UNKNOWN`

## Как интегрировать в свой CAD

### Рекомендуемый сценарий

На текущем этапе `OurPaintDCM` удобнее всего встраивать в CAD как исходники или как git-submodule внутри основного проекта.

#### Шаг 1. Подключить проект в CMake

```cmake
add_subdirectory(external/OurPaintDCM)

target_link_libraries(my_cad PRIVATE OurPaintDCM)

target_include_directories(my_cad PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/headers
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/headers/figures
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/headers/functions
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/headers/system
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/headers/requirements
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/headers/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/optimizers
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/optimizers/MatrixOptimizers
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/optimizers/EigenOptimizers
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/decomposition
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/graph
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/Tasks
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/Tasks/Matrix
    ${CMAKE_CURRENT_SOURCE_DIR}/external/OurPaintDCM/math/headers/Tasks/Eigen
)
```

Библиотека уже собирается как `OurPaintDCM`, но для внешнего CAD сейчас проще и надёжнее явно передать include-пути, которые используются внутри проекта.

#### Шаг 2. Сделать адаптер между объектами CAD и DCM

Обычно достаточно хранить отображения:

- `cadEntityId -> dcmFigureId`
- `cadConstraintId -> dcmRequirementId`

Тогда интеграция выглядит так:

1. Пользователь создаёт примитив в CAD.
2. CAD создаёт соответствующий `FigureDescriptor`.
3. `DCMManager` возвращает `ID`, который вы сохраняете в своей модели.
4. При добавлении ограничения CAD создаёт `RequirementDescriptor` и тоже сохраняет полученный `ID`.
5. После `solve()` CAD читает обновлённые координаты и обновляет экран.

#### Шаг 3. Выбрать правильный режим решения

- `GLOBAL` подходит для полного пересчёта эскиза после серии изменений.
- `LOCAL` удобен, если у вас несколько независимых групп геометрии.
- `DRAG` лучше всего подходит для интерактивного перемещения ручек и точек мышью.

### Практический поток для эскизника

Типичный жизненный цикл в CAD:

1. Создать в `DCMManager` фигуры, соответствующие объектам эскиза.
2. При создании зависимостей добавить ограничения через `RequirementDescriptor`.
3. Во время редактирования обновлять точки через `PointUpdateDescriptor`.
4. В `DRAG` или `LOCAL` режиме решать только нужную компоненту.
5. После решения синхронизировать новые координаты обратно в представление CAD.

## Ограничения текущей версии

- только 2D;
- основной сценарий интеграции сейчас - встраивание в исходниках.

## Тесты

В проекте уже есть покрытие для:

- дескрипторов фигур и ограничений;
- функций ограничений;
- `RequirementSystem`;
- `DCMManager`;
- режимов решения `GLOBAL`, `LOCAL`, `DRAG`.

Если вы внедряете библиотеку в CAD, тесты служат хорошей живой спецификацией поведения API.
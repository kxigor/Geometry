# Geometry Library

**C++ библиотека для вычислительной геометрии с поддержкой 2D/3D операций и точными вычислениями**

*Geometry Library* делится на две библиотеки:
+ *Geometry 2D Library*
+ *Geometry 3D Library*

о них подробнее ниже.  

### Инструменты
- *C++ 23*
- *Google tests*
- *CMake/CTest 3.16* 
- *Clang-format-19*
- *Clang-tidy-19*

### Компиляция
```shell
./runbuild.sh
```

### Расположение exe файлов
```shell
./build/bin/
├── D  # Пример работы с разностью Минковского
├── F  # Пример 2D геометрии  
├── G  # Пример 3D геометрии
└── H  # Дополнительные примеры
```

### Тестирование
```shell
./runtests.sh
```

### Структура проекта
```shell
├── apps                # Использование библиотеки на примере простых задач
├── include
│   ├── Double          # Заголовки double
│   └── Geometry        # Заголовки Geometry
├── runbuild.sh         # Компиляция
├── runtests.sh         # Тестирование
├── src                 # Имплементации
│   ├── Geometry
│   └── Geometry3d
└── tests               
    ├── GoogleTests     # Unit Google тесты
    ├── Primitive       # Простые тесты на приложения
    └── StaticAnalysis  # Статический анализ (tidy, format)
```

### Точные вычисления
```cpp
template <std::floating_point DoubleType, DoubleType Eps = std::numeric_limits<DoubleType>::epsilon()>
struct Double{...};
```
Была реализована библиотека для работы с числами с плавающей точкой с определенной точностью.

# Geometry 2D Library

Библиотека для вычислительной геометрии с поддержкой точных вычислений и различных геометрических примитивов.

## Основные возможности
### Геометрические примитивы
- **Vector/Point** - вектора и точки с точными вычислениями
- **Segment** - отрезки с проверкой вырожденности
- **Line** - линии в общем виде (Ax + By + C = 0)
- **Ray** - лучи с поддержкой вращения
- **HalfPlane** - полуплоскости для построения выпуклых оболочек

### Алгоритмы
- **GrahamsScan** - алгоритм Грэхема для построения выпуклой оболочки
- **ConvexHull** - выпуклые оболочки с итераторами по ребрам
- **VoronoiDiagram** - диаграммы Вороного
- **Minkowski Difference** - разность Минковского

### Точные вычисления
```cpp
using ScalarT = Double<BaseDoubleT, kBasicAccuracy>;  // long double с точностью 1e-10
```

# Geometry 3D Library

Библиотека для трёхмерной вычислительной геометрии с поддержкой точных вычислений и операций с 3D-объектами.

## Основные возможности

### Геометрические примитивы 3D
- **Vector/Point** - 3D вектора и точки с полной арифметической поддержкой
- **Line** - 3D линии (точка + направление)
- **Plane** - плоскости (точка + нормаль) с предопределенными координатными плоскостями
- **Face** - треугольные грани с автоматическим вычислением нормали
- **Segment** - 3D отрезки с проверкой вырожденности

### Алгоритмы
- **ConvexHull** - 3D выпуклая оболочка (алгоритм заворачивания подарка)
- Расстояния между 3D объектами
- Проверки пересечений
- Вычисление нормалей плоскостей

### Точные вычисления
```cpp
using ScalarT = Double<BaseDoubleT, kBasicAccuracy>;  // long double с точностью 1e-10
```

# Примеры использования

## Geometry 2D Library

### Работа с точками и векторами
```cpp
#include "Geometry/Geometry.hpp"

geometry::Point p1{1.0, 2.0};
geometry::Point p2{3.0, 4.0};

// Векторные операции
auto sum = p1 + p2;
auto diff = p1 - p2; 
auto scaled = p1 * 2.0;

// Геометрические операции
auto length = p1.Length();
auto dot = p1.ScalarProduct(p2);
auto cross = p1.VectorProduct(p2);
```

### Построение выпуклой оболочки
```cpp
geometry::PointsT points = {{0,0}, {1,1}, {2,0}, {1,-1}};

// Из произвольных точек
auto hull1 = geometry::ConvexHull::InitViaArbitraryPoints(points);

// Из неупорядоченной оболочки
auto hull2 = geometry::ConvexHull::InitViaUnorderedConvexHull(points);

// Прямая инициализация (уже упорядочено)
auto hull3 = geometry::ConvexHull::InitDirect(points);
```

### Проверка пересечений
```cpp
geometry::Segment seg1{{0,0}, {1,1}};
geometry::Segment seg2{{0,1}, {1,0}};
geometry::Point point{0.5, 0.5};

bool intersects = geometry::IsIntersect(seg1, seg2);  // true
bool on_segment = geometry::IsIntersect(point, seg1); // true
```

### Разность Минковского
```cpp
geometry::ConvexHull hull_a = /* ... */;
geometry::ConvexHull hull_b = /* ... */;

auto minkowski_diff = geometry::CalcMinkowskiDifference(hull_a, hull_b);
```

### Диаграмма Вороного
```cpp
geometry::PointsT sites = {{0,0}, {1,0}, {0,1}, {1,1}};
geometry::PointsT border = {{-1,-1}, {2,-1}, {2,2}, {-1,2}};

geometry::VoronoiDiagram diagram(border, sites);
auto cells = diagram.TakeCells();
```
## Geometry 3D Library

### Работа с 3D точками и векторами

```cpp
#include "Geometry/Geometry3d.hpp"

geometry3d::Point p1{1.0, 2.0, 3.0};
geometry3d::Point p2{4.0, 5.0, 6.0};

// Векторные операции
auto sum = p1 + p2;
auto cross_product = p1.VectorProduct(p2);  // Векторное произведение
auto dot_product = p1.ScalarProduct(p2);    // Скалярное произведение

// Нормализация
auto normalized = p1.GetNormalized();
```

```cpp
// Плоскость через три точки
geometry3d::Plane plane1{p1, p2, p3};

// Плоскость из точки и нормали
geometry3d::Vector normal{0, 0, 1};
geometry3d::Plane plane2{origin, normal};

// Предопределенные координатные плоскости
auto point_on_oxy = geometry3d::Oxy;
auto point_on_oxz = geometry3d::Oxz;

// Треугольная грань
geometry3d::Face face{p1, p2, p3};
auto face_normal = face.GetNormal();
```

### Построение 3D выпуклой оболочки
```cpp
geometry3d::PointsT points = {
    {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}, 
    {1,1,1}, {0,1,1}, {1,0,1}, {1,1,0}
};

geometry3d::ConvexHull hull(points);

// Вычисление расстояния от точки до оболочки
geometry3d::Point test_point{0.5, 0.5, 0.5};
auto distance = hull.CalcDistance(test_point);

// Получение всех граней
const auto& faces = hull.GetFaces();
```

### Работа с отрезками и линиями
```cpp
geometry3d::Segment seg{{0,0,0}, {1,1,1}};
geometry3d::Line line{{0,0,0}, {1,0,0}};  // origin + direction

// Проверка вырожденности
bool is_degenerate = seg.IsDegenerate();

// Вычисление расстояний
auto dist_to_seg = geometry3d::CalcDistance(point, seg);
auto dist_to_face = geometry3d::CalcDistance(point, face);
```
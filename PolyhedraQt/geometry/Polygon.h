#pragma once
#include <QColor>
#include <QVector>
#include "Point3D.h"

class Polygon {
public:
    QVector<int> indices;
    QColor color;
    Point3D normal; // Внешняя нормаль грани.
    QVector<Point3D> vertexNormals; // Нормали в углах грани.

    Polygon() = default;
    explicit Polygon(const QVector<int>& vertexIndices,
                     const QColor& faceColor = QColor(95, 160, 210))
        : indices(vertexIndices), color(faceColor) {}
};

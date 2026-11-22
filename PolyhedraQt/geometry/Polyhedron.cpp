#include "Polyhedron.h"

#include <cmath>
#include <algorithm>
#include <QPointF>

namespace {
Point3D cross(const Point3D& a, const Point3D& b)
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

double dot(const Point3D& a, const Point3D& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Point3D subtract(const Point3D& a, const Point3D& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Point3D normalized(const Point3D& value)
{
    const double length = std::sqrt(dot(value, value));
    if (length < 1e-12) {
        return {};
    }
    return {value.x / length, value.y / length, value.z / length};
}

bool transformNormal(const Matrix4& matrix, const Point3D& normal, Point3D& result)
{
    // Для нормалей используется строго обратная транспонированная матрица M^-T.
    const double a = matrix.m[0][0];
    const double b = matrix.m[0][1];
    const double c = matrix.m[0][2];
    const double d = matrix.m[1][0];
    const double e = matrix.m[1][1];
    const double f = matrix.m[1][2];
    const double g = matrix.m[2][0];
    const double h = matrix.m[2][1];
    const double i = matrix.m[2][2];

    const double determinant = a * (e * i - f * h)
                             - b * (d * i - f * g)
                             + c * (d * h - e * g);
    if (std::abs(determinant) < 1e-12) {
        return false;
    }

    // Cofactor matrix / determinant = (A^-1)^T.
    const double invT[3][3] = {
        {(e * i - f * h) / determinant, (f * g - d * i) / determinant, (d * h - e * g) / determinant},
        {(c * h - b * i) / determinant, (a * i - c * g) / determinant, (b * g - a * h) / determinant},
        {(b * f - c * e) / determinant, (c * d - a * f) / determinant, (a * e - b * d) / determinant}
    };

    result = normalized({
        invT[0][0] * normal.x + invT[0][1] * normal.y + invT[0][2] * normal.z,
        invT[1][0] * normal.x + invT[1][1] * normal.y + invT[1][2] * normal.z,
        invT[2][0] * normal.x + invT[2][1] * normal.y + invT[2][2] * normal.z
    });
    return true;
}
}

void Polyhedron::applyMatrix(const Matrix4& matrix)
{
    // До преобразования гарантируем наличие исходных нормалей.
    if (faces.isEmpty()) {
        return;
    }
    if (faces.first().normal.x == 0.0 && faces.first().normal.y == 0.0
        && faces.first().normal.z == 0.0) {
        calculateNormals();
    }

    for (Point3D& point : vertices) {
        point = matrix.transformPoint(point);
    }

    for (Polygon& face : faces) {
        Point3D transformed;
        if (transformNormal(matrix, face.normal, transformed)) {
            face.normal = transformed;
        }
    }
    calculateVertexNormals();
}

Point3D Polyhedron::center() const
{
    if (vertices.isEmpty()) {
        return {};
    }

    Point3D result;
    for (const Point3D& point : vertices) {
        result.x += point.x;
        result.y += point.y;
        result.z += point.z;
    }

    const double count = vertices.size();
    result.x /= count;
    result.y /= count;
    result.z /= count;
    return result;
}

void Polyhedron::normalizeSize(double targetRadius)
{
    if (vertices.isEmpty()) {
        return;
    }

    const Point3D centerPoint = center();
    double maxRadius = 0.0;

    for (const Point3D& point : vertices) {
        const double dx = point.x - centerPoint.x;
        const double dy = point.y - centerPoint.y;
        const double dz = point.z - centerPoint.z;
        maxRadius = qMax(maxRadius, std::sqrt(dx * dx + dy * dy + dz * dz));
    }

    if (maxRadius > 1e-9) {
        const double scale = targetRadius / maxRadius;
        applyMatrix(Matrix4::translation(-centerPoint.x, -centerPoint.y, -centerPoint.z)
                    * Matrix4::scaling(scale, scale, scale));
    }
}

void Polyhedron::calculateNormals()
{
    const Point3D modelCenter = center();

    for (Polygon& face : faces) {
        face.normal = {};
        if (face.indices.size() < 3) {
            continue;
        }

        const int i0 = face.indices[0];
        const int i1 = face.indices[1];
        const int i2 = face.indices[2];
        if (i0 < 0 || i1 < 0 || i2 < 0
            || i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) {
            continue;
        }

        const Point3D edge1 = subtract(vertices[i1], vertices[i0]);
        const Point3D edge2 = subtract(vertices[i2], vertices[i0]);
        Point3D normal = normalized(cross(edge1, edge2));

        Point3D faceCenter;
        for (int index : face.indices) {
            if (index >= 0 && index < vertices.size()) {
                faceCenter.x += vertices[index].x;
                faceCenter.y += vertices[index].y;
                faceCenter.z += vertices[index].z;
            }
        }
        const double count = face.indices.size();
        faceCenter.x /= count;
        faceCenter.y /= count;
        faceCenter.z /= count;

        if (dot(normal, subtract(faceCenter, modelCenter)) < 0.0) {
            normal = {-normal.x, -normal.y, -normal.z};
        }
        face.normal = normal;

        // Создаём простые UV-координаты для каждой грани проекцией на две оси.
        if (face.textureCoordinates.size() != face.indices.size()) {
            const double nx = std::abs(normal.x);
            const double ny = std::abs(normal.y);
            const double nz = std::abs(normal.z);
            const int dominantAxis = nx >= ny && nx >= nz ? 0 : (ny >= nz ? 1 : 2);
            double minA = 1e100;
            double maxA = -1e100;
            double minB = 1e100;
            double maxB = -1e100;
            for (int index : face.indices) {
                if (index < 0 || index >= vertices.size()) continue;
                const Point3D& point = vertices[index];
                const double a = dominantAxis == 0 ? point.y : point.x;
                const double b = dominantAxis == 2 ? point.y : point.z;
                minA = std::min(minA, a);
                maxA = std::max(maxA, a);
                minB = std::min(minB, b);
                maxB = std::max(maxB, b);
            }
            face.textureCoordinates.clear();
            for (int index : face.indices) {
                if (index < 0 || index >= vertices.size()) {
                    face.textureCoordinates.append(QPointF());
                    continue;
                }
                const Point3D& point = vertices[index];
                const double a = dominantAxis == 0 ? point.y : point.x;
                const double b = dominantAxis == 2 ? point.y : point.z;
                const double u = maxA > minA ? (a - minA) / (maxA - minA) : 0.0;
                const double v = maxB > minB ? (b - minB) / (maxB - minB) : 0.0;
                face.textureCoordinates.append(QPointF(u, v));
            }
        }
    }
    calculateVertexNormals();
}

void Polyhedron::calculateVertexNormals()
{
    // Нормаль каждого угла грани вычисляется как среднее нормалей тех
    // граней, которые имеют с ним общую вершину и образуют гладкий стык.
    // На резких рёбрах усреднения нет: сохраняется нормаль текущей грани.
    const double smoothCosine = 0.95;

    for (Polygon& currentFace : faces) {
        currentFace.vertexNormals.clear();
        for (int vertexIndex : currentFace.indices) {
            if (vertexIndex < 0 || vertexIndex >= vertices.size()) {
                currentFace.vertexNormals.append(currentFace.normal);
                continue;
            }

            Point3D sum;
            for (const Polygon& adjacentFace : faces) {
                if (!adjacentFace.indices.contains(vertexIndex)) {
                    continue;
                }
                const double cosine = dot(currentFace.normal, adjacentFace.normal);
                if (cosine >= smoothCosine) {
                    sum.x += adjacentFace.normal.x;
                    sum.y += adjacentFace.normal.y;
                    sum.z += adjacentFace.normal.z;
                }
            }

            const Point3D cornerNormal = normalized(sum);
            currentFace.vertexNormals.append(
                std::sqrt(dot(cornerNormal, cornerNormal)) > 1e-12
                    ? cornerNormal : currentFace.normal);
        }
    }
}

#include "PolyhedronWidget.h"
#include "../geometry/PolyhedronFactory.h"
#include "../math/Matrix4.h"

#include <QPainter>
#include <QImage>
#include <QPaintEvent>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
struct V3 {
    double x;
    double y;
    double z;
};

V3 toV3(const Point3D& p) { return {p.x, p.y, p.z}; }
V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 mul(V3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double length(V3 a) { return std::sqrt(dot(a, a)); }
V3 normalized(V3 a)
{
    const double l = length(a);
    return l > 1e-12 ? mul(a, 1.0 / l) : V3{0.0, 0.0, 0.0};
}

struct Vertex {
    V3 world;
    V3 camera;
    V3 normal;
    QPointF screen;
    QPointF uv;
    QColor gouraudColor;
};

int channel(double value)
{
    return qBound(0, static_cast<int>(std::round(value)), 255);
}

QColor scaleColor(const QColor& color, double intensity)
{
    return QColor(channel(color.red() * intensity),
                  channel(color.green() * intensity),
                  channel(color.blue() * intensity), color.alpha());
}

QColor multiplyColors(const QColor& a, const QColor& b)
{
    return QColor(a.red() * b.red() / 255,
                  a.green() * b.green() / 255,
                  a.blue() * b.blue() / 255,
                  a.alpha() * b.alpha() / 255);
}

QColor interpolateColor(const QColor& a, const QColor& b, const QColor& c,
                         double wa, double wb, double wc)
{
    return QColor(channel(a.red() * wa + b.red() * wb + c.red() * wc),
                  channel(a.green() * wa + b.green() * wb + c.green() * wc),
                  channel(a.blue() * wa + b.blue() * wb + c.blue() * wc));
}

QPointF defaultUv(const Point3D& point, int dominantAxis,
                  double minA, double maxA, double minB, double maxB)
{
    double a = point.x;
    double b = point.y;
    if (dominantAxis == 0) { a = point.y; b = point.z; }
    if (dominantAxis == 1) { a = point.x; b = point.z; }
    const double u = maxA > minA ? (a - minA) / (maxA - minA) : 0.0;
    const double v = maxB > minB ? (b - minB) / (maxB - minB) : 0.0;
    return QPointF(u, v);
}

QColor sampleTexture(const QImage& image, const QPointF& uv)
{
    if (image.isNull()) {
        return Qt::white;
    }
    const double u = uv.x() - std::floor(uv.x());
    const double v = uv.y() - std::floor(uv.y());
    const int x = qBound(0, static_cast<int>(u * (image.width() - 1)), image.width() - 1);
    const int y = qBound(0, static_cast<int>((1.0 - v) * (image.height() - 1)), image.height() - 1);
    return image.pixelColor(x, y);
}
}

PolyhedronWidget::PolyhedronWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(280, 280);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAutoFillBackground(true);
    setPalette(QPalette(QColor(248, 249, 251)));

    // Небольшая встроенная шахматная текстура доступна сразу после запуска.
    textureImage = QImage(128, 128, QImage::Format_ARGB32);
    for (int y = 0; y < textureImage.height(); ++y) {
        for (int x = 0; x < textureImage.width(); ++x) {
            const bool lightCell = ((x / 16) + (y / 16)) % 2 == 0;
            textureImage.setPixelColor(x, y, lightCell
                ? QColor(245, 205, 95) : QColor(55, 125, 185));
        }
    }

    camera.orbit(cameraYaw, cameraPitch, cameraDistance);
}

void PolyhedronWidget::setPolyhedron(const Polyhedron& p)
{
    polyhedron = p;
    if (!polyhedron.faces.isEmpty()) {
        const Point3D& normal = polyhedron.faces.first().normal;
        if (std::abs(normal.x) + std::abs(normal.y) + std::abs(normal.z) < 1e-12) {
            polyhedron.calculateNormals();
        }
    }
    update();
}

void PolyhedronWidget::setProjectionType(ProjectionType type)
{
    projectionType = type;
    update();
}

void PolyhedronWidget::setBackFaceCulling(bool enabled)
{
    cullingEnabled = enabled;
    update();
}

void PolyhedronWidget::setZBufferEnabled(bool enabled)
{
    zBufferEnabled = enabled;
    update();
}

void PolyhedronWidget::setCamera(double yaw, double pitch, double distance)
{
    cameraYaw = yaw;
    cameraPitch = pitch;
    cameraDistance = distance;
    camera.orbit(yaw, pitch, distance);
    update();
}

void PolyhedronWidget::setShadingMode(ShadingMode mode)
{
    shadingMode = mode;
    update();
}

void PolyhedronWidget::setLightPosition(const Point3D& position)
{
    lightPosition = position;
    update();
}

void PolyhedronWidget::setObjectColor(const QColor& color)
{
    objectColor = color;
    update();
}

void PolyhedronWidget::setTexture(const QImage& texture)
{
    textureImage = texture;
    update();
}

void PolyhedronWidget::setTextureEnabled(bool enabled)
{
    textureEnabled = enabled;
    update();
}

void PolyhedronWidget::paintEvent(QPaintEvent*)
{
    QPainter output(this);
    output.setRenderHint(QPainter::Antialiasing, true);
    output.fillRect(rect(), QColor(248, 249, 251));
    if (polyhedron.vertices.isEmpty()) {
        return;
    }

    // Сохраняем демонстрацию перекрытия объектов из лабораторной №8.
    QVector<Polyhedron> objects;
    objects.append(polyhedron);
    Polyhedron left = PolyhedronFactory::create(PolyhedronFactory::Hexahedron);
    left.applyMatrix(Matrix4::translation(-3.0, 0.0, 0.0));
    objects.append(left);
    Polyhedron rightPolyhedron = PolyhedronFactory::create(PolyhedronFactory::Octahedron);
    rightPolyhedron.applyMatrix(Matrix4::translation(3.0, 0.0, -0.4));
    objects.append(rightPolyhedron);

    camera.setPerspective(0.82, width() / double(qMax(1, height())), 0.1, 100.0);
    const V3 cameraPosition = toV3(camera.position);
    const V3 forward = normalized(mul(cameraPosition, -1.0));
    const V3 worldUp{0.0, 1.0, 0.0};
    const V3 right = normalized({forward.y * worldUp.z - forward.z * worldUp.y,
                                 forward.z * worldUp.x - forward.x * worldUp.z,
                                 forward.x * worldUp.y - forward.y * worldUp.x});
    const V3 up = normalized({right.y * forward.z - right.z * forward.y,
                              right.z * forward.x - right.x * forward.z,
                              right.x * forward.y - right.y * forward.x});
    const double focal = std::min(width(), height()) * camera.projectionMatrix.m[1][1] * 0.5;

    QImage image(size(), QImage::Format_ARGB32);
    image.fill(QColor(248, 249, 251));
    QVector<double> zBuffer(width() * height(), std::numeric_limits<double>::infinity());

    for (const Polyhedron& object : objects) {
        QVector<Vertex> vertices;
        vertices.reserve(object.vertices.size());

        for (int i = 0; i < object.vertices.size(); ++i) {
            const Point3D& point = object.vertices[i];
            const V3 world = toV3(point);
            const V3 relative = sub(world, cameraPosition);
            Vertex vertex{};
            vertex.world = world;
            vertex.camera = {dot(relative, right), dot(relative, up), dot(relative, forward)};
            if (projectionType == ProjectionType::Perspective) {
                if (vertex.camera.z <= 0.05) {
                    vertex.screen = QPointF(-10000.0, -10000.0);
                } else {
                    vertex.screen = QPointF(width() / 2.0 + focal * vertex.camera.x / vertex.camera.z,
                                            height() / 2.0 - focal * vertex.camera.y / vertex.camera.z);
                }
            } else {
                vertex.screen = QPointF(width() / 2.0 + focal * vertex.camera.x / cameraDistance,
                                        height() / 2.0 - focal * vertex.camera.y / cameraDistance);
            }
            vertices.append(vertex);
        }

        // Для каждой вершины усредняем нормали всех смежных граней (Gouraud/Phong).
        QVector<Point3D> vertexNormalSums(object.vertices.size());
        for (const Polygon& face : object.faces) {
            for (int index : face.indices) {
                if (index >= 0 && index < vertexNormalSums.size()) {
                    vertexNormalSums[index].x += face.normal.x;
                    vertexNormalSums[index].y += face.normal.y;
                    vertexNormalSums[index].z += face.normal.z;
                }
            }
        }
        for (int i = 0; i < vertices.size(); ++i) {
            vertices[i].normal = normalized(toV3(vertexNormalSums[i]));
            const V3 toLight = normalized(sub(toV3(lightPosition), vertices[i].world));
            const double diffuse = std::max(dot(vertices[i].normal, toLight), 0.0);
            vertices[i].gouraudColor = scaleColor(objectColor, 0.2 + diffuse);
        }

        for (const Polygon& face : object.faces) {
            if (face.indices.size() < 3) {
                continue;
            }
            bool valid = true;
            for (int index : face.indices) {
                if (index < 0 || index >= vertices.size()) {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                continue;
            }

            V3 faceCenter{0.0, 0.0, 0.0};
            for (int index : face.indices) {
                faceCenter = add(faceCenter, vertices[index].world);
            }
            faceCenter = mul(faceCenter, 1.0 / face.indices.size());
            const V3 faceNormal = toV3(face.normal);
            const V3 viewVector = sub(cameraPosition, faceCenter);
            if (cullingEnabled && dot(faceNormal, viewVector) <= 0.0) {
                continue;
            }

            // Если OBJ не содержит vt, строим простую плоскую UV-развёртку отдельно для грани.
            double minX = 1e100, minY = 1e100, minZ = 1e100;
            double maxX = -1e100, maxY = -1e100, maxZ = -1e100;
            for (int index : face.indices) {
                const Point3D& p = object.vertices[index];
                minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
                minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
                minZ = std::min(minZ, p.z); maxZ = std::max(maxZ, p.z);
            }
            const double ax = std::abs(faceNormal.x);
            const double ay = std::abs(faceNormal.y);
            const double az = std::abs(faceNormal.z);
            const int dominantAxis = ax >= ay && ax >= az ? 0 : (ay >= az ? 1 : 2);
            double minA = dominantAxis == 0 ? minY : minX;
            double maxA = dominantAxis == 0 ? maxY : maxX;
            double minB = dominantAxis == 2 ? minY : minZ;
            double maxB = dominantAxis == 2 ? maxY : maxZ;
            if (dominantAxis == 1) { minB = minZ; maxB = maxZ; }

            QVector<QPointF> faceUvs = face.textureCoordinates;
            if (faceUvs.size() != face.indices.size()) {
                faceUvs.clear();
                for (int index : face.indices) {
                    faceUvs.append(defaultUv(object.vertices[index], dominantAxis,
                                              minA, maxA, minB, maxB));
                }
            }

            // Разбиваем многоугольную грань на треугольники веером.
            for (int k = 1; k + 1 < face.indices.size(); ++k) {
                const int cornerIndices[3] = {0, k, k + 1};
                const Vertex* triangle[3] = {
                    &vertices[face.indices[cornerIndices[0]]],
                    &vertices[face.indices[cornerIndices[1]]],
                    &vertices[face.indices[cornerIndices[2]]]
                };
                if (triangle[0]->screen.x() < -1000.0 || triangle[1]->screen.x() < -1000.0
                    || triangle[2]->screen.x() < -1000.0) {
                    continue;
                }

                const double x0 = triangle[0]->screen.x();
                const double y0 = triangle[0]->screen.y();
                const double x1 = triangle[1]->screen.x();
                const double y1 = triangle[1]->screen.y();
                const double x2 = triangle[2]->screen.x();
                const double y2 = triangle[2]->screen.y();
                const double area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
                if (std::abs(area) < 1e-10) {
                    continue;
                }

                const int minPixelX = qBound(0, static_cast<int>(std::floor(std::min({x0, x1, x2}))), width() - 1);
                const int maxPixelX = qBound(0, static_cast<int>(std::ceil(std::max({x0, x1, x2}))), width() - 1);
                const int minPixelY = qBound(0, static_cast<int>(std::floor(std::min({y0, y1, y2}))), height() - 1);
                const int maxPixelY = qBound(0, static_cast<int>(std::ceil(std::max({y0, y1, y2}))), height() - 1);

                const QPointF uv0 = faceUvs[cornerIndices[0]];
                const QPointF uv1 = faceUvs[cornerIndices[1]];
                const QPointF uv2 = faceUvs[cornerIndices[2]];
                for (int y = minPixelY; y <= maxPixelY; ++y) {
                    for (int x = minPixelX; x <= maxPixelX; ++x) {
                        const double px = x + 0.5;
                        const double py = y + 0.5;
                        const double w0 = ((x1 - px) * (y2 - py) - (y1 - py) * (x2 - px)) / area;
                        const double w1 = ((x2 - px) * (y0 - py) - (y2 - py) * (x0 - px)) / area;
                        const double w2 = 1.0 - w0 - w1;
                        if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0) {
                            continue;
                        }

                        const double z = projectionType == ProjectionType::Perspective
                            ? 1.0 / (w0 / triangle[0]->camera.z
                                     + w1 / triangle[1]->camera.z
                                     + w2 / triangle[2]->camera.z)
                            : (w0 * triangle[0]->camera.z
                               + w1 * triangle[1]->camera.z
                               + w2 * triangle[2]->camera.z);
                        const int pixelIndex = y * width() + x;
                        if (zBufferEnabled && z >= zBuffer[pixelIndex]) {
                            continue;
                        }

                        // Для перспективы атрибуты интерполируются с учётом обратной глубины.
                        double wa = w0;
                        double wb = w1;
                        double wc = w2;
                        if (projectionType == ProjectionType::Perspective) {
                            wa /= triangle[0]->camera.z;
                            wb /= triangle[1]->camera.z;
                            wc /= triangle[2]->camera.z;
                            const double weightSum = wa + wb + wc;
                            if (std::abs(weightSum) < 1e-12) {
                                continue;
                            }
                            wa /= weightSum;
                            wb /= weightSum;
                            wc /= weightSum;
                        }

                        QColor litColor;
                        if (shadingMode == Gouraud) {
                            // Гуро: освещение Ламберта вычислено в вершинах, здесь смешиваются цвета.
                            litColor = interpolateColor(triangle[0]->gouraudColor,
                                                        triangle[1]->gouraudColor,
                                                        triangle[2]->gouraudColor,
                                                        wa, wb, wc);
                        } else {
                            // Фонг: сначала интерполируем нормали, затем нормализуем их.
                            const V3 normal = normalized(add(
                                add(mul(triangle[0]->normal, wa), mul(triangle[1]->normal, wb)),
                                mul(triangle[2]->normal, wc)));
                            const V3 worldPosition = add(
                                add(mul(triangle[0]->world, wa), mul(triangle[1]->world, wb)),
                                mul(triangle[2]->world, wc));
                            const V3 lightDirection = normalized(sub(toV3(lightPosition), worldPosition));
                            const V3 viewDirection = normalized(sub(cameraPosition, worldPosition));
                            const double normalLight = std::max(dot(normal, lightDirection), 0.0);
                            const V3 reflected = normalized(sub(mul(normal, 2.0 * normalLight), lightDirection));
                            const double specular = normalLight > 0.0
                                ? std::pow(std::max(dot(reflected, viewDirection), 0.0), 32.0)
                                : 0.0;
                            const double ambient = 0.2;
                            const double diffuse = normalLight;
                            const double highlight = 0.45 * specular;
                            litColor = QColor(
                                channel(objectColor.red() * (ambient + diffuse) + 255.0 * highlight),
                                channel(objectColor.green() * (ambient + diffuse) + 255.0 * highlight),
                                channel(objectColor.blue() * (ambient + diffuse) + 255.0 * highlight));
                        }

                        if (textureEnabled && !textureImage.isNull()) {
                            const QPointF uv(uv0.x() * wa + uv1.x() * wb + uv2.x() * wc,
                                            uv0.y() * wa + uv1.y() * wb + uv2.y() * wc);
                            litColor = multiplyColors(litColor, sampleTexture(textureImage, uv));
                        }

                        if (shadingMode == Gouraud && textureEnabled) {
                            // Цвет вершин уже содержит освещение; текстура умножается отдельно.
                        }
                        if (zBufferEnabled) {
                            zBuffer[pixelIndex] = z;
                        }
                        image.setPixelColor(x, y, litColor);
                    }
                }
            }
        }
    }

    output.drawImage(0, 0, image);
    output.setPen(QColor(60, 70, 85));
    const QString shadingName = shadingMode == Gouraud ? "Гуро" : "Фонг";
    output.drawText(12, 20,
                    QString("Камера: азимут %1°, наклон %2°, расстояние %3 | Шейдинг: %4")
                        .arg(cameraYaw, 0, 'f', 0)
                        .arg(cameraPitch, 0, 'f', 0)
                        .arg(cameraDistance, 0, 'f', 1)
                        .arg(shadingName));
}

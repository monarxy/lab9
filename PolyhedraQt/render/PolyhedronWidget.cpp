#include "PolyhedronWidget.h"
#include "../geometry/PolyhedronFactory.h"
#include "../math/Matrix4.h"

#include <QPainter>
#include <QPaintEvent>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
struct V3 {
    double x;
    double y;
    double z;

    V3() : x(0.0), y(0.0), z(0.0) {}
    V3(double xValue, double yValue, double zValue)
        : x(xValue), y(yValue), z(zValue) {}
};

V3 toV3(const Point3D& p) { return {p.x, p.y, p.z}; }
V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 mul(V3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b)
{
    return V3(a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}
double length(V3 a) { return std::sqrt(dot(a, a)); }
V3 normalized(V3 a)
{
    const double valueLength = length(a);
    return valueLength > 1e-12 ? mul(a, 1.0 / valueLength) : V3{};
}

struct Vertex {
    V3 world;
    V3 camera;
    QPointF screen;
};

struct Corner {
    const Vertex* vertex = nullptr;
    V3 normal;
    QPointF uv;
    QColor gouraudColor;
};

int channel(double value)
{
    return qBound(0, static_cast<int>(std::round(value)), 255);
}

QColor scaleColor(const QColor& color, double factor)
{
    return QColor(channel(color.red() * factor),
                  channel(color.green() * factor),
                  channel(color.blue() * factor), color.alpha());
}

QColor multiplyColors(const QColor& first, const QColor& second)
{
    return QColor(first.red() * second.red() / 255,
                  first.green() * second.green() / 255,
                  first.blue() * second.blue() / 255,
                  first.alpha() * second.alpha() / 255);
}

QColor lambertColor(const QColor& material, V3 normal, V3 lightDirection)
{
    normal = normalized(normal);
    lightDirection = normalized(lightDirection);
    const double diffuse = std::max(dot(normal, lightDirection), 0.0);
    return scaleColor(material, diffuse);
}

QColor toonDiffuseColor(const QColor& material, V3 normal, V3 lightDirection)
{
    // Лекция №11, слайд 43: именно три ступени 0.3 / 1.0 / 1.3.
    normal = normalized(normal);
    lightDirection = normalized(lightDirection);
    const double diffuse = std::max(dot(normal, lightDirection), 0.0);

    if (diffuse < 0.4) {
        return scaleColor(material, 0.3);
    }
    if (diffuse < 0.7) {
        return material;
    }
    return scaleColor(material, 1.3);
}

QColor sampleTextureNearest(const QImage& image, const QPointF& uv)
{
    if (image.isNull()) {
        return Qt::white;
    }

    // Повторяем текстуру за пределами диапазона [0, 1].
    const double u = uv.x() - std::floor(uv.x());
    const double v = uv.y() - std::floor(uv.y());

    // Берём один ближайший пиксель изображения.
    const int x = qBound(0, static_cast<int>(u * image.width()), image.width() - 1);
    const int y = qBound(0, static_cast<int>((1.0 - v) * image.height()),
                         image.height() - 1);
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

    camera.orbit(cameraYaw, cameraPitch, cameraDistance);
}

void PolyhedronWidget::setPolyhedron(const Polyhedron& p)
{
    polyhedron = p;
    if (!polyhedron.faces.isEmpty()) {
        const Polygon& firstFace = polyhedron.faces.first();
        const Point3D& normal = firstFace.normal;
        const bool normalsMissing = std::abs(normal.x) + std::abs(normal.y)
                                  + std::abs(normal.z) < 1e-12;
        const bool uvMissing = firstFace.textureCoordinates.size()
                            != firstFace.indices.size();
        if (normalsMissing || uvMissing) {
            polyhedron.calculateNormals();
        } else {
            polyhedron.calculateVertexNormals();
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
    textureImage = texture.convertToFormat(QImage::Format_ARGB32);
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

    QVector<Polyhedron> objects;
    objects.append(polyhedron);

    const int renderWidth = width();
    const int renderHeight = height();
    camera.setPerspective(0.82, renderWidth / double(qMax(1, renderHeight)), 0.1, 100.0);
    const V3 cameraPosition = toV3(camera.position);
    const V3 forward = normalized(mul(cameraPosition, -1.0));
    const V3 worldUp{0.0, 1.0, 0.0};
    const V3 right = normalized({forward.y * worldUp.z - forward.z * worldUp.y,
                                 forward.z * worldUp.x - forward.x * worldUp.z,
                                 forward.x * worldUp.y - forward.y * worldUp.x});
    const V3 up = normalized({right.y * forward.z - right.z * forward.y,
                              right.z * forward.x - right.x * forward.z,
                              right.x * forward.y - right.y * forward.x});
    const double focal = std::min(renderWidth, renderHeight)
                       * camera.projectionMatrix.m[1][1] * 0.5;

    QImage image(renderWidth, renderHeight, QImage::Format_ARGB32);
    image.fill(QColor(248, 249, 251));
    QVector<double> zBuffer(renderWidth * renderHeight, std::numeric_limits<double>::infinity());

    for (const Polyhedron& object : objects) {
        QVector<Vertex> vertices;
        vertices.reserve(object.vertices.size());

        for (const Point3D& point : object.vertices) {
            const V3 world = toV3(point);
            const V3 relative = sub(world, cameraPosition);
            Vertex vertex;
            vertex.world = world;
            vertex.camera = {dot(relative, right), dot(relative, up), dot(relative, forward)};

            if (projectionType == ProjectionType::Perspective) {
                if (vertex.camera.z <= 0.05) {
                    vertex.screen = QPointF(-10000.0, -10000.0);
                } else {
                    vertex.screen = QPointF(renderWidth / 2.0 + focal * vertex.camera.x / vertex.camera.z,
                                            renderHeight / 2.0 - focal * vertex.camera.y / vertex.camera.z);
                }
            } else {
                vertex.screen = QPointF(renderWidth / 2.0 + focal * vertex.camera.x / cameraDistance,
                                        renderHeight / 2.0 - focal * vertex.camera.y / cameraDistance);
            }
            vertices.append(vertex);
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

            V3 faceCenter{};
            for (int index : face.indices) {
                faceCenter = add(faceCenter, vertices[index].world);
            }
            faceCenter = mul(faceCenter, 1.0 / face.indices.size());
            const V3 faceNormal = toV3(face.normal);
            const V3 viewVector = projectionType == ProjectionType::Perspective
                ? sub(cameraPosition, faceCenter) : mul(forward, -1.0);
            if (cullingEnabled && dot(faceNormal, viewVector) <= 0.0) {
                continue;
            }

            if (face.textureCoordinates.size() != face.indices.size()) {
                continue;
            }
            const QVector<QPointF>& faceUvs = face.textureCoordinates;

            QVector<Corner> corners;
            corners.reserve(face.indices.size());
            for (int cornerIndex = 0; cornerIndex < face.indices.size(); ++cornerIndex) {
                const int vertexIndex = face.indices[cornerIndex];
                Corner corner;
                corner.vertex = &vertices[vertexIndex];
                corner.normal = cornerIndex < face.vertexNormals.size()
                    ? toV3(face.vertexNormals[cornerIndex]) : faceNormal;
                corner.uv = faceUvs[cornerIndex];
                const V3 lightDirection = normalized(sub(toV3(lightPosition), corner.vertex->world));
                corner.gouraudColor = lambertColor(objectColor, corner.normal, lightDirection);
                corners.append(corner);
            }

            // Невыпуклые грани требуют отдельной триангуляции; для граней
            // правильных многогранников используется веер, как в исходном проекте.
            for (int k = 1; k + 1 < corners.size(); ++k) {
                const Corner triangle[3] = {corners[0], corners[k], corners[k + 1]};
                if (triangle[0].vertex->screen.x() < -1000.0
                    || triangle[1].vertex->screen.x() < -1000.0
                    || triangle[2].vertex->screen.x() < -1000.0) {
                    continue;
                }

                const double x0 = triangle[0].vertex->screen.x();
                const double y0 = triangle[0].vertex->screen.y();
                const double x1 = triangle[1].vertex->screen.x();
                const double y1 = triangle[1].vertex->screen.y();
                const double x2 = triangle[2].vertex->screen.x();
                const double y2 = triangle[2].vertex->screen.y();
                const double area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
                if (std::abs(area) < 1e-10) {
                    continue;
                }

                const int minPixelX = qBound(0, static_cast<int>(std::floor(std::min({x0, x1, x2}))), renderWidth - 1);
                const int maxPixelX = qBound(0, static_cast<int>(std::ceil(std::max({x0, x1, x2}))), renderWidth - 1);
                const int minPixelY = qBound(0, static_cast<int>(std::floor(std::min({y0, y1, y2}))), renderHeight - 1);
                const int maxPixelY = qBound(0, static_cast<int>(std::ceil(std::max({y0, y1, y2}))), renderHeight - 1);

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

                        const double z0 = triangle[0].vertex->camera.z;
                        const double z1 = triangle[1].vertex->camera.z;
                        const double z2 = triangle[2].vertex->camera.z;
                        const double z = projectionType == ProjectionType::Perspective
                            ? 1.0 / (w0 / z0 + w1 / z1 + w2 / z2)
                            : (w0 * z0 + w1 * z1 + w2 * z2);
                        const int pixelIndex = y * renderWidth + x;
                        if (zBufferEnabled && z >= zBuffer[pixelIndex]) {
                            continue;
                        }

                        double wa = w0;
                        double wb = w1;
                        double wc = w2;
                        if (projectionType == ProjectionType::Perspective) {
                            wa /= z0;
                            wb /= z1;
                            wc /= z2;
                            const double weightSum = wa + wb + wc;
                            if (std::abs(weightSum) < 1e-12) {
                                continue;
                            }
                            wa /= weightSum;
                            wb /= weightSum;
                            wc /= weightSum;
                        }

                        // Барицентрические веса используются для интерполяции
                        // всех атрибутов внутри треугольника. Для перспективной
                        // проекции выше уже рассчитаны perspective-correct веса.
                        const V3 worldPosition = add(
                            add(mul(triangle[0].vertex->world, wa),
                                mul(triangle[1].vertex->world, wb)),
                            mul(triangle[2].vertex->world, wc));

                        V3 normal = add(
                            add(mul(triangle[0].normal, wa),
                                mul(triangle[1].normal, wb)),
                            mul(triangle[2].normal, wc));

                        const QColor gouraudColor(
                            channel(triangle[0].gouraudColor.red() * wa
                                     + triangle[1].gouraudColor.red() * wb
                                     + triangle[2].gouraudColor.red() * wc),
                            channel(triangle[0].gouraudColor.green() * wa
                                     + triangle[1].gouraudColor.green() * wb
                                     + triangle[2].gouraudColor.green() * wc),
                            channel(triangle[0].gouraudColor.blue() * wa
                                     + triangle[1].gouraudColor.blue() * wb
                                     + triangle[2].gouraudColor.blue() * wc));

                        // UV интерполируются по трём вершинам треугольника.
                        const QPointF uv(
                            triangle[0].uv.x() * wa + triangle[1].uv.x() * wb
                                + triangle[2].uv.x() * wc,
                            triangle[0].uv.y() * wa + triangle[1].uv.y() * wb
                                + triangle[2].uv.y() * wc);

                        QColor litColor;
                        const V3 lightDirection = normalized(sub(toV3(lightPosition), worldPosition));
                        if (shadingMode == GouraudLambert) {
                            litColor = gouraudColor;
                        } else {
                            normal = normalized(normal);
                            litColor = toonDiffuseColor(objectColor, normal, lightDirection);
                        }

                        // Получаем цвет одного ближайшего пикселя текстуры.
                        if (textureEnabled && !textureImage.isNull()) {
                            litColor = multiplyColors(litColor,
                                                      sampleTextureNearest(textureImage, uv));
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

    output.drawImage(rect(), image);
    output.setPen(QColor(60, 70, 85));
    const QString shadingName = shadingMode == GouraudLambert
        ? "Гуро (Ламберт)" : "Фонг (Toon shading)";
    output.drawText(12, 20,
                    QString("Камера: азимут %1°, наклон %2°, расстояние %3 | %4%5")
                        .arg(cameraYaw, 0, 'f', 0)
                        .arg(cameraPitch, 0, 'f', 0)
                        .arg(cameraDistance, 0, 'f', 1)
                        .arg(shadingName)
                        .arg(textureEnabled ? " | текстура" : ""));
}

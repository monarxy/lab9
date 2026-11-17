#include "ObjIO.h"
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QRegExp>

bool ObjIO::load(const QString& fileName, Polyhedron& model, QString* error)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) *error = file.errorString();
        return false;
    }

    Polyhedron result;
    QVector<QPointF> textureVertices;
    QTextStream in(&file);
    int lineNumber = 0;

    while (!in.atEnd()) {
        const QString line = in.readLine().trimmed();
        ++lineNumber;
        if (line.isEmpty() || line.startsWith('#')) {
            continue;
        }

        const QStringList parts = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);
        if (parts.isEmpty()) {
            continue;
        }

        if (parts[0] == "v" && parts.size() >= 4) {
            bool okX = false;
            bool okY = false;
            bool okZ = false;
            const double x = parts[1].toDouble(&okX);
            const double y = parts[2].toDouble(&okY);
            const double z = parts[3].toDouble(&okZ);
            if (!okX || !okY || !okZ) {
                if (error) *error = QString("Некорректная вершина в строке %1").arg(lineNumber);
                return false;
            }
            result.vertices.append(Point3D(x, y, z));
        } else if (parts[0] == "vt" && parts.size() >= 3) {
            bool okU = false;
            bool okV = false;
            const double u = parts[1].toDouble(&okU);
            const double v = parts[2].toDouble(&okV);
            if (!okU || !okV) {
                if (error) *error = QString("Некорректная текстурная координата в строке %1").arg(lineNumber);
                return false;
            }
            textureVertices.append(QPointF(u, v));
        } else if (parts[0] == "f" && parts.size() >= 4) {
            QVector<int> indices;
            QVector<QPointF> faceUvs;
            bool hasTextureCoordinates = true;

            for (int i = 1; i < parts.size(); ++i) {
                const QStringList fields = parts[i].split('/');
                bool okVertex = false;
                int vertexIndex = fields.value(0).toInt(&okVertex);
                if (!okVertex || vertexIndex == 0) {
                    if (error) *error = QString("Некорректная грань в строке %1").arg(lineNumber);
                    return false;
                }
                vertexIndex = vertexIndex > 0 ? vertexIndex - 1 : result.vertices.size() + vertexIndex;
                if (vertexIndex < 0 || vertexIndex >= result.vertices.size()) {
                    if (error) *error = QString("Индекс вершины вне диапазона в строке %1").arg(lineNumber);
                    return false;
                }
                indices.append(vertexIndex);

                if (fields.size() > 1 && !fields[1].isEmpty()) {
                    bool okTexture = false;
                    int textureIndex = fields[1].toInt(&okTexture);
                    if (!okTexture || textureIndex == 0) {
                        if (error) *error = QString("Некорректный индекс текстуры в строке %1").arg(lineNumber);
                        return false;
                    }
                    textureIndex = textureIndex > 0 ? textureIndex - 1 : textureVertices.size() + textureIndex;
                    if (textureIndex < 0 || textureIndex >= textureVertices.size()) {
                        if (error) *error = QString("Индекс текстуры вне диапазона в строке %1").arg(lineNumber);
                        return false;
                    }
                    faceUvs.append(textureVertices[textureIndex]);
                } else {
                    hasTextureCoordinates = false;
                }
            }

            Polygon face(indices);
            if (hasTextureCoordinates && faceUvs.size() == indices.size()) {
                face.textureCoordinates = faceUvs;
            }
            result.faces.append(face);
        }
    }

    if (result.vertices.isEmpty() || result.faces.isEmpty()) {
        if (error) *error = "OBJ-файл должен содержать вершины (v) и грани (f).";
        return false;
    }

    result.calculateNormals();
    model = result;
    return true;
}

bool ObjIO::save(const QString& fileName, const Polyhedron& model, QString* error)
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (error) *error = file.errorString();
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::SmartNotation);
    out.setRealNumberPrecision(10);
    out << "# Model exported by PolyhedraQt\n";

    for (const Point3D& point : model.vertices) {
        out << "v " << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }

    // UV-координаты записываются отдельно для каждого угла грани.
    int textureIndex = 1;
    for (const Polygon& face : model.faces) {
        if (face.textureCoordinates.size() != face.indices.size()) {
            continue;
        }
        for (const QPointF& uv : face.textureCoordinates) {
            out << "vt " << uv.x() << ' ' << uv.y() << '\n';
        }
    }

    textureIndex = 1;
    for (const Polygon& face : model.faces) {
        if (face.indices.size() < 3) {
            continue;
        }
        const bool hasUvs = face.textureCoordinates.size() == face.indices.size();
        out << "f";
        for (int corner = 0; corner < face.indices.size(); ++corner) {
            const int vertexIndex = face.indices[corner] + 1;
            if (hasUvs) {
                out << ' ' << vertexIndex << '/' << textureIndex++;
            } else {
                out << ' ' << vertexIndex;
            }
        }
        out << '\n';
    }
    return true;
}

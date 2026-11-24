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
        } else if (parts[0] == "f" && parts.size() >= 4) {
            QVector<int> indices;

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
            }

            result.faces.append(Polygon(indices));
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

    for (const Polygon& face : model.faces) {
        if (face.indices.size() < 3) {
            continue;
        }
        out << "f";
        for (int vertexIndex : face.indices) {
            out << ' ' << vertexIndex + 1;
        }
        out << '\n';
    }
    return true;
}

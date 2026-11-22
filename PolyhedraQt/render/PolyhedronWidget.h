#pragma once

#include <QWidget>
#include <QImage>
#include <QColor>

#include "../geometry/Polyhedron.h"
#include "../math/Projection.h"
#include "Camera.h"

class PolyhedronWidget : public QWidget {
    Q_OBJECT
public:
    enum ShadingMode {
        GouraudLambert,
        PhongToon
    };

    explicit PolyhedronWidget(QWidget* parent = nullptr);

    void setPolyhedron(const Polyhedron& p);
    void setProjectionType(ProjectionType type);
    void setBackFaceCulling(bool enabled);
    void setZBufferEnabled(bool enabled);
    void setCamera(double yawDegrees, double pitchDegrees, double distance);
    void setShadingMode(ShadingMode mode);
    void setLightPosition(const Point3D& position);
    void setObjectColor(const QColor& color);
    void setTexture(const QImage& texture);
    void setTextureEnabled(bool enabled);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    Polyhedron polyhedron;
    ProjectionType projectionType = ProjectionType::Perspective;
    bool cullingEnabled = true;
    bool zBufferEnabled = true;
    bool textureEnabled = false;
    ShadingMode shadingMode = GouraudLambert;
    double cameraYaw = 35.0;
    double cameraPitch = 22.0;
    double cameraDistance = 8.0;
    Point3D lightPosition{4.0, 5.0, 6.0};
    QColor objectColor{95, 160, 210};
    QImage textureImage;
    Camera camera;
};

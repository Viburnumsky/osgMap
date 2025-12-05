#include <osgGA/CameraManipulator>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osg/Matrix>
#include <osg/Vec3d>
#include <osg/BoundingSphere>
#include <osg/Node>
#include <algorithm>

class GoogleMapsManipulator : public osgGA::CameraManipulator {
public:
    GoogleMapsManipulator()
        : _distance(100.0), _lastX(0), _lastY(0), _center(0, 0, 1),
          _isMoving(false), _lastMoveTime(0.0), _movementTimeout(0.2)
    {}

    // ---- NEW PUBLIC API ----
    bool isMoving() const 
    {
        double now = osg::Timer::instance()->time_s();
        return _isMoving && (now - _lastMoveTime < _movementTimeout);
    }

    void setMovementTimeout(double seconds) { _movementTimeout = seconds; }
    // -------------------------

    void resetFromBounds()
    {
        if (_node.valid())
        {
            const osg::BoundingSphere bs = _node->getBound();
            _center = bs.center();
            _distance = bs.radius() * 0.5;
        }
    }

    void setNode(osg::Node* node) override
    {
        _node = node;
        osgGA::CameraManipulator::setNode(node);
        resetFromBounds();
    }
    void home(double) override { resetFromBounds(); }
    void home(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&) override
    {
        resetFromBounds();
    }

    osg::Matrixd getInverseMatrix() const override
    {
        osg::Vec3d eye = _center;
        eye.normalize();
        eye *= (_center.length() + _distance);
        return osg::Matrixd::lookAt(eye, _center, osg::Vec3d(0, 0, 1));
    }

    osg::Matrixd getMatrix() const override
    {
        return osg::Matrixd::inverse(getInverseMatrix());
    }

    void setByMatrix(const osg::Matrixd&) override {}
    void setByInverseMatrix(const osg::Matrixd&) override {}

    bool handle(const osgGA::GUIEventAdapter& ea,
                osgGA::GUIActionAdapter& aa) override
    {
        // Record movement helpers
        auto markMovement = [&]() {
            _isMoving = true;
            _lastMoveTime = osg::Timer::instance()->time_s();
        };

        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
        {
            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();
            markMovement();
            return true;
        }

        if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG
            && (ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
        {
            osg::Vec3d up = _center;
            up.normalize();
            osg::Vec3d east = osg::Vec3d(0, 0, 1) ^ up;
            east.normalize();
            osg::Vec3d north = up ^ east;

            float x = ea.getXnormalized(), y = ea.getYnormalized();
            _center -= (east * (x - _lastX) * _distance)
                + (north * (y - _lastY) * _distance);
            _lastX = x;
            _lastY = y;

            markMovement();
            aa.requestRedraw();
            return true;
        }

        if (ea.getEventType() == osgGA::GUIEventAdapter::SCROLL)
        {
            _distance *=
                (ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP
                     ? 0.8
                     : 1.25);
            _distance = std::max(_distance, 10.0);

            markMovement();
            aa.requestRedraw();
            return true;
        }

        // Home key resets center/distance to node bounds.
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN
            && ea.getKey() == osgGA::GUIEventAdapter::KEY_Home)
        {
            resetFromBounds();
            markMovement();
            aa.requestRedraw();
            return true;
        }

        return false;
    }

private:
    osg::observer_ptr<osg::Node> _node;
    osg::Vec3d _center;
    double _distance;
    float _lastX, _lastY;

    // ---- NEW FIELDS ----
    bool _isMoving;
    double _lastMoveTime;
    double _movementTimeout;
    // ---------------------
};

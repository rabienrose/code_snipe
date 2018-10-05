//
// Created by user on 2/24/18.
//

#ifndef PROJECT_IMAGE_WIDGET_H
#define PROJECT_IMAGE_WIDGET_H
#include <QWidget>
#include <QLabel>

namespace rviz_plugin_tutorials
{
//
// For maximum reusability, this class is only responsible for user
// interaction and display inside its widget.  It does not make any
// ROS or RViz calls.  It communicates its data to the outside just
// via Qt signals.
    class ImageWidget: public QWidget
    {
        Q_OBJECT
    public:
        // This class is not instantiated by pluginlib::ClassLoader, so the
        // constructor has no restrictions.
        ImageWidget( QWidget* parent = 0 );

        QLabel *labelLocImage;

        // We override QWidget::paintEvent() to do custom painting.
        // virtual void paintEvent( QPaintEvent* event );

        // We override the mouse events and leaveEvent() to keep track of
        // what the mouse is doing.
        virtual void mouseMoveEvent( QMouseEvent* event );
        virtual void mousePressEvent( QMouseEvent* event );
        virtual void mouseReleaseEvent( QMouseEvent* event );
        virtual void leaveEvent( QEvent* event );

        // Override sizeHint() to give the layout managers some idea of a
        // good size for this.
        virtual QSize sizeHint() const { return QSize( 640, 400 ); }
    };
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials
#endif //PROJECT_IMAGE_WIDGET_H

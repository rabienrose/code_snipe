//
// Created by user on 2/24/18.
//

#include <stdio.h>
#include <math.h>

#include <QPainter>
#include <QMouseEvent>

#include "image_widget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLayout>

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// The ImageWidget constructor does the normal Qt thing of
// passing the parent widget to the superclass constructor, then
// initializing the member variables.
    ImageWidget::ImageWidget( QWidget* parent )
            : QWidget( parent )
    {
        QHBoxLayout *layout = new QHBoxLayout;
        labelLocImage = new QLabel;

        labelLocImage->setStyleSheet("background-color:grey");
        labelLocImage->setText("No Image");
        //labelLocImage->setFrameStyle(QFrame::Box | QFrame::Sunken);
        //labelLocImage->setLineWidth(1);


        QFont ft;
        ft.setPointSize(48);
        //ft.setStyle(QFont::StyleItalic);
        labelLocImage->setFont(ft);
        labelLocImage->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

        layout->addWidget(labelLocImage);
        setLayout(layout);
    }

// This paintEvent() is complex because of the drawing of the two
// arc-arrows representing wheel motion.  It is not particularly
// relevant to learning how to make an RViz plugin, so I will kind of
// skim it.
//    void ImageWidget::paintEvent( QPaintEvent* event )
//    {
//        //
//    }


    void ImageWidget::mouseMoveEvent( QMouseEvent* event )
    {
        //
    }

    void ImageWidget::mousePressEvent( QMouseEvent* event )
    {
        //
    }

// When the mouse leaves the widget but the button is still held down,
// we don't get the leaveEvent() because the mouse is "grabbed" (by
// default from Qt).  However, when the mouse drags out of the widget
// and then other buttons are pressed (or possibly other
// window-manager things happen), we will get a leaveEvent() but not a
// mouseReleaseEvent().  Without catching this event you can have a
// robot stuck "on" without the user controlling it.
    void ImageWidget::leaveEvent( QEvent* event )
    {
        //
    }

    void ImageWidget::mouseReleaseEvent( QMouseEvent* event )
    {
        //
    }
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

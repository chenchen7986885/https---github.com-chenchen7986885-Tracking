#include "imageview.h"

ImageView::ImageView(QWidget *parent) :
    QLabel(parent)
{
}

void ImageView::mouseMoveEvent(QMouseEvent *event)
{
    this->x = event->x();
    this->y = event->y();
    Mouse_Pos();
}

void ImageView::mousePressEvent(QMouseEvent *event)
{
    this->x = event->x();
    this->y = event->y();
    Mouse_Pressed();
}

void ImageView::leaveEvent(QEvent *)
{
    Mouse_Left();
}

void ImageView::mouseReleaseEvent(QMouseEvent *ev)
{
    this->x = ev->x();
    this->y = ev->y();
    Mouse_Left();
}

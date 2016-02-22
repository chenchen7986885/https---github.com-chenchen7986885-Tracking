#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#include <QMouseEvent>
#include <QLabel>
#include <QEvent>
#include <QDebug>

class ImageView : public QLabel
{
    Q_OBJECT
public:
    explicit ImageView(QWidget *parent = 0);

    void mouseMoveEvent(QMouseEvent* event);

    void mousePressEvent(QMouseEvent *event);

    void leaveEvent(QEvent*);

    void mouseReleaseEvent(QMouseEvent *ev);

    int x, y;

signals:
    void Mouse_Pressed();

    void Mouse_Pos();

    void Mouse_Left();

public slots:

};

#endif // IMAGEVIEW_H

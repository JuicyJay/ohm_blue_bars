/*
 * Node.cpp
 *
 *  Created on: 16.10.2014
 *      Author: chris
 */


#include "Node.h"
#include "Link.h"


// qt includes
#include <QColor>
#include <QFontMetricsF>
#include <QPainter>
#include <QStyle>
#include <QStyleOptionGraphicsItem>
#include <QApplication>
#include <QGraphicsEllipseItem>

#include <QMessageBox>
#include <QGraphicsSceneMouseEvent>

#include <QDebug>


Node::Node(QPointF pos)
: QGraphicsItem()
{
   _textColor        = Qt::darkGreen;
   _outlineColor     = Qt::darkBlue;
   _backgroundColor  = Qt::white;

   setPos(pos);

   setFlags(ItemIsMovable | ItemIsSelectable);

   setAcceptHoverEvents(true);
}


Node::~Node(void)
{
   Q_FOREACH(Link *link, _links)
       delete link;
}

QPainterPath Node::shape() const
{
    QRectF rect = outlineRect();

    QPainterPath path;
    path.addRoundRect(rect, roundness(rect.width()),
                            roundness(rect.height()));
    return path;
}




void Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */)
{
    QPen pen(Qt::blue);
    if (option->state & QStyle::State_Selected) {
        pen.setStyle(Qt::DotLine);
        pen.setWidth(2);
    }
    painter->setPen(pen);
    painter->setBrush(_backgroundColor);

    QRectF rect = outlineRect();
    painter->drawRoundRect(rect, roundness(rect.width()), roundness(rect.height()));

    painter->setPen(_textColor);
    painter->drawText(rect, Qt::AlignCenter, _name);

//    // Sum up all forces pushing this item away
//    qreal xvel = 0;
//    qreal yvel = 0;
//    Q_FOREACH (QGraphicsItem *item, scene()->items()) {
//        Node *node = qgraphicsitem_cast<Node *>(item);
//        if (!node)
//            continue;
//
//        QLineF line(mapFromItem(node, 0, 0), QPointF(0, 0));
//        qreal dx = line.dx();
//        qreal dy = line.dy();
//        double l = 2.0 * (dx * dx + dy * dy);
//        if (l > 0) {
//            xvel += (dx * 150.0) / l;
//            yvel += (dy * 150.0) / l;
//        }
//    }
//
//    // Now subtract all forces pulling items together
//    double weight = (_links.size() + 1) * 10;
//    Q_FOREACH(Link *link, _links) {
//        QPointF pos;
//        if (link->fromNode() == this)  pos = mapFromItem(link->toNode(), 0, 0);
//        else                           pos = mapFromItem(link->fromNode(), 0, 0);
//        xvel += pos.x() / weight * 3.3;
//        yvel += pos.y() / weight * 3.3;
//    }
//
//    if (qAbs(xvel) < 0.1 && qAbs(yvel) < 0.1) xvel = yvel = 0;
//
//    QRectF sceneRect = scene()->sceneRect();
//    newPos = pos() + QPointF(xvel, yvel);
//    newPos.setX(qMin(qMax(newPos.x(), sceneRect.left() + 10), sceneRect.right() - 10));
//    newPos.setY(qMin(qMax(newPos.y(), sceneRect.top() + 10), sceneRect.bottom() - 10));
}



QRectF Node::boundingRect(void) const
{
   const int Margin = 1;
   return outlineRect().adjusted(-Margin, -Margin, +Margin, +Margin);
}

void Node::addLink(Link *link)
{
    _links.insert(link);
}

void Node::removeLink(Link *link)
{
   _links.remove(link);
}

void Node::setText(const QString &text)
{
    prepareGeometryChange();
    _name = text;
    update();
}

/* PROTECTED */
QVariant Node::itemChange(GraphicsItemChange change,
                          const QVariant &value)
{
    if (change == ItemPositionChange /*ItemPositionHasChanged*/ ) {
        Q_FOREACH(Link *link, _links)
            link->update();
    }
    return QGraphicsItem::itemChange(change, value);
}

void Node::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
   update();
   QMessageBox msgBox;
   msgBox.setWindowTitle("Force State");
   msgBox.setText("You want to force the state " + _name);
   msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
   msgBox.setDefaultButton(QMessageBox::No);

   if(msgBox.exec() == QMessageBox::Yes)
      emit force(_name);
}


void Node::mouseMoveEvent(QMouseEvent* event)
{
   QRectF rect = outlineRect();
   _backgroundColor = Qt::red;

   qDebug() << __PRETTY_FUNCTION__;
}


//void Node::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
//{
//   _backgroundColor = Qt::red;
//   update();
//}
//
//void Node::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
//{
//   _backgroundColor = Qt::white;
//   update();
//}


/* PRIVATE*/
QRectF Node::outlineRect() const
{
    const int Padding = 8;
    QFontMetricsF metrics = qApp->fontMetrics();
    QRectF rect = metrics.boundingRect(_name);
    rect.setHeight(60.0);
    rect.setWidth(60.0);
    rect.adjust(-Padding, -Padding, +Padding, +Padding);
    rect.translate(-rect.center());
    return rect;
}

int Node::roundness(double size) const
{
    const int Diameter = 12;
    return 100 * Diameter / int(size);
}

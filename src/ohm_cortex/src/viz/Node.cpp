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

#include <QInputDialog>
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
    QString text = QInputDialog::getText(event->widget(),
                                         tr("Edit Text"), tr("Enter new text:"),
                                         QLineEdit::Normal, _name);
    if (!text.isEmpty())
        setText(text);
}


void Node::mouseMoveEvent(QMouseEvent* event)
{
   QRectF rect = outlineRect();
   _backgroundColor = Qt::red;

   qDebug() << __PRETTY_FUNCTION__;
}


void Node::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
   _backgroundColor = Qt::red;
   update();
}

void Node::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
   _backgroundColor = Qt::white;
   update();
}


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

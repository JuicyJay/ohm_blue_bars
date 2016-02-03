/*
 * Link.cpp
 *
 *  Created on: 16.10.2014
 *      Author: chris
 */

#include "Link.h"
#include "Node.h"

#include <QPen>
#include <QPainter>
#include <QDebug>

#include <cmath>



using namespace std;

Link::Link(Node *fromNode, Node *toNode)
: QGraphicsItem()
{
   _from  = fromNode;
   _to    = toNode;

   qDebug() << "POS FROM: " << _from->pos();

   _from->addLink(this);
   _to->addLink(this);

   setFlags(QGraphicsItem::ItemIsSelectable);
   setZValue(-1);

   trackNodes();
}

Link::~Link()
{
   _from->removeLink(this);
   _to->removeLink(this);
}


void Link::trackNodes()
{
//    setLine(QLineF(_from->pos(), _to->pos()));

    QLineF line(mapFromItem(_from, 0, 0), mapFromItem(_to, 0, 0));
    qreal length = line.length();

    prepareGeometryChange();

    if (length > qreal(20.)) {
        QPointF edgeOffset((line.dx() * 10) / length, (line.dy() * 10) / length);
        sourcePoint = line.p1() + edgeOffset;
        destPoint   = line.p2() - edgeOffset;
    } else {
        sourcePoint = destPoint = line.p1();
    }
}


void Link::adjust(void)
{
   QLineF line(mapFromItem(_from, 0, 0), mapFromItem(_to, 0, 0));
   qreal length = line.length();

   prepareGeometryChange();

   if (length > qreal(20.)) {
       QPointF edgeOffset((line.dx() * 10) / length, (line.dy() * 10) / length);
       sourcePoint = line.p1() + edgeOffset;
       destPoint   = line.p2() - edgeOffset;
   } else {
       sourcePoint = destPoint = line.p1();
   }
}

QRectF Link::boundingRect() const
{
    qreal penWidth = 1;
    qreal extra = (penWidth + 12.0) / 2.0;

    return QRectF(sourcePoint, QSizeF(destPoint.x() - sourcePoint.x(),
                                      destPoint.y() - sourcePoint.y()))
        .normalized()
        .adjusted(-extra, -extra, extra, extra);
}



void Link::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
   qreal arrowSize = 12.0;

   QLineF line(_from->pos(), _to->pos());

   painter->setPen(QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
   painter->drawLine(line);

   // Draw the arrows
   double angle = acos(line.dx() / line.length());
   if (line.dy() >= 0)
      angle = 3.14*2 - angle;

   QPointF sourceArrowP1 = _from->pos() + QPointF(sin(angle + M_PI / 3)        * arrowSize,
                                                   cos(angle + M_PI / 3)        * arrowSize);
   QPointF sourceArrowP2 = _from->pos() + QPointF(sin(angle + M_PI - M_PI/ 3)  * arrowSize,
                                                   cos(angle + M_PI - M_PI/ 3)  * arrowSize);
   QPointF destArrowP1 = _to->pos()     + QPointF(sin(angle - M_PI / 3)        * arrowSize,
                                                   cos(angle - M_PI / 3)        * arrowSize);
   QPointF destArrowP2 = _to->pos()     + QPointF(sin(angle - M_PI + M_PI / 3) * arrowSize,
                                                   cos(angle - M_PI + M_PI / 3) * arrowSize);


   painter->setBrush(Qt::black);
   painter->drawPolygon(QPolygonF() << line.p1() << sourceArrowP1 << sourceArrowP2);
   painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1   << destArrowP2);
}

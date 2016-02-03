/*
 * Link.h
 *
 *  Created on: 16.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_VIZ_LINK_H_
#define OHM_CORTEX_SRC_VIZ_LINK_H_

#include <QGraphicsLineItem>

class Node;

class Link : public QObject, public QGraphicsItem
{
public:
   Link(Node *fromNode, Node *toNode);

   virtual ~Link();

   Node *fromNode() const;

   Node *toNode() const;

   QColor color() const;

   QRectF boundingRect() const;

   void trackNodes();

   void adjust(void);

   void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *);


protected:

private:

   Node* _from;
   Node* _to;

   QPointF sourcePoint;
   QPointF destPoint;
};

#endif /* OHM_CORTEX_SRC_VIZ_LINK_H_ */

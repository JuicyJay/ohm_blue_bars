/*
 * Node.h
 *
 *  Created on: 16.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_VIZ_UI_NODE_H_
#define OHM_CORTEX_SRC_VIZ_UI_NODE_H_


#include <QGraphicsItem>
#include <QSet>
#include <QMouseEvent>
#include <QPointF>

class Link;

class Node : public QObject, public QGraphicsItem
{
//   Q_DECLARE_TR_FUNCTIONS(Node);
   Q_OBJECT
public:
   Node(QPointF pos = QPointF(0.0, 0.0));

   ~Node(void);

   QRectF boundingRect() const;

   QPainterPath shape() const;

   void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

   void addLink(Link *link);

   void removeLink(Link *link);

   QString getText(void) const { return _name; }


   // SETTERS
   void setText(const QString& text);

   void setBackgroundColor(const QColor& color) { _backgroundColor = color; }

   void setTextColor(const QColor& color)       { _textColor       = color; }

protected:
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
//    void hoverEnterEvent(QGraphicsSceneHoverEvent* event);
//    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);

    void mouseMoveEvent(QMouseEvent* event);



    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

signals:
   void force(QString state);

private:

    QRectF outlineRect() const;

    int roundness(double size) const;

    QSet<Link*> _links;
    QString     _name;
    QColor      _textColor;
    QColor      _backgroundColor;
    QColor      _outlineColor;
};

#endif /* OHM_CORTEX_SRC_VIZ_UI_NODE_H_ */

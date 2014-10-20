/*
 * StateViz.h
 *
 *  Created on: 20.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_VIZ_STATEVIZ_H_
#define OHM_CORTEX_SRC_VIZ_STATEVIZ_H_

#include <QString>
#include <QVector>

#include "../viz/Node.h"

#include <iostream>

class StateViz
{
public:
   /**
    * Default constructor
    */
   StateViz(void);
   /**
    * Default destructor
    */
   virtual ~StateViz(void);


   // SETTERS
   void setId(const unsigned int& id) {   _id = id;   }

   void setName(const QString& name)
   {
      QString tmp = name;
      if(tmp.contains("cpp"))
         tmp.replace(".cpp", "");
      _name = tmp;
   }

   void setNode(Node* node)         { _node = node; }

   // GETTERS
   unsigned int getId(void) const         { return _id; }

   QString getName(void) const            { return _name; }

   QVector<StateViz> getLinks(void) const { return _links; }

   void addLink(const StateViz& s)
   {
      _links.push_back(s);
   }

   Node* getNode(void) const { return _node; }


   // OPERATORS
   bool operator ==(const StateViz& s){
      if(_id == s.getId() && _name == s.getName()) return true;
      else                                         return false;
   }

   friend std::ostream& operator << (std::ostream& stream, const StateViz&  p)
   {
//     stream << "name: " << p.getName() << "\t id: " << p.getId(); // << std::endl;
//     stream << "|" << std::endl;
//
//     for(unsigned int i=0 ; i<_links.size() ; ++i)
//            stream << "---> " << _links[i] << std::endl;

     return stream;
   }
private:
   unsigned int      _id;
   QString           _name;
   QVector<StateViz> _links;

   Node*             _node;



};

#endif /* OHM_CORTEX_SRC_VIZ_STATEVIZ_H_ */

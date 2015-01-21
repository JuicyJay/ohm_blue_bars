/*
 * Astar.cpp
 *
 *  Created on: 15.12.2014
 *      Author: m1ch1
 */

#include "Astar_dt.h"

#include "../PathFind_base/PathFind_base.h"

namespace{

}

namespace apps
{

Astar_dt::Astar_dt(GridMap* map) : PathFind_base::PathFind_base(map)
{
   // TODO Auto-generated constructor stub
   //_currentCosts = 0;
   _wall_value = 255; //default
   _cost_short_step = 1;
   _cost_long_step = _cost_short_step * ::sqrt(2);
   _factor_dist = 1;
}

Astar_dt::~Astar_dt()
{
   // TODO Auto-generated destructor stub
}

//todo delete open and closed list;
std::vector<Pixel> Astar_dt::computePath(Pixel start, Pixel end)
{
   //spwap start and end
   _st = end;
   _en = start;

   std::priority_queue<Astar_node*, std::deque<Astar_node*>, Astar_node_comp> openlist; //contains index of openContainer
   std::map<unsigned int, Astar_node*> openlistMap;
   std::map<unsigned int, Astar_node*> closedlist;

   //bin img
   //this->binarize(WALL_THRESHOLD, WALL, WALL, FREE);
   //inflate
   //this->inflate(WALL, this->getRobotRadius());

   unsigned int width = _map->getWidth();
   unsigned int height = _map->getHeight();
   uint8_t* mapData = _map->getMapData();

   //add root node
   Astar_node* currNode = new Astar_node;
   Astar_node* endNode = new Astar_node;
   endNode->pixel = _en;
   endNode->parent = NULL;

   currNode->idx = this->pixelToIdx(_st, width);
   currNode->isDiagonal = false;
   currNode->isOverritten = false;
   currNode->pixel = _st;
   currNode->parent = NULL;
   currNode->g = 0;//_currentCosts;
   currNode->f = this->getHx(_st,_en);// + _currentCosts; //f(x) = h(x) + g(x) ... heuristic + current cost

   bool astar_rdy = false;

   while(!astar_rdy)
   {
      //prove target reached
      if(currNode->pixel.x == _en.x && currNode->pixel.y == _en.y)
      {//astar rdy
         astar_rdy = true; //todo check if thats all to do if finish
         endNode->parent = currNode;
         break;
      }

      std::vector<Astar_child> childs;
      this->getChildNodes(childs, currNode->pixel);
      //check children
      for(unsigned int i = 0; i < childs.size(); ++i)
      {
         unsigned int idx = this->pixelToIdx(childs[i].pixel,width);

         if(mapData[idx] == _wall_value)   //prove wall
         {
            //nop
         }
         else if(!(closedlist.find(idx) == closedlist.end())) //prove closed List //todo with extra allocated map and set there info for open and closed list
         {
            //nop
         }
         else if(!(openlistMap.find(idx) == openlistMap.end())) //prove openlist //todo better way see closed list todo maybee adde at first if clause
         {//update f(x) and parent ptr
            //--
            // problem: if change openlist f then openlist musst be new sorted
            //--

            //workaround.. mark old queue element as overritten  and insert new... prove to isOverritten flag when call xx.top();

            astar_f_type g = this->getGx(currNode, childs[i]);
            Astar_node* tmp = openlistMap.at(idx);

            if(tmp->g > g)
            {
               Astar_node* node = new Astar_node;
               tmp->isOverritten = true;
               node->f = this->getHx(childs[i].pixel, _en) + g;//this->updateFx(childs[i]);
               node->g = g;
               node->isDiagonal = childs[i].isDiagonal;
               node->isOverritten = false;
               node->parent = currNode;
               node->pixel = tmp->pixel;
               node->idx = tmp->idx;
               openlist.push(node);

               //change in map
               openlistMap.at(idx) = node;
            }
         }
         else  //not explored
         {//add to openlist with f(x) ptr to parent
            Astar_node* node = new Astar_node;
            astar_f_type g = this->getGx(currNode, childs[i]);
            node->f = this->getHx(childs[i].pixel, _en) + g;//this->updateFx(childs[i]);
            node->g = g;
            node->idx = idx;
            node->isDiagonal = childs[i].isDiagonal;
            node->isOverritten = false;
            node->pixel = childs[i].pixel;
            node->parent = currNode;
            openlist.push(node);
            openlistMap.insert(std::pair<unsigned int, Astar_node*>(idx,node));
         }
      }



      // add curr node to close list
      closedlist.insert(std::pair<unsigned int, Astar_node*>(currNode->idx, currNode));

      //choose new currNode from openlist and delete from openlist
      if(!openlist.empty())
      {
         do{
            currNode = openlist.top();
            openlist.pop();
         }while(currNode->isOverritten && !openlist.empty());

         openlistMap.erase(currNode->idx);
      }

      if(openlist.empty())
      {//no path found
         //return std::vector<Point2D>(0);
         return std::vector<Pixel>(0);
      }

   }

   _path.clear();
   if(astar_rdy)
   {//trace back path
      this->tracePath(endNode);
   }

   //todo delete all nodes :D

   return _path;
}

void Astar_dt::getChildNodes(std::vector<Astar_child>& vec, Pixel node)
{
   //todo evtl Hint for branch predicter
   if((int)node.x > 0 && (int)node.y > 0 && node.x < (_map->getWidth()-1) && node.y < (_map->getHeight()-1))
   {//all nodes around
      vec.resize(MAX_NUM_CHILD);
      vec[0] = (Astar_child(Pixel(node.x-1, node.y-1), true));
      vec[1] = (Astar_child(Pixel(node.x,   node.y-1), false));
      vec[2] = (Astar_child(Pixel(node.x+1, node.y-1), true));
      vec[3] = (Astar_child(Pixel(node.x-1, node.y  ), false));
      vec[4] = (Astar_child(Pixel(node.x+1, node.y  ), false));
      vec[5] = (Astar_child(Pixel(node.x-1, node.y+1), true));
      vec[6] = (Astar_child(Pixel(node.x,   node.y+1), false));
      vec[7] = (Astar_child(Pixel(node.x+1, node.y+1), true));
   }
   else
   {
      std::vector<Astar_child> tmp_vec(8);

      tmp_vec[0] = (Astar_child(Pixel(node.x-1, node.y-1), true));
      tmp_vec[1] = (Astar_child(Pixel(node.x,   node.y-1), false));
      tmp_vec[2] = (Astar_child(Pixel(node.x+1, node.y-1), true));
      tmp_vec[3] = (Astar_child(Pixel(node.x-1, node.y  ), false));
      tmp_vec[4] = (Astar_child(Pixel(node.x+1, node.y  ), false));
      tmp_vec[5] = (Astar_child(Pixel(node.x-1, node.y+1), true));
      tmp_vec[6] = (Astar_child(Pixel(node.x,   node.y+1), false));
      tmp_vec[7] = (Astar_child(Pixel(node.x+1, node.y+1), true));

      vec.clear();

      for(unsigned int i = 0; i < MAX_NUM_CHILD; ++i)
      {
         Astar_child tmp = tmp_vec[i];
         if((int)tmp.pixel.x >= 0 && (int)tmp.pixel.y >= 0 && tmp.pixel.x <= (_map->getWidth()-1) && tmp.pixel.y <= (_map->getHeight()-1))
         {
            vec.push_back(tmp_vec[i]);
         }
      }
      tmp_vec.clear();
   }
}

void Astar_dt::tracePath(Astar_node* node)
{
   if(node == NULL)
   {
      return;
   }
   _path.push_back(node->pixel);
   this->tracePath(node->parent);
}


} /* namespace apps */



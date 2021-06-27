#include <iostream>
#include "DetectorState.h"
#include "JsonHelper.h"
#include "Box.h"
#include "Quadtree.h"

using namespace std;
struct Node
{
    Box box;
};
int main() {
    // read json from disk, parse it and store into unorderedmap
    JsonHelper jsonHelper;
    jsonHelper.readSingleJson("/home/zcc/Code/TestData/0-baseline.json");
    //unordered_map<int, vector<DetectorState>> res;
    vector<DetectorState> res;
    //将所有数据存到res中
    jsonHelper.parseJsonForMiris(res);

    int vec_len = res.size();
    cout<<vec_len<<endl;
    auto nodes = std::vector<Node>();
    for(int i=0;i<vec_len;i++){
        Node node;
        node.box.left = res[i].left;
        node.box.top = res[i].top;
        node.box.width = 0;
        node.box.height = 0;
        node.box.track_id = res[i].track_id;
       // cout<<res[i].track_id<<endl;
        node.box.frameIndex = res[i].frameIndex;
        nodes.push_back(node);
    }

    auto box = Box(0.0, 0.0, 2048.0, 2048.0);
    auto getBox = [](Node* node)
    {
        return node->box;
    };
    auto quadtree = Quadtree<Node*, decltype(getBox)>(box, getBox);
    for (auto& node : nodes){
        quadtree.add(&node);
    }
    auto box1 = Box(550.0, 400.0, 150.0, 150.0);
    Point p = Point(550.0,550.0);
    vector<Point>hull;
    hull.push_back(p);
    p = Point(700.0,550.0);
    hull.push_back(p);
    p = Point(700.0,400.0);
    hull.push_back(p);
    p = Point(550.0,400.0);
    hull.push_back(p);
   auto values1= quadtree.query(hull);
   auto values= quadtree.query(box1);
   cout<< values.size()<<endl;
  /* int xa=0;
   for(int j=0;j<res.size();j++){
        if(res[j].left==571&&res[j].top==435){
            xa++;
        }

   }
   cout<<xa<<" !!"<<endl;*/
   int numm=0;
   for(int i=0;i<values.size();i++){
       //cout<<values[i]->box.left<<" "<< values[i]->box.top <<" "<<  values[i]->box.getRight() <<" "<<values[i]->box.getBottom()<<" "<<values[i]->box.frameIndex<<endl;
       if(values[i]->box.top<=box1.getBottom()&&values[i]->box.left>=box1.getRight()&&values[i]->box.getRight()<=box1.getRight()&&values[i]->box.getBottom()>=box1.getBottom()){
           numm++;
       }
   }
    cout<<numm<<endl;
    values= quadtree.query(hull);
    return 0;
}

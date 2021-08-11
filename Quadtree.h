//
// Created by zcc on 2021/6/23.
//

#ifndef QUADTRANS_QUADTREE_H
#define QUADTRANS_QUADTREE_H

#include <memory>
#include <vector>
#include <cassert>
#include <algorithm>
#include "Box.h"
#include "Point.h"
#include<cmath>
#include<algorithm>
#include "Point.h"
#include "B+ tree.h"
#include <vector>
const double eps = 1e-10;
template<typename T,typename GetBox, typename Equal = std::equal_to<T>>
class Quadtree{

public:
    Quadtree(const Box& box, const GetBox& getBox = GetBox(),
             const Equal& equal = Equal()) :
            mBox(box), mRoot(std::make_unique<Node>()), mGetBox(getBox), mEqual(equal)
    {

    }

    void add(const T& value)
    {
        add(mRoot.get(), 0, mBox, value);
    }

    void remove(const T& value)
    {
        remove(mRoot.get(), nullptr, mBox, value);
    }

    std::vector<T> query(const Box& box) const
    {
        auto values = std::vector<T>();
        query(mRoot.get(), mBox, box, values );
        return values;
    }

    std::vector<T> query(vector<Point> hull) const{
        auto values = std::vector<T>();
        query(mRoot.get(), mBox, hull, values);
        return values;
    }

    void  bianli()
    {
        int anss=0;
        vector<int>vec;
        query1(mRoot.get(),anss,vec);
        cout<<anss<<" anss "<<endl;
        cout<<vec.size()<<endl;
        sort(vec.begin(),vec.end());
        for(int i=anss-1;i>=anss-2000;i--){
            cout<<vec[i]<<" ";
        }
        cout<<endl;
    }



    std::vector<T> query_time(vector<Point> hull,int st,int ed) const{
        auto values = std::vector<T>();
        query(mRoot.get(), mBox, hull, values, st , ed);
        return values;
    }

    void query_stop_pointer(vector<Point> hull1 ,int st,int ed) const{
        auto values = std::vector<pair<Node*,pair<int,int>>>();
        query_pointer(mRoot.get(), mBox, hull1, values, st , ed);
        int len1 = values.size();
        unordered_set<int>answer;
        for(int i=0;i<len1;i++){
            while(values[i].second.first<=values[i].second.second && values[i].first->values[values[i].second.first]->box.frameIndex+45<values[i].first->values[values[i].second.second]->box.frameIndex){
                int l=values[i].second.first,r=values[i].second.second;
                int tmp= values[i].first->values[values[i].second.first]->box.frameIndex+45;
                int res_st =l;
                while(l<=r){
                    int mid=l+((r-l)>>1);
                    int res_time=values[i].first->values[mid]->box.frameIndex;
                    if(res_time < tmp)
                        l=mid+1;
                    else
                        r=mid-1;
                }
                res_st=l;
                if(res_st >values[i].second.second || values[i].first->values[res_st]->box.frameIndex<tmp)
                    break;
                unordered_map<int,int>hash;
                while(values[i].second.first<=values[i].second.second && values[i].first->values[values[i].second.first]->box.frameIndex==tmp-45){
                    hash[values[i].first->values[values[i].second.first]->box.track_id]=1;
                    values[i].second.first++;
                }
                for(int j=res_st;j<=values[i].second.second;j++){
                    if(tmp!=values[i].first->values[j]->box.frameIndex)
                        break;
                    if(hash[values[i].first->values[j]->box.track_id]==1){
                        answer.insert(values[i].first->values[j]->box.track_id);
                    }
                }
                hash.clear();
            }

        }
cout<<" !!!!!!!!!!!! ";
        for(auto it=answer.begin();it!=answer.end();it++){
            cout<< *it<<endl;
        }
    }

    void query_stop_pointer2(vector<Point> hull1 ,int st,int ed) const{
        auto values = std::vector<pair<Node*,pair<int,int>>>();
        query_pointer(mRoot.get(), mBox, hull1, values, st , ed);
        int len1 = values.size();
        unordered_set<int>answer;
        int tag=false;
        for(int i=0;i<len1;i++){
            if(values[i].second.first<=values[i].second.second && values[i].first->values[values[i].second.first]->box.frameIndex+100<values[i].first->values[values[i].second.second]->box.frameIndex ){
                tag=true;
                break;
            }
        }
        while(tag){
            int tmp=0;
            int min_time=0x3f3f3f3f,min_loc;
            for(int i=0;i<len1;i++){

                if(values[i].second.first<=values[i].second.second){
                    min_time=values[i].first->values[values[i].second.first]->box.frameIndex;
                    min_loc=i;
                    tmp=i;
                    break;
                }
            }

            for(int i=tmp+1;i<len1;i++){
                if(values[i].second.first<=values[i].second.second){
                    if(min_time>values[i].first->values[values[i].second.first]->box.frameIndex){
                        min_time=values[i].first->values[values[i].second.first]->box.frameIndex;
                        min_loc=i;
                    }
                }
            }
            unordered_map<int,int>hash;
            for(int i=0;i<len1;i++){
                while(values[i].second.first<=values[i].second.second && values[i].first->values[values[i].second.first]->box.frameIndex == min_time){
                    hash[values[i].first->values[values[i].second.first]->box.track_id]=1;
                    values[i].second.first++;
                }
            }


            for(int i=0;i<len1;i++){
                int l=values[i].second.first,r=values[i].second.second;
                int res_st =l;
                while(l<=r){
                    int mid=l+((r-l)>>1);
                    int res_time=values[i].first->values[mid]->box.frameIndex;
                    if(res_time < min_time+100)
                        l=mid+1;
                    else
                        r=mid-1;
                }
                res_st=l;
                if(res_st >values[i].second.second || values[i].first->values[res_st]->box.frameIndex<min_time+100){
                    continue;
                }
                for(int j=res_st;j<=values[i].second.second;j++){
                    if(min_time+100 !=values[i].first->values[j]->box.frameIndex)
                        break;
                    if(hash[values[i].first->values[j]->box.track_id]==1){
                        answer.insert(values[i].first->values[j]->box.track_id);
                    }
                }

            }
            hash.clear();
            tag=false;
            for(int i=0;i<len1;i++){
                if(values[i].second.first<=values[i].second.second && values[i].first->values[values[i].second.second]->box.frameIndex > min_time + 100){
                    tag=true;
                    break;
                }
            }
        }
        cout<<"???????"<<endl;
        for(auto it = answer.begin();it!=answer.end();it++){
            cout<<*it<<endl;
        }
    }

    void query_time_pointer(vector<Point> hull1,vector<Point> hull2 ,int st,int ed) const{
        auto values1 = std::vector<pair<Node*,pair<int,int>>>();
        query_pointer(mRoot.get(), mBox, hull1, values1, st , ed);
        auto values2 = std::vector<pair<Node*,pair<int,int>>>();
        query_pointer(mRoot.get(), mBox, hull2, values2, st , ed);

        int len1 = values1.size(),len2 = values2.size();

        for(int i=0;i<len1;i++){
            sort(values1[i].first->values.begin()+values1[i].second.first,values1[i].first->values.begin()+values1[i].second.second+1);
        }

        for(int i=0;i<len2;i++){
            sort(values2[i].first->values.begin()+values2[i].second.first,values2[i].first->values.begin()+values2[i].second.second+1);
        }

        int tag1= false,tag2= false;
        for(int i=0;i<len1;i++){
            if(values1[i].second.first<=values1[i].second.second){
                tag1=true;
                break;
            }
        }
        for(int i=0;i<len2;i++){
            if(values2[i].second.first<=values2[i].second.second) {
                tag2=true;
                break;
            }
        }
        unordered_set<int>answer1_2,answer2_1;
        int cnt=0;
        while(tag1&&tag2){
            int tmp=0;
            int min_time,min_carId=0x3f3f3f3f,min_loc;
            for(int i=0;i<len1;i++){

                if(values1[i].second.first<=values1[i].second.second){
                    min_time=values1[i].first->values[values1[i].second.first]->box.frameIndex;
                    min_carId=values1[i].first->values[values1[i].second.first]->box.track_id;
                    min_loc=i;
                    tmp=i;
                    break;
                }
            }

            for(int i=tmp+1;i<len1;i++){
                if(values1[i].second.first<=values1[i].second.second){
                    if(min_carId>values1[i].first->values[values1[i].second.first]->box.track_id){
                        min_time=values1[i].first->values[values1[i].second.first]->box.frameIndex;
                        min_carId=values1[i].first->values[values1[i].second.first]->box.track_id;
                        min_loc=i;
                    }
                }
            }

            int min_time2,min_carId2=0x3f3f3f3f,min_loc2;
            for(int i=0;i<len2;i++){
                if(values2[i].second.first<=values2[i].second.second){
                    min_time2=values2[i].first->values[values2[i].second.first]->box.frameIndex;
                    min_carId2=values2[i].first->values[values2[i].second.first]->box.track_id;
                    min_loc2=i;
                    tmp=i;
                    break;
                }
            }
            for(int i=tmp+1;i<len2;i++){
                if(values2[i].second.first<=values2[i].second.second){
                    if(min_carId2>values2[i].first->values[values2[i].second.first]->box.track_id){
                        min_time2=values2[i].first->values[values2[i].second.first]->box.frameIndex;
                        min_carId2=values2[i].first->values[values2[i].second.first]->box.track_id;
                        min_loc2=i;
                    }
                }
            }

            if(min_carId==min_carId2){
                int xx=values1[min_loc].first->values[values1[min_loc].second.first]->box.left;
                int yy=values1[min_loc].first->values[values1[min_loc].second.first]->box.top;
                int xx2=values2[min_loc2].first->values[values2[min_loc2].second.first]->box.left;
                int yy2=values2[min_loc2].first->values[values2[min_loc2].second.first]->box.top;
                if(!IsPointInPolygon(Point(xx,yy),hull1)){
                    values1[min_loc].second.first++;
                }else if(!IsPointInPolygon(Point(xx2,yy2),hull2)){
                    values2[min_loc2].second.first++;
                }else{
                    values1[min_loc].second.first++;
                    values2[min_loc2].second.first++;
                    if(min_time<=min_time2)
                        answer1_2.insert(min_carId2);
                    else
                        answer2_1.insert(min_carId);
                }
            }else if(min_carId<min_carId2){
                values1[min_loc].second.first++;
            }else if(min_carId2<min_carId){
                values2[min_loc2].second.first++;
            }

            tag1=false,tag2=false;
            for(int i=0;i<len1;i++){
                if(values1[i].second.first<=values1[i].second.second){
                    tag1=true;
                    break;
                }
            }
            for(int i=0;i<len2;i++){
                if(values2[i].second.first<=values2[i].second.second) {
                    tag2=true;
                    break;
                }
            }
            cnt++;
        }
       cout<<answer1_2.size()<<"  2——1 ："<<answer2_1.size()<<endl;
     /*   for(auto it=answer1_2.begin();it!=answer1_2.end();it++){
            cout<<*it<<endl;
        }*/
    }

    std::vector<Leaf> query_time_B(vector<Point> hull,int st,int ed) const{
        auto values = std::vector<Leaf>();
            query_BT(mRoot.get(), mBox, hull, values, st , ed);
        return values;
    }

    double dcmp(double x) const{
        if(fabs(x) < eps) return 0;
        else return x < 0 ? -1 : 1;
    }
    bool SegmentProperIntersection(const Point& a1, const Point& a2, const Point& b1, const Point& b2) const{
        double c1 = Cross(a2-a1,b1-a1), c2 = Cross(a2-a1,b2-a1),
                c3 = Cross(b2-b1,a1-b1), c4=Cross(b2-b1,a2-b1);
        return dcmp(c1)*dcmp(c2)<0 && dcmp(c3)*dcmp(c4)<0;
    }
    bool OnSegment(const Point& p, const Point& a1, const Point& a2) const{
        return dcmp(Cross(a1-p, a2-p)) == 0 && dcmp(Dot(a1-p, a2-p)) < 0;
    }
    // 点集凸包
    vector<Point> ConvexHull(vector<Point> q) const{
        // 预处理，删除重复点
        vector<Point> p=q;
        sort(p.begin(), p.end());
        p.erase(unique(p.begin(), p.end()), p.end());

        int n = p.size();
        int m = 0;
        vector<Point> ch(n+1);
        for(int i = 0; i < n; i++) {
            while(m > 1 && Cross(ch[m-1]-ch[m-2], p[i]-ch[m-2]) <= 0) m--;
            ch[m++] = p[i];
        }
        int k = m;
        for(int i = n-2; i >= 0; i--) {
            while(m > k && Cross(ch[m-1]-ch[m-2], p[i]-ch[m-2]) <= 0) m--;
            ch[m++] = p[i];
        }
        if(n > 1) m--;
        ch.resize(m);
        return ch;
    }
    //点是否在凸包内
    int IsPointInPolygon(const Point& p, const vector<Point>& poly) const{
        int wn = 0;
        int n = poly.size();
        for(int i=0; i<n; ++i) {
            const Point& p1 = poly[i];
            const Point& p2 = poly[(i+1)%n];
            if(p1 == p || p2 == p || OnSegment(p, p1, p2)) return 2;//在边界上
            int k = dcmp(Cross(p2-p1, p-p1));
            int d1 = dcmp(p1.y - p.y);
            int d2 = dcmp(p2.y - p.y);
            if(k > 0 && d1 <= 0 && d2 > 0) wn++;
            if(k < 0 && d2 <= 0 && d1 > 0) wn--;
        }
        return wn!=0;
    }
    //判断2个凸包是否相交
    bool ConvexPolygonDisjoint(const vector<Point> ch1, const vector<Point> ch2) const{
        int c1 = ch1.size();
        int c2 = ch2.size();
        for(int i=0; i<c1; ++i)
            if(IsPointInPolygon(ch1[i], ch2) != 0) return true;
        for(int i=0; i<c2; ++i)
            if(IsPointInPolygon(ch2[i], ch1) != 0) return true;
        for(int i=0; i<c1; ++i)
            for(int j=0; j<c2; ++j)
                if(SegmentProperIntersection(ch1[i], ch1[(i+1)%c1], ch2[j], ch2[(j+1)%c2])) return true;
        return false;
    }

    bool ConvexPolygonDisjoint(const vector<Point> ch1, const Box box) const{
        vector<Point> ch2;
        Point point = Point(box.left,box.top);
        ch2.push_back(point);
        point = Point(box.left,box.top+box.height);
        ch2.push_back(point);
        point = Point(box.left+box.width,box.top+box.height);
        ch2.push_back(point);
        point = Point(box.left+box.width,box.top);
        ch2.push_back(point);
        ch2=ConvexHull(ch2);
        int c1 = ch1.size();
        int c2 = ch2.size();
        for(int i=0; i<c1; ++i)
            if(IsPointInPolygon(ch1[i], ch2) != 0) return true;
        for(int i=0; i<c2; ++i)
            if(IsPointInPolygon(ch2[i], ch1) != 0) return true;
        for(int i=0; i<c1; ++i)
            for(int j=0; j<c2; ++j)
                if(SegmentProperIntersection(ch1[i], ch1[(i+1)%c1], ch2[j], ch2[(j+1)%c2])) return true;
        return false;
    }

private:
///阈值
    static constexpr auto Threshold = std::size_t(3);
///最大深度
    static constexpr auto MaxDepth = std::size_t(11);
    struct Node
    {
        std::array<std::unique_ptr<Node>, 4> children;
        std::vector<T> values;
        BPTree* bPTree = new BPTree(1000, 1000);
    };
    Box mBox;
    std::unique_ptr<Node> mRoot;
    Equal mEqual;
    GetBox mGetBox;

    bool isLeaf(const Node* node) const
    {
        return !static_cast<bool>(node->children[0]);
    }

    Box computeBox(const Box& box, int i) const
    {
        auto origin = box.getTopLeft();
        auto childSize = box.getSize() / static_cast<float>(2);
        switch (i)
        {
            // North West
            case 0:
                return Box(origin, childSize);
                // Norst East
            case 1:
                return Box(Vector2(origin.x + childSize.x, origin.y), childSize);
                // South West
            case 2:
                return Box(Vector2(origin.x, origin.y + childSize.y), childSize);
                // South East
            case 3:
                return Box(origin + childSize, childSize);
            default:
                assert(false && "Invalid child index");
                return Box();
        }
    }

    int getQuadrant(const Box& nodeBox, const Box& valueBox) const
    {
        auto center = nodeBox.getCenter();
        // West
        if (valueBox.getRight() < center.x)
        {
            // North West
            if (valueBox.getBottom() < center.y)
                return 0;
                // South West
            else if (valueBox.top >= center.y)
                return 2;
                // Not contained in any quadrant
            else
                return -1;
        }
            // East
        else if (valueBox.left >= center.x)
        {
            // North East
            if (valueBox.getBottom() < center.y)
                return 1;
                // South East
            else if (valueBox.top >= center.y)
                return 3;
                // Not contained in any quadrant
            else
                return -1;
        }
            // Not contained in any quadrant
        else
            return -1;
    }

    void add(Node* node, std::size_t depth, const Box& box, const T& value)
    {
        assert(node != nullptr);
        assert(box.contains(mGetBox(value)));
        if (isLeaf(node))
        {
            // Insert the value in this node if possible
            if (depth >= MaxDepth || node->values.size() < Threshold){
               /* Box box = mGetBox(value);
                Leaf leaf(box.left,box.top,box.width,box.height,box.track_id,box.frameIndex);
                node->bPTree->insert(box.frameIndex,leaf);*/
                node->values.push_back(value);
            }
                // Otherwise, we split and we try again
            else
            {
                split(node, box);
                add(node, depth, box, value);
            }
        }
        else
        {
            auto i = getQuadrant(box, mGetBox(value));
            // Add the value in a child if the value is entirely contained in it
            if (i != -1)
                add(node->children[static_cast<std::size_t>(i)].get(), depth + 1, computeBox(box, i), value);
                // Otherwise, we add the value in the current node
            else{
                node->values.push_back(value);
                /*Box box = mGetBox(value);
                Leaf leaf(box.left,box.top,box.width,box.height,box.track_id,box.frameIndex);
                node->bPTree->insert(box.frameIndex,leaf);*/
            }

        }
    }

    void split(Node* node, const Box& box)
    {
        assert(node != nullptr);
        assert(isLeaf(node) && "Only leaves can be split");
        // Create children
        for (auto& child : node->children)
            child = std::make_unique<Node>();
        // Assign values to children
        auto newValues = std::vector<T>(); // New values for this node

        for (const auto& value : node->values)
        {
            auto i = getQuadrant(box, mGetBox(value));
            if (i != -1){
                node->children[static_cast<std::size_t>(i)]->values.push_back(value);
               /* Box box = mGetBox(value);
                Leaf leaf(box.left,box.top,box.width,box.height,box.track_id,box.frameIndex);
                node->children[static_cast<std::size_t>(i)]->bPTree->insert(box.frameIndex,leaf);*/
            }
            else{
                newValues.push_back(value);
            }
        }
        node->values = std::move(newValues);
      //  node->bPTree = nullptr;
    }

     void query1(Node* node,int &ans,vector<int> &vec){
        if(!isLeaf(node)){
            for (auto i = std::size_t(0); i < node->children.size(); ++i)
                query1(node->children[i].get(),ans,vec);
        }else{
            ans++;
            //node->bPTree->display(node->bPTree->getRoot());
            vec.push_back(node->values.size());
        }
    }
    void query(Node* node, const Box& box, const Box& queryBox, std::vector<T>& values) const
    {
        assert(node != nullptr);
        assert(queryBox.intersects(box));
        for (const auto& value : node->values)
        {
            if (queryBox.intersects(mGetBox(value)))
            {
                if(queryBox.baohan(mGetBox(value)))
                    values.push_back(value);
            }

        }
        if (!isLeaf(node))
        {
            for (auto i = std::size_t(0); i < node->children.size(); ++i)
            {
                auto childBox = computeBox(box, static_cast<int>(i));

                if (queryBox.intersects(childBox))
                    query(node->children[i].get(), childBox, queryBox, values);
            }
        }
    }

    void query(Node* node, const Box& box, vector<Point>& queryPull, std::vector<T>& values) const{
        assert(node != nullptr);
        if (!isLeaf(node))
        {
            for (auto i = std::size_t(0); i < node->children.size(); ++i)
            {
                auto childBox = computeBox(box, static_cast<int>(i));
                vector<Point> childBox1;
                Point point = Point(childBox.left,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left,childBox.top+childBox.height);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top+childBox.height);
                childBox1.push_back(point);
                if (ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(childBox1)))
                    query(node->children[i].get(), childBox, queryPull, values);
            }
        }else{

            for (const auto& value : node->values)
            {

                Box box = mGetBox(value);
                Point p = Point(box.left,box.top);
                if(IsPointInPolygon(p,queryPull)){
                    values.push_back(value);
                }
            }
        }
    }

    void query_BT(Node* node, const Box& box, vector<Point>& queryPull, std::vector<Leaf>& values,int st,int ed) const{
        assert(node != nullptr);
        // cout<< queryPull[2].y << endl;
//        assert(ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(box1)));

        if (!isLeaf(node))
        {
            for (auto i = std::size_t(0); i < node->children.size(); ++i)
            {
                auto childBox = computeBox(box, static_cast<int>(i));
                vector<Point> childBox1;
                Point point = Point(childBox.left,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left,childBox.top+childBox.height);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top+childBox.height);
                childBox1.push_back(point);
                if (ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(childBox1)))
                    query_BT(node->children[i].get(), childBox, queryPull, values,st,ed);
            }
        }
        else{
          /*  if(node->values.size()<len){
                int res_st=-1,res_ed=-1;
                int l=0,r=node->values.size()-1;
                while(l<=r){
                    int mid=l+((r-l)>>1);
                    int res_time=node->values[mid]->box.frameIndex;
                    if(res_time<=ed){
                        l=mid+1;
                    }else{
                        r=mid-1;
                    }
                }
                res_ed=l;
                l=0,r=node->values.size()-1;
                while(l<=r){
                    int mid=l+((r-l)>>1);
                    int res_time=node->values[mid]->box.frameIndex;
                    if(res_time < st)
                        l=mid+1;
                    else
                        r=mid-1;
                }
                res_st=l;
                for(int i=res_st;i<res_ed;i++){
                    Box box = mGetBox(node->values[i]);
                    Point p = Point(box.left,box.top);
                    Leaf leaf(node->values[i]->box.left,node->values[i]->box.top,0,0,node->values[i]->box.track_id,node->values[i]->box.frameIndex);
                    if(IsPointInPolygon(p,queryPull)){
                        values.push_back(leaf);
                    }
                }
            }else*/
             {
                BNode* STNode=node->bPTree->search_key(st);
                BNode* EdNode=node->bPTree->search_key(ed);
                if(STNode != nullptr){
                    int idx = std::lower_bound(STNode->keys.begin(), STNode->keys.end(), st) - STNode->keys.begin();
                    int idy ;
                    if(STNode==EdNode)
                        idy = std::upper_bound(STNode->keys.begin(), STNode->keys.end(), ed) - STNode->keys.begin();
                    else
                        idy = STNode->keys.size();
                    if(!((idx == STNode->keys.size() || STNode->keys[idx] < st))){
                        for(int i=idx;i<idy;i++)
                            if(IsPointInPolygon(Point(STNode->ptr2TreeOrData.dataPtr[i].left,STNode->ptr2TreeOrData.dataPtr[i].top),queryPull))
                                values.push_back(STNode->ptr2TreeOrData.dataPtr[i]);
                    }

                    while(STNode->ptr2next!=EdNode&&STNode!=EdNode){
                        for(int i=0;i<STNode->ptr2next->keys.size();i++){
                            if(IsPointInPolygon(Point(STNode->ptr2TreeOrData.dataPtr[i].left,STNode->ptr2TreeOrData.dataPtr[i].top),queryPull))
                                values.push_back(STNode->ptr2next->ptr2TreeOrData.dataPtr[i]);
                        }
                        STNode=STNode->ptr2next;
                    }
                    if(EdNode!= nullptr && STNode!=EdNode){
                        idy = std::upper_bound(EdNode->keys.begin(), EdNode->keys.end(), ed) - EdNode->keys.begin();
                        for(int i=0;i<idy;i++){
                            if(IsPointInPolygon(Point(STNode->ptr2TreeOrData.dataPtr[i].left,STNode->ptr2TreeOrData.dataPtr[i].top),queryPull))
                                values.push_back(EdNode->ptr2TreeOrData.dataPtr[i]);
                        }
                    }
                }
            }

        }
    }


    void query(Node* node, const Box& box, vector<Point>& queryPull, std::vector<T>& values ,int st ,int ed) const{
        assert(node != nullptr);
        // cout<< queryPull[2].y << endl;
//        assert(ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(box1)));

        if (!isLeaf(node))
        {
            for (auto i = std::size_t(0); i < node->children.size(); ++i)
            {
                auto childBox = computeBox(box, static_cast<int>(i));
                vector<Point> childBox1;
                Point point = Point(childBox.left,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left,childBox.top+childBox.height);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top+childBox.height);
                childBox1.push_back(point);
                if (ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(childBox1)))
                    query(node->children[i].get(), childBox, queryPull, values,st,ed);
            }
        }else{
            int res_st=-1,res_ed=-1;
            int l=0,r=node->values.size()-1;
            while(l<=r){
                int mid=l+((r-l)>>1);
                int res_time=node->values[mid]->box.frameIndex;
                if(res_time<=ed){
                    l=mid+1;
                }else{
                    r=mid-1;
                }
            }
            res_ed=l;
            l=0,r=node->values.size()-1;
            while(l<=r){
                int mid=l+((r-l)>>1);
                int res_time=node->values[mid]->box.frameIndex;
                if(res_time < st)
                    l=mid+1;
                else
                    r=mid-1;
            }
            res_st=l;
            for(int i=res_st;i<res_ed;i++){
                Box box = mGetBox(node->values[i]);
                Point p = Point(box.left,box.top);
                if(IsPointInPolygon(p,queryPull)){
                    values.push_back(node->values[i]);
                }
            }
        }
    }

    void query_pointer(Node* node, const Box& box, vector<Point>& queryPull, std::vector<pair<Node*,pair<int,int>>>& values ,int st ,int ed) const{
        assert(node != nullptr);
        // cout<< queryPull[2].y << endl;
//        assert(ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(box1)));

        if (!isLeaf(node))
        {
            for (auto i = std::size_t(0); i < node->children.size(); ++i)
            {
                auto childBox = computeBox(box, static_cast<int>(i));
                vector<Point> childBox1;
                Point point = Point(childBox.left,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left,childBox.top+childBox.height);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top);
                childBox1.push_back(point);
                point = Point(childBox.left+childBox.width,childBox.top+childBox.height);
                childBox1.push_back(point);
                if (ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(childBox1)))
                    query_pointer(node->children[i].get(), childBox, queryPull, values,st,ed);
            }
        }else{
            int res_st=-1,res_ed=-1;
            int l=0,r=node->values.size()-1;
            while(l<=r){
                int mid=l+((r-l)>>1);
                int res_time=node->values[mid]->box.frameIndex;
                if(res_time<=ed){
                    l=mid+1;
                }else{
                    r=mid-1;
                }
            }
            res_ed=l;
            l=0,r=node->values.size()-1;
            while(l<=r){
                int mid=l+((r-l)>>1);
                int res_time=node->values[mid]->box.frameIndex;
                if(res_time < st)
                    l=mid+1;
                else
                    r=mid-1;
            }
            res_st=l;
            if(res_st<res_ed){
                pair<Node*,pair<int,int>> pointer;
                pointer.first=node;
                pointer.second.first = res_st;
                pointer.second.second = res_ed-1;
                values.push_back(pointer);
            }

        }
    }
};
#endif //QUADTRANS_QUADTREE_H

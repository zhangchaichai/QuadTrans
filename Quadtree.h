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

    std::vector<T> query_time(vector<Point> hull,int st,int ed) const{
        auto values = std::vector<T>();
        query(mRoot.get(), mBox, hull, values, st , ed);
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
            if (depth >= MaxDepth || node->values.size() < Threshold)
                node->values.push_back(value);
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
            else
                node->values.push_back(value);
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
            if (i != -1)
                node->children[static_cast<std::size_t>(i)]->values.push_back(value);
            else
                newValues.push_back(value);
        }
        node->values = std::move(newValues);
    }

    void remove(Node* node, Node* parent, const Box& box, const T& value)
    {
        assert(node != nullptr);
        assert(box.contains(mGetBox(value)));
        if (isLeaf(node))
        {
            // Remove the value from node
            removeValue(node, value);
            // Try to merge the parent
            if (parent != nullptr)
                tryMerge(parent);
        }
        else
        {
            // Remove the value in a child if the value is entirely contained in it
            auto i = getQuadrant(box, mGetBox(value));
            if (i != -1)
                remove(node->children[static_cast<std::size_t>(i)].get(), node, computeBox(box, i), value);
                // Otherwise, we remove the value from the current node
            else
                removeValue(node, value);
        }
    }

    void removeValue(Node* node, const T& value)
    {
        // Find the value in node->values
        auto it = std::find_if(std::begin(node->values), std::end(node->values),
                               [this, &value](const auto& rhs){ return mEqual(value, rhs); });
        assert(it != std::end(node->values) && "Trying to remove a value that is not present in the node");
        // Swap with the last element and pop back
        *it = std::move(node->values.back());
        node->values.pop_back();
    }

    void tryMerge(Node* node)
    {
        assert(node != nullptr);
        assert(!isLeaf(node) && "Only interior nodes can be merged");
        auto nbValues = node->values.size();
        for (const auto& child : node->children)
        {
            if (!isLeaf(child.get()))
                return;
            nbValues += child->values.size();
        }
        if (nbValues <= Threshold)
        {
            node->values.reserve(nbValues);
            // Merge the values of all the children
            for (const auto& child : node->children)
            {
                for (const auto& value : child->values)
                    node->values.push_back(value);
            }
            // Remove the children
            for (auto& child : node->children)
                child.reset();
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

        vector<Point> box1;
        Point point = Point(box.left,box.top);
        box1.push_back(point);
        point = Point(box.left,box.top-box.height);
        box1.push_back(point);
        point = Point(box.left+box.width,box.top);
        box1.push_back(point);
        point = Point(box.left+box.width,box.top-box.height);
       // cout<< point.y<<endl;
        box1.push_back(point);
       // cout<< queryPull[2].y << endl;
//        assert(ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(box1)));
        for (const auto& value : node->values)
        {
            Box box = mGetBox(value);
            Point p = Point(box.left,box.top);
            if(IsPointInPolygon(p,queryPull)){
                    values.push_back(value);
            }
        }
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
        }
    }

    void query(Node* node, const Box& box, vector<Point>& queryPull, std::vector<T>& values ,int st ,int ed) const{
        assert(node != nullptr);

        vector<Point> box1;
        Point point = Point(box.left,box.top);
        box1.push_back(point);
        point = Point(box.left,box.top-box.height);
        box1.push_back(point);
        point = Point(box.left+box.width,box.top);
        box1.push_back(point);
        point = Point(box.left+box.width,box.top-box.height);
        // cout<< point.y<<endl;
        box1.push_back(point);
        // cout<< queryPull[2].y << endl;
//        assert(ConvexPolygonDisjoint(ConvexHull(queryPull),ConvexHull(box1)));
        int res_st=-1,res_ed=-1;
        int l=0,r=node->values.size()-1;
        while(l<=r){
            int mid=l+((r-l)>>1);
            int res_time=node->values[mid]->box.frameIndex;
            if(res_time>=ed){
                r=mid-1;
            }else{
                l=mid+1;
            }
        }
        if(!(r<0||r==node->values.size()-1)){
            res_ed=r;
        }
        l=0,r=node->values.size();
        int tag=0;
        while(l<r){
            int mid=l+((r-l)>>1);
            int res_time=node->values[mid]->box.frameIndex;
            if(res_time < st)
                l=mid+1;
            else if(res_time > st)
                r=mid;
            else{
                res_st=mid;
                tag=1;
                break;
            }
        }
        if(!tag){
            res_st=l;
        }
        for(int i=res_st;i<=res_ed;i++){
            Box box = mGetBox(node->values[i]);
            Point p = Point(box.left,box.top);
            if(IsPointInPolygon(p,queryPull)){
                values.push_back(node->values[i]);
            }
        }
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
        }
    }
};
#endif //QUADTRANS_QUADTREE_H

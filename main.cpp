#include <iostream>
#include <cstring>
#include <unordered_set>
#include "DetectorState.h"
#include "JsonHelper.h"
#include "Box.h"
#include "Quadtree.h"

using namespace std;

struct Nodes
{
    Box box;
};

struct node
{
    int x,y;
};
node vex[100000];//存入的所有的点
node stackk[100000];//凸包中所有的点
int xx,yy;
bool cmp1(node a,node b)//排序找第一个点
{
    if(a.y==b.y)
        return a.x<b.x;
    else
        return a.y<b.y;
}
double Cross(const node& A, const node& B) {
    return A.x*B.y - A.y*B.x;
}
int cross(node a,node b,node c)//计算叉积
{
    return (b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y);
}
double dis(node a,node b)//计算距离
{
    return sqrt((a.x-b.x)*(a.x-b.x)*1.0+(a.y-b.y)*(a.y-b.y));
}
bool cmp2(node a,node b)//极角排序另一种方法，速度快
{
    if(atan2(a.y-yy,a.x-xx)!=atan2(b.y-yy,b.x-xx))
        return (atan2(a.y-yy,a.x-xx))<(atan2(b.y-yy,b.x-xx));
    return a.x<b.x;
}
bool cmp(node a,node b)//极角排序
{
    int m=cross(vex[0],a,b);
    if(m>0)
        return 1;
    else if(m==0&&dis(vex[0],a)-dis(vex[0],b)<=0)
        return 1;
    else return 0;
    /*if(m==0)
        return dis(vex[0],a)-dis(vex[0],b)<=0?true:false;
    else
        return m>0?true:false;*/
}

double dcmp(double x) {
    if(fabs(x) < eps) return 0;
    else return x < 0 ? -1 : 1;
}
bool OnSegment(const Point& p, const Point& a1, const Point& a2) {
    return dcmp(Cross(a1-p, a2-p)) == 0 && dcmp(Dot(a1-p, a2-p)) < 0;
}
//点是否在凸包内
int IsPointInPolygon(const Point& p, const vector<Point>& poly) {
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
int maxChildInt = 4, maxNodeLeaf = 3;
int main() {
    // read json from disk, parse it and store into unorderedmap
    JsonHelper jsonHelper;
    jsonHelper.readSingleJson("/home/zcc/Code/TestData/0-baseline1.json");
    //unordered_map<int, vector<DetectorState>> res;
    vector<DetectorState> res;
    //将所有数据存到res中
 //   for(int i=0;i<2;i++)
        jsonHelper.parseJsonForMiris(res);
    int vec_len = res.size();
    cout<<vec_len<<endl;
    auto nodes = std::vector<Nodes>();
    int max_left=-1;
    int max_frame = -1;
    int maxxx= -1;
    int ressss=0;
    int sum_A=0;
    for(int i=0;i<vec_len;i++){
        Nodes node;
        node.box.left = (res[i].left+res[i].right)/2;
        node.box.top =  (res[i].top+res[i].bottom)/2;
        node.box.width = 0;
        node.box.height = 0;
        node.box.track_id = res[i].track_id;
       // cout<<res[i].track_id<<endl;
        node.box.frameIndex = res[i].frameIndex;
        nodes.push_back(node);
      /*  max_left=max(max_left,res[i].left);
        if(res[i].frameIndex==max_frame){
            ressss++;
        }else{
            maxxx=max(maxxx,ressss);
            ressss=0;
            max_frame=res[i].frameIndex;
        }*/
    }
   // cout<<" 1帧 几辆车 " << maxxx <<endl;
    clock_t startTime,endTime;

    startTime = clock();
    auto box = Box(0.0, 0.0, 2048.0, 2048.0);
    auto getBox = [](Nodes* node)
    {
        return node->box;
    };
    auto quadtree = Quadtree<Nodes*, decltype(getBox)>(box, getBox);
    for (auto& node : nodes){
        quadtree.add(&node);
    }

    endTime = clock();
    cout << "The 建树 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
 //   quadtree.bianli();
    //A路口
    vector<Point>hull_A;
    Point p = Point(0.0,0.0);
    hull_A.push_back(p);
    p = Point(0.0,525.0);
    hull_A.push_back(p);
    p = Point(550.0,525.0);
    hull_A.push_back(p);
    p = Point(1200.0,420.0);
    hull_A.push_back(p);
    p = Point(1200.0,0.0);
    hull_A.push_back(p);

    //C路口
    vector<Point>hull_C;
    p = Point(1640.0,630.0);
    hull_C.push_back(p);
    p = Point(1040.0,1080.0);
    hull_C.push_back(p);
    p = Point(1920.0,1080.0);
    hull_C.push_back(p);
    p = Point(1920.0,630.0);
    hull_C.push_back(p);

    startTime = clock();

    /// 二分查找A C路口的车辆
    auto values= quadtree.query_time(hull_A,0,30999);
    auto values1= quadtree.query_time(hull_C,0,30999);




    unordered_map<int,int> track_time;
    unordered_set<int> track_A_C;
    unordered_set<int> track_C_A;

    for(int i=0;i<values.size();i++){
        track_time[values[i]->box.track_id]=values[i]->box.frameIndex;
    }

    for(int i=0;i<values1.size();i++){
        if(track_time[values1[i]->box.track_id]){
            if(values1[i]->box.frameIndex>track_time[values1[i]->box.track_id]){
                track_A_C.insert(values1[i]->box.track_id);
            }else{
                track_C_A.insert(values1[i]->box.track_id);
            }
        }
    }

    endTime = clock();

    cout << "The 二分查询转弯 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


    startTime = clock();
    quadtree.query_time_pointer(hull_A,hull_C,0,30999);

    endTime = clock();

    cout << "The 堆 查询转弯 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    cout<<"A C 的数量："<<track_A_C.size()<<endl;
    for(auto it=track_A_C.begin();it!=track_A_C.end();it++){
  //      cout<<" A - C "<< *it<<endl;
    }
    cout<<"C A 的数量："<<track_C_A.size()<<endl;
   /*for(auto it=track_C_A.begin();it!=track_C_A.end();it++){
        cout<<" C - A "<< *it<<endl;
    }*/

    // 查询指定区域内停留的车辆
    vector<Point>hull_S;
    p = Point(710.0*2,197.0*2);
    hull_S.push_back(p);
    p = Point(853.0*2,234.0*2);
    hull_S.push_back(p);
    p = Point(832.0*2,250.0*2);
    hull_S.push_back(p);
    p = Point(676.0*2,207.0*2);
    hull_S.push_back(p);

    startTime = clock();

    auto values_stay= quadtree.query(hull_S);

  //  quadtree.query_stop_pointer(hull_S,0,30999);

    //方法一： 即可转换成在这个区间内出现了多少次
    unordered_map<int,int> track_stay;
    for(int i=0;i<values_stay.size();i++){
        track_stay[values_stay[i]->box.track_id]++;
    }
    for(auto it=track_stay.begin();it!=track_stay.end();it++){
       // cout<<it->first<<" 出现了几次 ： "<<track_stay[it->first]<<endl;
    }
    endTime = clock();
    cout << "The 停留 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    startTime = clock();
    quadtree.query_stop_pointer2(hull_S,0,30999);
    endTime = clock();
    cout << "The 滑动窗口 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    //方法二： 进入和离开这个区域的时间帧
    unordered_map<int,int> track_stay_st;
    unordered_map<int,int> track_stay_ed;
    for(int i=0;i<values_stay.size();i++){
        if(track_stay_st[values_stay[i]->box.track_id]==0)
            track_stay_st[values_stay[i]->box.track_id]=values_stay[i]->box.frameIndex;
        else
            track_stay_st[values_stay[i]->box.track_id]=min(track_stay_st[values_stay[i]->box.track_id],values_stay[i]->box.frameIndex);
        track_stay_ed[values_stay[i]->box.track_id]=max(track_stay_ed[values_stay[i]->box.track_id],values_stay[i]->box.frameIndex);
    }
  /*  for(auto it=track_stay.begin();it!=track_stay.end();it++){
        if(track_stay_ed[it->first]-track_stay_st[it->first]>=100)
            cout<<it->first<<" 时间差 ："<<track_stay_ed[it->first]-track_stay_st[it->first]<<endl;
    }*/

    // 查询闯红灯的车辆
    startTime = clock();
    //通行区
    vector<Point>hull_Red_Y;
    p = Point(0.0,0.0);
    hull_Red_Y.push_back(p);
    p = Point(0.0,525.0);
    hull_Red_Y.push_back(p);
    p = Point(550.0,525.0);
    hull_Red_Y.push_back(p);
    p = Point(1200.0,420.0);
    hull_Red_Y.push_back(p);
    p = Point(1200.0,0.0);
    hull_Red_Y.push_back(p);

    unordered_map<int,int> track_red_y;
    int st,ed;
    st=6291;ed=7321;
    startTime = clock();
    auto values_Red_Y= quadtree.query_time(hull_Red_Y,st,ed);
    for(int i=0;i<values_Red_Y.size();i++){
        track_red_y[values_Red_Y[i]->box.track_id]=max(track_red_y[values_Red_Y[i]->box.track_id],values_Red_Y[i]->box.frameIndex);
    }

    vector<Point>hull_Red_N;
    p = Point(550.0,525.0);
    hull_Red_N.push_back(p);
    p = Point(1040.0,1080.0);
    hull_Red_N.push_back(p);
    p = Point(1640.0,630.0);
    hull_Red_N.push_back(p);
    p = Point(1200.0,420.0);
    hull_Red_N.push_back(p);
   // unordered_map<int,int>track_red_n;

    auto values_Red_N= quadtree.query_time(hull_Red_N,st,ed);

    /* vector<Leaf> values_Red_N_B;
    values_Red_N_B = quadtree.query_time_B(hull_Red_N,st,ed);
    for(int i=0;i<values_Red_N_B.size();i++){
        if(track_red_n[values_Red_N_B[i].track_id]!=0)
            track_red_n[values_Red_N_B[i].track_id]=min(track_red_n[values_Red_N_B[i].track_id],values_Red_N_B[i].frameIndex);
        else
            track_red_n[values_Red_N_B[i].track_id]=values_Red_N_B[i].frameIndex;
    }
    for(auto it=track_red_n.begin();it!=track_red_n.end();it++){
        if(it->second<=ed&&it->second>=st&&track_red_y[it->first]<=ed&&track_red_y[it->first]>=st&&it->second>track_red_y[it->first]){
            cout<<it->first<<" 闯红灯"<<endl;
        }
    }*/
    unordered_set<int> track_chuang;
    for(int i=0;i<values_Red_N.size();i++){
        if(track_red_y[values_Red_N[i]->box.track_id]!=0) {
            /// values_Red_N[i]->box.frameIndex<=ed && values_Red_N[i]->box.frameIndex>=st && track_red_y[values_Red_N[i]->box.track_id]<=ed && track_red_y[values_Red_N[i]->box.track_id]>=st &&
            if( values_Red_N[i]->box.frameIndex>track_red_y[values_Red_N[i]->box.track_id]){
                track_chuang.insert(values_Red_N[i]->box.track_id);
            }
        }
    }

    for(auto it=track_chuang.begin();it!=track_chuang.end();it++){
        cout<< *it<<" 闯红灯"<<endl;
    }

    endTime = clock();
    cout << "The 闯红灯 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
  /*  for(int i=0;i<values_Red_N.size();i++){
        if(track_red_n[values_Red_N[i]->box.track_id]!=0)
            track_red_n[values_Red_N[i]->box.track_id]=min(track_red_n[values_Red_N[i]->box.track_id],values_Red_N[i]->box.frameIndex);
        else
            track_red_n[values_Red_N[i]->box.track_id]=values_Red_N[i]->box.frameIndex;
    }
    for(auto it=track_red_n.begin();it!=track_red_n.end();it++){
        if(it->second<=ed&&it->second>=st&&track_red_y[it->first]<=ed&&track_red_y[it->first]>=st&&it->second>track_red_y[it->first]){
            cout<<it->first<<" 闯红灯"<<endl;
        }
    }*/

    endTime = clock();
    cout << "The 闯红灯 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return 0;
}

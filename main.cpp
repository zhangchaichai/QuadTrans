#include <iostream>
#include <cstring>
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
int maxChildInt = 4, maxNodeLeaf = 3;
int main() {
    // read json from disk, parse it and store into unorderedmap
    JsonHelper jsonHelper;
    jsonHelper.readSingleJson("/home/zcc/Code/TestData/0-baseline1.json");
    //unordered_map<int, vector<DetectorState>> res;
    vector<DetectorState> res;
    //将所有数据存到res中
   // for(int i=0;i<72;i++)
        jsonHelper.parseJsonForMiris(res);
    int vec_len = res.size();
    cout<<vec_len<<endl;
    auto nodes = std::vector<Nodes>();
    int max_left=-1;
    int max_frame = -1;
    int maxxx= -1;
    int ressss=0;
    for(int i=0;i<vec_len;i++){
        Nodes node;
        node.box.left = (res[i].left+res[i].right)/2;
        node.box.top = (res[i].top+res[i].bottom)/2;
        node.box.width = 0;
        node.box.height = 0;
        node.box.track_id = res[i].track_id;
       // cout<<res[i].track_id<<endl;
        node.box.frameIndex = res[i].frameIndex;
        nodes.push_back(node);
        max_left=max(max_left,res[i].left);
        if(res[i].frameIndex==max_frame){
            ressss++;
        }else{
            maxxx=max(maxxx,ressss);
            ressss=0;
            max_frame=res[i].frameIndex;
        }
    }
cout<<" 1帧 几辆车 " << maxxx <<endl;
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
    quadtree.bianli();
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
    auto values= quadtree.query(hull_A);

    auto values1= quadtree.query(hull_C);
    endTime = clock();
    unordered_map<int,int> track;
    unordered_map<int,int> track_time;
    for(int i=0;i<values.size();i++){
       // cout<<values[i]->box.track_id<<endl;
       // if(track[values[i]->box.track_id]!=1)
        track_time[values[i]->box.track_id]=values[i]->box.frameIndex;
        track[values[i]->box.track_id]=1;
    }
   // cout<< " A路口： " <<track.size() << " " << track_time.size()<<endl;
    unordered_map<int,int> track1;
    unordered_map<int,int> track_time1;
    for(int i=0;i<values1.size();i++){
        //if(track1[values1[i]->box.track_id]!=1)
        track_time1[values1[i]->box.track_id]=values1[i]->box.frameIndex;
        track1[values1[i]->box.track_id]=1;
    }
    unordered_map<int,pair<double,double>> track_test;
    for(int i=0;i<values1.size();i++){
        pair<double,double>pair;
        pair.first=(values1[i]->box.left+values1[i]->box.getRight())/2.0;
        pair.second=(values1[i]->box.top+values1[i]->box.getBottom())/2.0;
        if(track_test[values1[i]->box.track_id].first==0&&track_test[values1[i]->box.track_id].second==0)
            track_test[values1[i]->box.track_id]=pair;
    }
    int cnt=0;
    for(auto it=track_test.begin();it!=track_test.end();it++){
        node node;
        node.x=it->second.first;
        node.y=it->second.second;
        vex[cnt++]=node;
    }
    int t=cnt;
    memset(stackk,0,sizeof(stackk));
    sort(vex,vex+t,cmp1);
    stackk[0]=vex[0];
    xx=stackk[0].x;
    yy=stackk[0].y;
    sort(vex+1,vex+t,cmp2);//cmp2是更快的，cmp更容易理解
    stackk[1]=vex[1];//将凸包中的第两个点存入凸包的结构体中
    int top=1;//最后凸包中拥有点的个数
    for(int i=2; i<t; i++)
    {
        while(i>=1&&cross(stackk[top-1],stackk[top],vex[i])<0)   //对使用极角排序的i>=1有时可以不用，但加上总是好的
            top--;
        stackk[++top]=vex[i];                                    //控制<0或<=0可以控制重点，共线的，具体视题目而定。
    }
    /*for(int i=1; i<=top; i++)//输出凸包上的点
        cout<<stackk[i].x<<" "<<stackk[i].y<<" " <<endl;*/
    double ans= 0 ;
    for(int i=1;i<top-1;i++){
        node node1,node2;
        node1.x=stackk[i].x-stackk[0].x;
        node1.y=stackk[i].y-stackk[0].y;
        node2.x=stackk[i+1].x-stackk[0].x;
        node2.y=stackk[i+1].y-stackk[0].y;
        ans += Cross(node1,node2);
    }
    cout<<ans/580.0/450.0<<endl;
  //  cout<< " B路口： " <<track1.size()<<"  " << track_time1.size()<<endl;
    vector<int> track_A_C;
    vector<int> track_C_A;
    for(auto it=track1.begin();it!=track1.end();it++){
        if(track[it->first]){
            if(track_time[it->first]<track_time1[it->first]){
                track_A_C.push_back(it->first);
            }
            else if(track_time[it->first]>track_time1[it->first]){
                track_C_A.push_back(it->first);
            }
        }
    }
    cout << "The 转弯 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    cout<<"A C 的数量："<<track_A_C.size()<<endl;
    for(int i=0;i<track_A_C.size();i++){
       // cout<<" A - C "<< track_A_C[i]<<endl;
    }
    cout<<"C A 的数量："<<track_C_A.size()<<endl;
    for(int i=0;i<track_C_A.size();i++){
      //  cout<<" C - A "<< track_C_A[i]<<endl;
    }

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
    auto values_stay= quadtree.query(hull_S);

    //方法一： 即可转换成在这个区间内出现了多少次
    unordered_map<int,int> track_stay;
    for(int i=0;i<values_stay.size();i++){
        track_stay[values_stay[i]->box.track_id]++;
    }
    for(auto it=track_stay.begin();it!=track_stay.end();it++){
        //cout<<it->first<<" 出现了几次 ： "<<track_stay[it->first]<<endl;
    }

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
    for(auto it=track_stay.begin();it!=track_stay.end();it++){
        //cout<<it->first<<" 时间差 ："<<track_stay_ed[it->first]-track_stay_st[it->first]<<endl;
    }

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
    auto values_Red_Y= quadtree.query_time(hull_Red_Y,st,ed);
    //cout<<values_Red_Y.size()<<endl;

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
    unordered_map<int,int> track_red_n;
    auto values_Red_N= quadtree.query_time(hull_Red_N,st,ed);
    cout<<values_Red_N.size()<<endl;

    for(int i=0;i<values_Red_N.size();i++){
        if(track_red_n[values_Red_N[i]->box.track_id]!=0)
            track_red_n[values_Red_N[i]->box.track_id]=min(track_red_n[values_Red_N[i]->box.track_id],values_Red_N[i]->box.frameIndex);
        else
            track_red_n[values_Red_N[i]->box.track_id]=values_Red_N[i]->box.frameIndex;
    }
    for(auto it=track_red_n.begin();it!=track_red_n.end();it++){
        if(it->second<=ed&&it->second>=st&&track_red_y[it->first]<=ed&&track_red_y[it->first]>=st&&it->second>track_red_y[it->first]){
            cout<<it->first<<" 闯红灯"<<endl;
        }
    }

    endTime = clock();
    cout << "The 闯红灯 time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return 0;
}

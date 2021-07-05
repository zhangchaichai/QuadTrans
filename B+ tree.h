//
// Created by zcc on 2021/7/5.
//

#ifndef QUADTRANS_B_TREE_H
#define QUADTRANS_B_TREE_H
#include <iostream>
#include <vector>
using namespace std;
class Leaf{
public:
    double left;
    double top;
    double width; // Must be positive
    double height; // Must be positive
    int track_id;
    int frameIndex;
    Leaf(double Left, double Top, double Width, double Height,int Track_id,int FrameIndex) :
            left(Left), top(Top), width(Width), height(Height) , track_id(Track_id) ,frameIndex(FrameIndex){

    }
    friend class BPTree;
};
class BNode {
    /*
		Generally size of the this node should be equal to the block size. Which will limit the number of disk access and increase the accesssing time.
		Intermediate nodes only hold the Tree pointers which is of considerably small size(so they can hold more Tree pointers) and only Leaf nodes hold
		the data pointer directly to the disc.

		IMPORTANT := All the data has to be present in the leaf node
	*/
public:
    bool isLeaf;
    vector<int> keys;
    //Node* ptr2parent; //Pointer to go to parent node CANNOT USE check https://stackoverflow.com/questions/57831014/why-we-are-not-saving-the-parent-pointer-in-b-tree-for-easy-upward-traversal-in
    BNode* ptr2next;              //Pointer to connect next node for leaf nodes
    union ptr {                  //to make memory efficient Node
        vector<BNode*> ptr2Tree;  //Array of pointers to Children sub-trees for intermediate Nodes
        vector<Leaf> dataPtr;   // Data-Pointer for the leaf node

        ptr();   // To remove the error !?
        ~ptr();  // To remove the error !?
    } ptr2TreeOrData;

    friend class BPTree;  // to access private members of the Node and hold the encapsulation concept
public:
    BNode();
};
class BPTree {
    /*
		::For Root Node :=
			The root node has, at least two tree pointers
		::For Internal Nodes:=
			1. ceil(maxIntChildLimit/2)     <=  #of children <= maxIntChildLimit
			2. ceil(maxIntChildLimit/2)-1  <=  #of keys     <= maxIntChildLimit -1
		::For Leaf Nodes :=
			1. ceil(maxLeafNodeLimit/2)   <=  #of keys     <= maxLeafNodeLimit -1
	*/
private:
    int maxIntChildLimit;                                   //Limiting  #of children for internal Nodes!
    int maxLeafNodeLimit;                                   // Limiting #of nodes for leaf Nodes!!!
    BNode* root;                                             //Pointer to the B+ Tree root
    void insertInternal(int x, BNode** cursor, BNode** child);  //Insert x from child in cursor(parent)
    BNode** findParent(BNode* cursor, BNode* child);
    BNode* firstLeftNode(BNode* cursor);

public:
    BPTree();
    BPTree(int degreeInternal, int degreeLeaf);
    BNode* getRoot();
    int getMaxIntChildLimit();
    int getMaxLeafNodeLimit();
    void setRoot(BNode *);
    void display(BNode* cursor);
    void seqDisplay(BNode* cursor);
    void search(int key);
    BNode* search_key(int key);
    void insert(int key, Leaf filePtr);
};
#endif //QUADTRANS_B_TREE_H

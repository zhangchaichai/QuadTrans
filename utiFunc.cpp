//
// Created by zcc on 2021/7/5.
//

#include <iostream>
#include "B+ tree.h"

BNode* parent = NULL;

BNode::ptr::ptr() {
}

BNode::ptr::~ptr() {
}

BNode::BNode() {
    this->isLeaf = false;
    this->ptr2next = NULL;
}

BPTree::BPTree() {
    /*
        By Default it will take the maxIntChildLimit as 4. And
        maxLeafNodeLimit as 3.

        ::REASON FOR TWO SEPERATE VARIABLES maxIntChildLimit & maxLeafNodeLimit !!
        We are keeping the two seperate Orders
        because Internal Nodes can hold more values in one disc block
        as the size of the Tree pointer is small but the size of the
        data pointer in the leaf nodes is large so we can only put less
        nodes in the leafs as compared to the internal Nodes. Thats the
        reson to reperate out these to variables.

    */
    this->maxIntChildLimit = 4;
    this->maxLeafNodeLimit = 3;
    this->root = NULL;
}

BPTree::BPTree(int degreeInternal, int degreeLeaf) {
    this->maxIntChildLimit = degreeInternal;
    this->maxLeafNodeLimit = degreeLeaf;
    this->root = NULL;
}

int BPTree::getMaxIntChildLimit() {
    return maxIntChildLimit;
}

int BPTree::getMaxLeafNodeLimit() {
    return maxLeafNodeLimit;
}

BNode* BPTree::getRoot() {
    return this->root;
}

void BPTree::setRoot(BNode *ptr) {
    this->root = ptr;
}

BNode* BPTree::firstLeftNode(BNode* cursor) {
    if (cursor->isLeaf)
        return cursor;
    for (int i = 0; i < cursor->ptr2TreeOrData.ptr2Tree.size(); i++)
        if (cursor->ptr2TreeOrData.ptr2Tree[i] != NULL)
            return firstLeftNode(cursor->ptr2TreeOrData.ptr2Tree[i]);

    return NULL;
}

BNode** BPTree::findParent(BNode* cursor, BNode* child) {
    /*
		Finds parent using depth first traversal and ignores leaf nodes as they cannot be parents
		also ignores second last level because we will never find parent of a leaf node during insertion using this function
	*/

    if (cursor->isLeaf || cursor->ptr2TreeOrData.ptr2Tree[0]->isLeaf)
        return NULL;

    for (int i = 0; i < cursor->ptr2TreeOrData.ptr2Tree.size(); i++) {
        if (cursor->ptr2TreeOrData.ptr2Tree[i] == child) {
            parent = cursor;
        } else {
            //Commenting To Remove vector out of bound Error:
            //new (&cursor->ptr2TreeOrData.ptr2Tree) std::vector<Node*>;
            BNode* tmpCursor = cursor->ptr2TreeOrData.ptr2Tree[i];
            findParent(tmpCursor, child);
        }
    }

    return &parent;
}
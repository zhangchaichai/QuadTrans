//
// Created by zcc on 2021/7/5.
//
#include <iostream>
#include <queue>
#include "B+ tree.h"


void BPTree::display(BNode* cursor) {
    /*
		Depth First Display

    if (cursor != NULL) {
        for (int i = 0; i < cursor->keys.size(); i++)
            cout << cursor->keys[i] << " ";
        cout << endl;
        if (cursor->isLeaf != true) {
            for (int i = 0; i < cursor->ptr2TreeOrData.ptr2Tree.size(); i++)
                display(cursor->ptr2TreeOrData.ptr2Tree[i]);
        }
    }
    */

    /*
        Level Order Display
    */
    if (cursor == NULL) return;
    queue<BNode*> q;
    q.push(cursor);

    while (!q.empty()) {
        int sz = q.size();
        for (int i = 0; i < sz; i++) {
            BNode* u = q.front(); q.pop();

            //printing keys in self
            for (int val : u->keys)
                cout << val << " ";

            cout << "|| ";//to seperate next adjacent nodes

            if (u->isLeaf != true) {
                for (BNode* v : u->ptr2TreeOrData.ptr2Tree) {
                    q.push(v);
                }
            }
        }
        cout << endl;
    }
}

void BPTree::seqDisplay(BNode* cursor) {
    BNode* firstLeft = firstLeftNode(cursor);

    if (firstLeft == NULL) {
        cout << "No Data in the Database yet!" << endl;
        return;
    }
    while (firstLeft != NULL) {
        for (int i = 0; i < firstLeft->keys.size(); i++) {
            cout << firstLeft->keys[i] << " ";
        }

        firstLeft = firstLeft->ptr2next;
    }
    cout << endl;
}

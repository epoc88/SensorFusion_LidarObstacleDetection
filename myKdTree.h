#ifndef MYKDTREE_H
#define MYKDTREE_H

#include <pcl/common/common.h>
#include <vector>
#include <unordered_set>

struct Node
{
    pcl::PointXYZI point;
    int id;
    Node* left;
    Node* right;

    Node(pcl::PointXYZI arr, int setId)
    : point(arr), id(setId), left(NULL), right(NULL)
    {}
};

class MyKdTree
{
public:
    //constructor
    MyKdTree();
    //deconstructor
    ~MyKdTree();

    std::vector<int> search(pcl::PointXYZI target, float distanceTolerance);
    void insert(pcl::PointXYZI point, int id);

private:
    Node* _root;
    void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id);
    void searchHelper(pcl::PointXYZI target,Node* node,int depth, float distanceTolerance, std::vector<int>& ids);
};


#endif // MYKDTREE_H

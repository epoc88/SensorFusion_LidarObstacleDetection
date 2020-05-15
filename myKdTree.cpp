#include "myKdTree.h"


// ------------------ Public members ------------------------------------------
MyKdTree::MyKdTree()
: _root(NULL)
{
}

MyKdTree::~MyKdTree()
{
}

// Return a list of point ids in the tree within the distance of target
std::vector<int> MyKdTree::search(pcl::PointXYZI target, float distanceTolerance)
{
    std::vector<int> ids;
    searchHelper(target, _root, 0, distanceTolerance, ids);
    return ids;
}

// To insert a new point to the tree
// the function should create a new node and place correctly with in the root
void MyKdTree::insert(pcl::PointXYZI point, int id)
{
    insertHelper(&_root, 0, point, id);
}

// ------------------ Private members ------------------------------------------
void MyKdTree::insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
{
    if(*node == NULL)
    {
        *node = new Node(point, id);
    }
    else
    {
        int chkdepth = depth % 3;

        if(chkdepth==0)    // To check bounbdary conditions on both sides of the node
        {
            if(point.x < ((*node)->point.x))
            {
                insertHelper((&(*node)-> left), depth+1, point, id);    //left side
            }
            else
            {
                insertHelper((&(*node)-> right), depth+1, point, id);   //right side
            }
        }
        else
        {
            if(point.y < ((*node)->point.y))
            {
                insertHelper((&(*node)-> left), depth+1, point, id);    //left side
            }
            else
            {
                insertHelper((&(*node)-> right), depth+1, point, id);   //right side
            }
        }
    }
}


void MyKdTree::searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTolerance, std::vector<int>& ids)
{
    int chkdepth=depth % 3;
    if (node!= NULL)
    {
        if ((node->point.x>= (target.x-distanceTolerance)&& node->point.x<=(target.x+distanceTolerance))&&(node->point.y>=(target.y-distanceTolerance)&& node->point.y<=(target.y+distanceTolerance))&&(node->point.z>=(target.z-distanceTolerance)&& node->point.z<=(target.z+distanceTolerance)))
        {
            // Calculate the euclidean distance between the points
            float caldistance = sqrtf((node->point.x-target.x)*(node->point.x-target.x)+(node->point.y-target.y)*(node->point.y-target.y)+(node->point.z-target.z)*(node->point.z-target.z));

            if(caldistance <= distanceTolerance)
            {
                ids.push_back(node->id);
            }
        }
            // To check bounbdary conditions on both sides of the target node
            // left side:  target-distolerance
            // right side: target+distolerance
        if(chkdepth==0)
        {
            if(target.x-distanceTolerance< node->point.x) // Search left side
            {                 
                searchHelper(target, node->left, depth+1, distanceTolerance, ids);
            }
            if(target.x+distanceTolerance> node->point.x) // Search right side
            {                 
                searchHelper(target, node->right, depth+1, distanceTolerance, ids);
            }
        }
        else
        {
            if(target.y-distanceTolerance< node->point.y) // Search left side
            {                 
                searchHelper(target, node->left, depth+1, distanceTolerance, ids);
            }
            if(target.y+distanceTolerance> node->point.y) // Search right side
            {                 
                searchHelper(target, node->right, depth+1, distanceTolerance, ids);
            }
        }
    }
}



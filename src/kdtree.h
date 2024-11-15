#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
    pcl::PointXYZI point;
    int id;
    Node* left;
    Node* right;

    Node(pcl::PointXYZI point, int setId)
    : point(point), id(setId), left(NULL), right(NULL) {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct KdTree
{
    Node* root;

    KdTree()
    : root(NULL) {}

    ~KdTree()
    {
        delete root;
    }

    void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
    {
        if (*node == NULL)
        {
            *node = new Node(point, id);
        }
        else
        {
            uint cd = depth % 3; // Use 3 for 3D points

            if (point.data[cd] < ((*node)->point.data[cd])) // Use .data for pcl::PointXYZI
                insertHelper(&((*node)->left), depth + 1, point, id);
            else
                insertHelper(&((*node)->right), depth + 1, point, id);
        }
    }

    void insert(pcl::PointXYZI point, int id)
    {
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
        if (node != NULL)
        {
            // Check if the current node is within the distance tolerance
            if (node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol) &&
                node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol) &&
                node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol)) {
                
                float distance = sqrt(pow(node->point.x - target.x, 2) +
                                      pow(node->point.y - target.y, 2) +
                                      pow(node->point.z - target.z, 2));
                if (distance <= distanceTol) {
                    ids.push_back(node->id);
                }
            }

        // Check the left and right branches
        if ((depth % 3 == 0 && (target.x - distanceTol) < node->point.x) || 
            (depth % 3 == 1 && (target.y - distanceTol) < node->point.y) || 
            (depth % 3 == 2 && (target.z - distanceTol) < node->point.z)) {
            searchHelper(target, node->left, depth + 1, distanceTol, ids);
        }


        if ((depth % 3 == 0 && (target.x + distanceTol) > node->point.x) || 
            (depth % 3 == 1 && (target.y + distanceTol) > node->point.y) || 
            (depth % 3 == 2 && (target.z + distanceTol) > node->point.z)) {
            searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
        }
    }

    // Return a list of point ids in the tree that are within distance of target
    std::vector<int> search(pcl::PointXYZI target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};

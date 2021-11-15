/**************************************************************************************************
 * Implementation of the TAD Node
**************************************************************************************************/

#ifndef NODE_H_INCLUDED
#define NODE_H_INCLUDED
#include "Edge.h" // Include of the Edge class

using namespace std;

// Definition of the Node class
class Node{

    // Attributes
    private:
        Edge* first_edge;
        Edge* last_edge;
        int total_edge;
        int id;
        unsigned int in_degree;
        unsigned int out_degree;
        float weight;
        Node* next_node;
        int number;
        int cor;
    public:
        // Constructor
        Node(int id);
        // Destructor
        ~Node();
        // Getters
        Edge* getFirstEdge();
        Edge* getLastEdge();
        int getId();
        int getInDegree();
        int getOutDegree();
        float getWeight();
        Node* getNextNode();
        // Setters
        void setNextNode(Node* node);
        void setWeight(float weight);
        // Other methods
        bool searchEdge(int target_id);
        void insertEdge(int target_id, float weight);
        void removeAllEdges();
        int removeEdge(int id, bool directed, Node* target_node);
        void incrementOutDegree();
        void decrementOutDegree();
        void incrementInDegree();
        void decrementInDegree();
        Edge* hasEdgeBetween(int target_id);
        // Auxiliar methods
        void set_Cor(int i);
        int get_Cor();
        void setNumber(int number);
        void getNumber();
};

#endif // NODE_H_INCLUDED
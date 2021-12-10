/**************************************************************************************************
 * Implementation of the TAD Edge
**************************************************************************************************/

#ifndef EDGE_H_INCLUDED
#define EDGE_H_INCLUDED

using namespace std;

// Definition of the Edge class
class Edge{

    // Attributes
    private:
        int target_id; // id of the edge;
        Edge* next_edge; // set the next edge to the actual edge;
        float weight; // set the weight of the edge;
        int total_edges;
        int idNode; /// posi��o equivalente no vetor;

    public:
        // Constructor
        Edge(int target_id, int idNode); // set the id of the edge in the class;
        // Destructor
        ~Edge();
        // Getters
        int getTargetId(); // return the id of the edge;
        Edge* getNextEdge(); // return the next edge of the actual edge;
        float getWeight(); // return the weight of the actual edge;
        int getTargetIdNode();
        // Setters
        void setNextEdge(Edge* edge); // set the next edge to the actual edge;
        void setWeight(float weight); // set the weight of the actual edge;
        int total_edge();

};

#endif // EDGE_H_INCLUDED

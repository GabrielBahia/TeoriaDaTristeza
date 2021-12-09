#include "Edge.h"
#include <iostream>

using namespace std;

/**************************************************************************************************
 * Defining the Edge's methods
**************************************************************************************************/

// Constructor
Edge::Edge(int target_id, int idNode){

    this->target_id = target_id; // setting the id in the edge
    this->next_edge = nullptr; // setting the next edge as null;
    this->weight = 0; // setting the weight as 0;
    this->idNode = idNode;
}

// Destructor
Edge::~Edge(){

    while(this->next_edge != nullptr){
        delete this->next_edge;
        this->next_edge = nullptr;
    }

}

// Getters
int Edge::getTargetId(){

    return this->target_id; // return the id of the edge;

}

int Edge::getTargetIdNode() {
    return this->idNode; // posi��o equivalente no vetor
}


Edge* Edge::getNextEdge(){

    return this->next_edge; // return the next edge of the actual edge;

}

float Edge::getWeight(){

    return this->weight; // return the weight of the edge;

}

// Setters
void Edge::setNextEdge(Edge* edge){

    this->next_edge = edge; // set the next edge to the actual edge;
}

void Edge::setWeight(float weight){

    this->weight = weight; // set the weight to the actual edge;

}


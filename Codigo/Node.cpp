#include "Node.h"
#include "Edge.h"
#include <iostream>

using namespace std;

/**************************************************************************************************
 * Defining the Node's methods
**************************************************************************************************/

// Constructor
Node::Node(int id){

    this->id = id; // id of the node!;
    this->in_degree = 0; //
    this->out_degree = 0; //
    this->weight = 0; // weigth of the node!;
    this->first_edge = nullptr; // set the first edge as null!;
    this->last_edge = nullptr; // set the last edge as null!;
    this->next_node = nullptr; // set the next node as null!;

};

// Destructor
Node::~Node(){

    Edge* next_edge = this->first_edge;

    while(next_edge != nullptr){

        Edge* aux_edge = next_edge->getNextEdge();
        delete next_edge;
        next_edge = aux_edge;

    }

};

// Getters
Edge* Node::getFirstEdge(){

    return this->first_edge; // return the first edge!;

}

Edge* Node::getLastEdge(){

    return this->last_edge; // return the last edge!;

}

int Node::getId(){

    return this->id; // return the id of the edge!;

}

int Node::getInDegree(){

    return this->in_degree; //

}

int Node::getOutDegree(){

    return this->out_degree; //

}

float Node::getWeight(){

    return this->weight; // return the weight of the node!;

}

Node* Node::getNextNode(){

    return this->next_node; // return the next node!;

}

// Setters

void Node::setNextNode(Node* next_node){

    this->next_node = next_node; // set the next node to the node that is calling the function!;

}

void Node::setWeight(float weight){

    this->weight = weight; // set the weigth to the node that is calling the function!;

}

// Other methods
void Node::insertEdge(int target_id, float weight){
    // Verifies whether there are at least one edge in the node
    if(this->first_edge != nullptr){ // if the first edge isn't null or in other words if the graph is not empty do
        // Allocating the new edge and keeping the integrity of the edge list
        Edge* edge = new Edge(target_id); //
        edge->setWeight(weight);
        this->last_edge->setNextEdge(edge);
        this->last_edge = edge;

    }
    else{
         // Allocating the new edge and keeping the integrity of the edge list
        this->first_edge = new Edge(target_id);
        this->first_edge->setWeight(weight);
        this->last_edge = this->first_edge;

    }

}

void Node::removeAllEdges(){
    // Verifies whether there are at least one edge in the node
    if(this->first_edge != nullptr){

        Edge* next = nullptr;
        Edge* aux = this->first_edge;
        // Removing all edges of the node
        while(aux != nullptr){

            next = aux->getNextEdge();
            delete aux;

        }

    }

    this->first_edge = this->last_edge = nullptr;

}

int Node::removeEdge(int id, bool directed, Node* target_node){
    // Verifies whether the edge to remove is in the node
    if(this->searchEdge(id)){ // if the edge is in the node do

        Edge* aux = this->first_edge;
        Edge* previous = nullptr;
        // Searching for the edge to be removed
        while(aux->getTargetId() != id){

            previous = aux;
            aux = aux->getNextEdge();

        }
        // Keeping the integrity of the edge list
        if(previous != nullptr) // what means that the while occurred at least one
            previous->setNextEdge(aux->getNextEdge());

        else
            this->first_edge = aux->getNextEdge(); // the while didnt occurred any time so the first edge recieves null so that we can delete the actual node

        if(aux == this->last_edge)
            this->last_edge = previous; //the while didnt occurred any time so the first edge recieves null so that we can delete the actual node

        if(aux->getNextEdge() == this->last_edge)
            this->last_edge = aux->getNextEdge(); // again if the edge that we want to delete is null we will pass the last edge as null

        delete aux; // deleting the node that we want!;
        // Verifies whether the graph is directed
        if(directed) // verifies if the graph if directed!;
            this->decrementOutDegree();

        else{

            this->decrementInDegree();
            target_node->decrementInDegree();

        }

        return 1;

    }

    return 0;

}

bool Node::searchEdge(int target_id){
     // Verifies whether there are at least one edge in the node
    if(this->first_edge != nullptr){ // in other words verifies if the graph is not empty
        // Searching for a specific edge of target id equal to target id
        for(Edge* aux = this->first_edge; aux != nullptr; aux = aux->getNextEdge())
            if(aux->getTargetId() == target_id)
                return true;

    }

    return false;

}

void Node::incrementInDegree(){

    this->in_degree++; // incrementing the degree of the graph????

}

void Node::incrementOutDegree(){

    this->out_degree++; // incrementing the degree of the graph????

}

void Node::decrementInDegree(){

    this->in_degree--;

}

void Node::decrementOutDegree(){

    this->out_degree--;

}

Edge* Node::hasEdgeBetween(int target_id)
{

    for(Edge *auxEdge = this->first_edge; auxEdge != nullptr; auxEdge = auxEdge->getNextEdge())
    {
        if(auxEdge->getTargetId() == target_id)
            return auxEdge;
    }
    return nullptr;
}

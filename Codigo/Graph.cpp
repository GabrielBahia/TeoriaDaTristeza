#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <iomanip>

using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{

    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->number_edges = 0;
}

// Destructor
Graph::~Graph()
{

    Node *next_node = this->first_node;

    while (next_node != nullptr)
    {

        next_node->removeAllEdges();
        Node *aux_node = next_node->getNextNode();
        delete next_node;
        next_node = aux_node;
    }
}

// Getters
int Graph::getOrder() // return the Order of the graph!;
{
    return this->order;
}
int Graph::getNumberEdges() // return the number of edges in the graphic!;
{
    return this->number_edges;
}
//Function that verifies if the graph is directed!;
bool Graph::getDirected() // return if the graphic is directed!;
{
    return this->directed;
}
//Function that verifies if the graph is weighted at the edges!;
bool Graph::getWeightedEdge() // return if the graphic have weight at the edges!;
{
    return this->weighted_edge;
}

//Function that verifies if the graph is weighted at the nodes
bool Graph::getWeightedNode() // return if the graphic have weight at the nodes!;
{
    return this->weighted_node;
}


Node *Graph::getFirstNode() // return the first node of the graph!;
{
    return this->first_node;
}

Node *Graph::getLastNode() // return the last node of the graph!;
{
    return this->last_node;
}


///////////////////////////////////////////////////////////////////
// Other methods
/*
    The outdegree attribute of nodes is used as a counter for the number of edges in the graph.
    This allows the correct updating of the numbers of edges in the graph being directed or not.
*/
void Graph::insertNode(int id)
{
    Node node = new Node(id);
    if(order == 0) // if there are no nodes in the graph
    {
        this->first_node = this->last_node = node; // both of them receive the new node;
        order++; // increase the order of the graph
    }
    else { // if there are more than 0 nodes in the graph
        last_node->setNextNode(node); // set the next node to the new node;
        last_node = node; // set the last node as the new node;
    }
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    if(order == 0) // if there are no nodes in the graph;
    {
        this->first_node // i don´t know already where the int id in the function goes;
    }
}

void Graph::removeNode(int id)
{

    if(searchNode(id)) // searching if the node is in the graph;
    {
        Node *node = getNode(id); // new id receiving the target node;
        Node *previous = this->first_node; // setting new node as first node;
        if(previous;previous->getNextNode()!=node;previous = previous->getNextNode()) // just looking for the node;
        {
        }
        if(node == previous ) // if the node i want is equals previous so it is the first node;
        {
            if(node == this->last_node) // if the graph only have one node so the first node and the last node are the same
            {
                delete previous;
                first_node = last_node = nullptr;
                order--;
            }
            else
            {
                previous = previous->getNextNode();
                first_node = previous;
                delete previous;
                order--;
            }
        }
        else // the first node is not the node that we found
        {
            previous->setNextNode(node->getNextNode())
            if(last_node == node) // verifying if the node that we found is not the last node;
            {
                last_node = nullptr;
            }
            delete node;
            order--;
        }
    }

}

bool Graph::searchNode(int id)
{
    Node *node = this->first_node; // first node to the actual node
    for( node; node!= nullptr; node = node->getNextNode() ) // searching for the node if this id;
    {
        if(node->getId() == id) // found the node if this id
        {
            return true;
        }
    }
    return false; // didnt found the node if this id;
}

Node *Graph::getNode(int id)
{
    Node *node = this->first_node; // first node to the actual node
    for( node; node!= nullptr; node = node->getNextNode() )  // searching for the node
    {
        if(node->getId() == id) // if it finds it returns the node
        {
            return node;
        }
    }
    return nullptr; // else it returns nullptr;
}



/// ALL THIS FUNCTIONS HERE WE DONT KNOW ALREADY

//Function that prints a set of edges belongs breadth tree

void Graph::breadthFirstSearch(ofstream &output_file){

}



float Graph::floydMarshall(int idSource, int idTarget){

}



float Graph::dijkstra(int idSource, int idTarget){

}

//function that prints a topological sorting
void topologicalSorting(){

}

void breadthFirstSearch(ofstream& output_file){

}
Graph* getVertexInduced(int* listIdNodes){

}

Graph* agmKuskal(){

}
Graph* agmPrim(){

}

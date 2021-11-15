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
#include <algorithm> /// função find
using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{

    this->order = order; // numbers of nodes
    this->number_edges = 0; // number of edges
    this->directed = directed; // if it's directed;
    this->weighted_edge = weighted_edge; // if it has weight on its edges
    this->weighted_node = weighted_node; // if it has weight on its nodes
    this->first_node = this->last_node = nullptr; // first and last node starts as null cause theres is nothing in the start
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
    Node *node = new Node(id);
    Node->setNumber(order+1);
    if(order == 0) // if there are no nodes in the graph
    {
        this->first_node = this->last_node = node; // both of them receive the new node;
    }
    else { // if there are more than 0 nodes in the graph
        last_node->setNextNode(node); // set the next node to the new node;
        last_node = node; // set the last node as the new node;
    }
        order++; // increase the order of the graph
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    if(searchNode(id) && searchNode(target_id)) // search if the two nodes are in the graph
    {
        Node *node = getNode(id); // search the actual node that's being called;
        if(!node->hasEdgeBetween(target_id)) // return if theres is no edge between the node id and the node target_id
        {
            node->insertEdge(target_id,weight);  // inserts the edge between the two nodes
        }
    }
}

void Graph::removeNode(int id)
{

    if(searchNode(id)) // searching if the node is in the graph;
    {
        Node *node = getNode(id); // new id receiving the target node;
        Node *previous = this->first_node; // setting new node as first node;
        for(previous;previous->getNextNode()!=node;previous = previous->getNextNode()) // just looking for the node;
        {
            cout << "";
        }
        if(node == previous ) // if the node i want is equals previous so it is the first node;
        {
            if(node == this->last_node) // if the graph only have one node so the first node and the last node are the same
            {
                delete previous;
                first_node = last_node = nullptr;
            }
            else
            {
                previous = previous->getNextNode();
                first_node = previous;
                delete previous;
            }
        }
        else // the first node is not the node that we found
        {
            previous->setNextNode(node->getNextNode());
            if(last_node == node) // verifying if the node that we found is not the last node;
            {
                last_node = nullptr;
            }
            delete node;
        }
        order--;
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

void Graph::breadthFirstSearch(ofstream &output_file, int id_inicial){ /// No parametro dessa função, não deveria ser o id?
    if(searchNode(id_inicial))
    {
        Node *node = getNode(id_inicial);
        int total = this->order - node->getNumber(); /// total of nodes in the graph;
        bool visitado = new bool[total];
        for(int i = 0;i<total ;i++)
        {
            node->set_Cor(i);
            visitado[x] = 0;
            node = node->getNextNode();
        }
        node = getNode(id_inicial);
        while(total > 0)
        {
            Node *aux_node = node;
            int total_edge = node->total_edge;
            if(visitado[node->get_Cor()] == 0)
            {
                cout << node->getId();
                visitado[node->get_Cor()] = 1;
            }
            Edge *edge = node->getFirstEdge();
            while(edge;edge!= nullptr;edge = node->getNextEdge()){
                cout << edge->getTargetId();
                visitado[aux_node->get_Cor] = 1;
            }
            node =
            total--;
        }
    }

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

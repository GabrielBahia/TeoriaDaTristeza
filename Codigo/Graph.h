/**************************************************************************************************
 * Implementation of the TAD Graph
**************************************************************************************************/

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED
#include "Node.h"
#include <fstream>
#include <stack>
#include <list>

using namespace std;

class Graph{

    //Atributes
    private:
        int order; // number of nodes in the graph;
        int number_edges; // number of the edges in the graph;
        bool directed; // directed graph?;
        bool weighted_edge; // weighted edge?;
        bool weighted_node; // weighted node?;
        Node* first_node; // first node of the graph;
        Node* last_node; // last node of the graph;
        bool negative_edge;

    public:
        //Constructor
        Graph(int order, bool directed, bool weighted_edge, bool weighted_node);
        Graph(bool directed, bool weighted_edge, bool weighted_node);
        //Destructor
        ~Graph();
        //Getters
        int getOrder();
        int getNumberEdges(); // return the number of edges in the graph(int);
        bool getDirected(); // return if the graph is directed or not(boolean);
        bool getWeightedEdge(); // return if the edges have weight(boolean);
        bool getWeightedNode(); // return if the nodes have weight(boolean);
        Node* getFirstNode(); // return the first node of the graph(node);
        Node* getLastNode(); // return the last node of the graph(node);
        //Other methods
        void insertNode(int id); // incrementing a node in the graph;
        void insertEdge(int id, int target_id, float weight); // incrementing an edge in the graph;
        void removeNode(int id); // removing a node of the graph;
        bool searchNode(int id); // searching for a node in the graph(boolean);
        Node* getNode(int id); // getting a node of the graph(node);
        Node* getNodeId(int id);

        //methods phase1
        void topologicalSorting(); // don�t know yet
        void breadthFirstSearch(ofstream& output_file); // busca em largura;
        bool deephFirstSearch1(int id, int start); // busca em profundidade; � chamada pela fecho transitivo indireta
        void fechoTransitivoDireto(ofstream &output_file, int id); // fecho transitivo direto;
        void fechoTransitivoIndireto(ofstream &output_file, int id);
        Graph* getVertexInduced(int* listIdNodes); // don�t know yet;
        Graph* agmKuskal(); // don�t know yet;
        Graph* agmPrim(); // don�t know yet;
        float floydMarshall(int idSource, int idTarget); // don�t know yet;
        float dijkstra(int idSource, int idTarget); // don�t know yet;
        int **floyd(int tam, int **dist);
        //fun��o auxiliar
        void auxDeepthFirstSearch1(bool verify[], Node *v);
        bool graphCiclo();
        Graph *getVertexInduced(int *listIdNodes, int tam);
        void getWeithlessEdge(int *nohAresta);
        bool verificaSubarvore(int v1, int v2, Graph *subGrafo);
        int getWeightFromEdgeNodeCombo(int idNoh, int idAresta, Graph *subGrafo);
        //methods phase1
        float greed(); // don�t know yet;
        float greedRandom(); // don�t know yet;
        float greedRactiveRandom(); // don�t know yet;

};

#endif // GRAPH_H_INCLUDED

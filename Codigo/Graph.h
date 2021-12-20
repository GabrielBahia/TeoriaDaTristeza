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
        //Methods
        void insertNode(int id, int weight); // incrementing a node in the graph;
        void insertEdge(int id, int target_id, float weight); // incrementing an edge in the graph;
        void removeNode(int id); // removing a node of the graph;
        bool searchNode(int id); // searching for a node in the graph(boolean);
        Node* getNode(int id); // getting a node of the graph(node);
        Node* getNodeId(int id); // pegando o equivalente de cada node no indice de um vetor;

        //Methods 
        void fechoTransitivoDireto(ofstream &output_file, int id); // fecho transitivo direto; int id é o node inicial
        void fechoTransitivoIndireto(ofstream &output_file, int id); // fecho transitivo indireto; int id é o node inicial
        int dijkstra(int idSource, int idTarget); // dijkstra
        void floydMarshall(ofstream &output_file, int idSource, int idTarget); // floyd;
        Graph* agmPrim(ofstream &output_file); // algoritmo de Prim
        Graph* agmKuskal(ofstream &output_file); // algoritmo de kuskal
        void breadthFirstSearch(ofstream& output_file); // busca em largura;
        void ordenacaoTopologica(ofstream &output_file); // ordenação topologica 
        
        
        //função auxiliar
        bool deephFirstSearch(int id, int start); // busca em profundidade; � chamada pela fecho transitivo indireta
        void auxDeepthFirstSearch(bool verify[], Node *v);
        int **floyd(int tam, int **dist); 
        Graph *getVertexInduced(int *listIdNodes, int tam);
        void getWeithlessEdge(int *nohAresta);
        bool verificaSubarvore(int v1, int v2, Graph *subGrafo);
        int getWeightFromEdgeNodeCombo(int idNoh, int idAresta, Graph *subGrafo);
        void printGraph(ofstream &output_file);
        bool graphTemCiclo();
};

#endif // GRAPH_H_INCLUDED

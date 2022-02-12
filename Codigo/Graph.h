/**************************************************************************************************
 * Implementation of the TAD Graph
**************************************************************************************************/

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED
#include "Node.h"
#include <vector>
#include <fstream>
#include <stack>
#include <list>

using namespace std;

class Graph{

    //Atributes
    private:
        int order; // number of nodes in the graph;
       // int number_edges; 
        int number_edges;// number of the edges in the graph;
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

        //Methods Segunda Etapa
        void Guloso(ofstream &output_file, int p, int numIter);
        void GulosoRandomizado(ofstream &output_file, int p, float alfa, int numIter);
        void GulosoRandomizadoReativo(ofstream &output_file, int p, float *alfa, int numIter, int blocoIter, int m);

        //Funções Auxiliares Segunda Etapa
        void atualizaProbabilidades(ofstream &output_file, float *vetProbAlfa, float *qAlfa, float melhorGap, float *medias, int m);
        int escolheAlfa(ofstream &output_file, float *vetProbAlfa, int m);
        vector<Node*> criaVectorTeste();
        vector<Node> *criaVector();
        vector<float> *criaVetorRank(int p);
        vector<int> *criaVetorMaiorMenor();
        void *verificaVizinhos(Node *node, bool *verifica, int cor);
        void teste(ofstream &output_file);
        //vector<float> geraRank(vector<vector<Node>> vectorCluster, int idCluster, vector<int> nodeWeight, vector<int> nodeGrau );
        //void getMaiorMenorVal(float *maiorValor, float *menorValor, vector<Node> vectorNode, int i ,int p);
};

#endif // GRAPH_H_INCLUDED

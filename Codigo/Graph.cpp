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
        if(this->directed)
        {
            Node *node = getNode(id); // search the actual node that's being called;
            if(!node->hasEdgeBetween(target_id)) // return if theres is no edge between the node id and the node target_id
            {
                node->insertEdge(target_id,weight);  // inserts the edge between the node we are to the node targeted
            }
        }
        else
        {
            Node *node = getNode(id); // search the actual node that's being called;
            if(!node->hasEdgeBetween(target_id)) // return if theres is no edge between the node id and the node target_id
            {
                node->insertEdge(target_id,weight);  // inserts the edge between the two nodes
                Node *aux = getNode(target_id);
                aux->insertEdge(node->getId(),node->getWeight()); // inserts the edge between the two nodes;
            }
        }
    }
}

void Graph::removeNode(int id)
{
    if(this->first_node != NULL) // graph not empty
    {
        if(searchNode(id)) // node is in the graph
        {
            if(this->first_node == this->last_node) // theres only one node in the graph and the node we want do delete is in the graph
            {
                this->first_node = NULL;
                this->last_node = NULL;
                order--;
            }
            else // theres more than only one node and the node we want to delete is in the graph
            {
                Node *node = getNode(id); // new id receiving the target node;
                Node *previous = this->first_node; // setting new node as first node;
                for(previous;(previous->getNextNode()!=node || previous == node);previous = previous->getNextNode()) // just looking for the node before the targetedNode;
                {
                    cout << "";
                }
                if(node == previous ) // if the node i want is equals previous so it is the first node;
                {
                    previous = previous->getNextNode();
                    first_node = previous;
                    delete previous;
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
                previous = this->first_node; // passando o primeiro vertice para o previous
                while(previous != nullptr) // enquanto previous não chegar no ultimo vertice
                {
                    previous->removeEdge(id,this->directed,previous); // vai chamando todos os vertices e verificando se eles tem aresta com o id(vertice que a gente quer excluir)
                    previous = previous->getNextNode(); // passa o previous para o prox vertice
                }
                node->removeAllEdges(); // remove todas as arestas do vertice que a gente vai deletar, não sei se é necessario já que a gente já vai deletar ele mesmo, mas fica ai
                delete node; // deleta o node que a gente quer
                delete previous; // deleta o previous que usamos na função
                order--; // diminui a ordem do grafo ou seja o número de vértices presente nele, já que excluimos um vértice;
            }
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

void Graph::breadthFirstSearch(ofstream &output_file, int id_inicial){ /// No parametro dessa função, não deveria ser o id?


}

/// ATUAL

void Graph::fechoTransitivoDireto(ofstream &output_file, int id)
{

    //com o id do vértice acha o vertice que deve ser analisado
    int idParametro = id - 1; // vai pegar a posição exata em um vetor, pois os vetores começam do 0 e possivelmente os vertices do ed1
    //cria um vetor que marca quais vértices ja foram analisados
    bool visitados[this->order];
    //cria o vetor fecho transitivo direto
    bool FTD[this->order];
    //cria uma fila que diz quais vertices ainda precisam ser analisados
    queue<int> fila;
    //adiciona o vertice inicial nele
    fila.push_front(id);
    ordem.push_front(id);

    for (int i = 0; i < this->order; i++)
    {
        visitados[i] = false;
        FTD[i] = false;
    }

    //começa iteração (enquanto a fila não estiver vazia repita)
    while (!(fila.empty()))
    {
        //pega um vértice a ser analisado da fila
        int aux = fila.front();
        int IdAnalisado = aux - 1; // já que os vetores começam da posição 0, isso possivelmente equivale a passar a posição equivalente do id do vertice no vetor
        Node *V = getNode(fila.front());
        ///V = getNode(fila.front());
        //exclui ele da fila
        fila.pop_front();
        //verifica se o vértice a ser analisado ja foi analisado. (se ele ja foi acaba essa iteração)
        if (visitados[IdAnalisado] == false)
        {
            //marca o vértice como visitado;
            visitados[IdAnalisado] = true;
            //adiciona ele no vetor fecho transitivo direto
            FTD[IdAnalisado] = true;
            //adiciona todos os vértices adjacentes a ele na fila
            for (Edge *it = V->getFirstEdge(); it != NULL; it = it->getNextEdge())
            {
                int verticeAdjacente = it->getTargetId(); // aqui ele possivelmente tá passando o id do vertice com o qual it(ou seja V) está ligado pela aresta e que tem como id o vértice alvo
                fila.push_front(verticeAdjacente);

            }
        }
    }

    //imprimir o FTD
    output_file << "O conjunto FTD do vértice " << id << " é: {";
    int contador = 0;
    for (int i = 0; i < this->order; i++)
    {
        if (FTD[i] == true)
        {
            contador++;
        }
    }
    for (int i = 0; i < this->order; i++)
    {
        if (FTD[i] == true)
        {
            if (contador - 1 > 0)
            {
                output_file << i + 1 << ", ";
                contador--;
            }
            else if (contador - 1 == 0)
            {
                output_file << i + 1;
            }
        }
    }
    output_file << "}" << endl;
}

void Graph::fechoTransitivoIndireto(ofstream &output_file, int id)
{
    bool *fti = new bool[this->order];
    bool node = new bool[this->order];
    for(int i =0;i<this->order;i++)
    {
        fti[i] = false;
        verify[i] = false;
    }

    int conta = 0;

    for(Node *p = this->first_node; p!= nullptr; p = p->getNextNode())
    {
        if(!verify[p->getId() - 1])
        {
            verify[p->getId() - 1] = true;
            fti[p->getId() - 1 ] = deepthFirstSearch1(id, p->getId());
            if(fti[p->getId() - 1])
            {
                conta++;
            }
        }
    }

    output_file << "O fecho transitivo indireto de " << id << "é: ";
    output_file << "{";

    int aux = 0;
    for(int i = 0;i < this->order;i++)
    {
        if(fti[i])
        {
            if(aux == conta - 1)
            {
                output_file << (j+1);
                aux++;
            }
            else
            {
                output_file << (j+1) << ",";
                aux++;
            }
        }
    }
}

bool Graph::deephFirstSearch1(int id, int start)
{

    //Criando vetor para verificar e também vetor predecessor de profundidade
    bool *verify = new bool[this->order];
    int conta = 0;
    int idParametro;
    for(int i = 0;i < this->order; i++)
    {
        verify[i] = false;
    }
    // cria vetor para auxiliar
    Node *p;

    //Para todo v em G;
    p = getNode(start);
    idParametro = p->getId() - 1;
    //Se v não visitado então

    if(id != p->getId())
    {
        //Aux-BuscaEmProfundida(G,v);
        auxDeephFirstSearch1(verify, p);
    }
    else
    {
        return true;
    }

    //Se encontrou
    if(verify[id - 1])
    {
        delete[] verify;
        return true;
    }
    delete[] verify;
    return false;
}

void Graph::auxDeepthFirstSearch1(bool verify[], Node *v, bool verify2[])
{
    //Protocolo inicial
    int idParametro = v->getId() - 1;

    Node *aux;
    //Marca v como visitado;

    verify[idParametro] = true;

    //Para todo w em Adj(v)
    for(Edge *p = v->getFirstEdge(); p != NULL; p = p->getNextEdge())
    {
        id Parametro = p->getTargetId() - 1;
        //Se w não visitado então

        if(!verify[idParametro])
        {

            aux = getNode(p->getTargetId());
            //AuxBuscaEmProfundidade(G,w);
            auxDeephFirstSearch1(verify, aux);
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

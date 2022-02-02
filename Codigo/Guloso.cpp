#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <iomanip>
#include <iterator>
#include <algorithm> /// fun��o find
using namespace std;
#define INFINITO 1000000000;
#include <limits.h>
/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{

    this->order = 0;                              // numbers of nodes
    this->number_edges = 0;                       // number of edges
    this->directed = directed;                    // if it's directed;
    this->weighted_edge = weighted_edge;          // if it has weight on its edges
    this->weighted_node = weighted_node;          // if it has weight on its nodes
    this->first_node = this->last_node = nullptr; // first and last node starts as null cause theres is nothing in the start
    this->negative_edge = false;
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

void Graph::insertNode(int id, int weight)
{
    Node *node = new Node(id, order);
    node->setWeight(weight);
    if(!searchNode(id))
    {
         
        if (order == 0) // if there are no nodes in the graph
        {
            this->first_node = this->last_node = node; // both of them receive the new node;
        }
        else
        {                                 // if there are more than 0 nodes in the graph
            last_node->setNextNode(node); // set the next node to the new node;
            last_node = node;             // set the last node as the new node;
        }
        order++; // increase the order of the graph
    }

}

void Graph::insertEdge(int id, int target_id, float weight)
{
    if (searchNode(id) && searchNode(target_id)) // search if the two nodes are in the graph
    {
        if (weight < 0)
            this->negative_edge = true; // verifica se o peso da aresta � negativa

        if (this->directed)
        {
            Node *node = getNode(id);             // search the actual node that's being called;
            if (!node->hasEdgeBetween(target_id)) // return if theres is no edge between the node id and the node target_id
            {
                Node *target = getNode(target_id);
                node->insertEdge(target_id, weight, target->getIdNode()); // inserts the edge between the node we are to the node targeted 
                target->incrementInDegree();
                this->number_edges++;
            }
        }
        else
        {
            Node *node = getNode(id);             // search the actual node that's being called;
            if (!node->hasEdgeBetween(target_id)) // return if theres is no edge between the node id and the node target_id
            {
                Node *aux = getNode(target_id);
                node->insertEdge(target_id, weight, aux->getIdNode()); // inserts the edge between the two nodes
                aux->insertEdge(node->getId(), weight, node->getIdNode()); // inserts the edge between the two nodes;
                this->number_edges++; 
                node->incrementInDegree();
                aux->incrementInDegree();
            }
        }
    }
}

void Graph::removeNode(int id)
{
    if (this->first_node != NULL) // graph not empty
    {
        if (searchNode(id)) // node is in the graph
        {
            if (this->first_node == this->last_node) // theres only one node in the graph and the node we want do delete is in the graph
            {
                this->first_node = NULL;
                this->last_node = NULL;
                order--;
            }
            else // theres more than only one node and the node we want to delete is in the graph
            {
                Node *node = getNode(id);                                                                                 // new id receiving the target node;
                Node *previous = this->first_node;                                                                        // setting new node as first node;
                for (previous; (previous->getNextNode() != node || previous == node); previous = previous->getNextNode()) // just looking for the node before the targetedNode;
                {
                    cout << "";
                }
                if (node == previous) // if the node i want is equals previous so it is the first node;
                {
                    previous = previous->getNextNode();
                    first_node = previous;
                    delete previous;
                }
                else // the first node is not the node that we found
                {
                    previous->setNextNode(node->getNextNode());
                    if (last_node == node) // verifying if the node that we found is not the last node;
                    {
                        last_node = nullptr;
                    }
                    delete node;
                }
                previous = this->first_node; // passando o primeiro vertice para o previous
                while (previous != nullptr)  // enquanto previous n�o chegar no ultimo vertice
                {
                    previous->removeEdge(id, this->directed, previous); // vai chamando todos os vertices e verificando se eles tem aresta com o id(vertice que a gente quer excluir)
                    previous = previous->getNextNode();                 // passa o previous para o prox vertice
                }
                node->removeAllEdges(); // remove todas as arestas do vertice que a gente vai deletar, n�o sei se � necessario j� que a gente j� vai deletar ele mesmo, mas fica ai
                delete node;            // deleta o node que a gente quer
                delete previous;        // deleta o previous que usamos na fun��o
                order--;                // diminui a ordem do grafo ou seja o n�mero de v�rtices presente nele, j� que excluimos um v�rtice;
                if (order > 0)
                {
                    for (Node *aux = this->first_node; aux != NULL; aux = aux->getNextNode())
                    {
                        if (aux->getIdNode() == 0)
                        {
                        }
                        else
                        {
                            aux->setIdNode(aux->getIdNode()-1);
                        }
                    }
                }
            }
        }
    }
}

bool Graph::searchNode(int id)
{
    Node *node = this->first_node;                          // first node to the actual node
    for (node; node != nullptr; node = node->getNextNode()) // searching for the node if this id;
    {
        if (node->getId() == id) // found the node if this id
        {
            return true;
        }
    }
    return false; // didnt found the node if this id;
}

Node *Graph::getNode(int id)
{
    Node *node = this->first_node;                          // first node to the actual node
    for (node; node != nullptr; node = node->getNextNode()) // searching for the node
    {
        if (node->getId() == id) // if it finds it returns the node
        {
            return node;
        }
    }
    return nullptr; // else it returns nullptr;
}

Node *Graph::getNodeId(int id)
{
    Node *node = this->first_node;                          // first node to the actual node
    for (node; node != nullptr; node = node->getNextNode()) // searching for the node
    {
        if (node->getIdNode() == id) // if it finds it returns the node
        {
            return node;
        }
    }
    return nullptr; // else it returns nullptr;
}

//----------------------------------------------------------------------------------------------------------


void Graph::Guloso(ofstream &output_file, int p)
{
    bool *visitado = new bool[this->order];  // vetor para verificar os vértices já utilizados
    
    for(int i=0;i<this->order;i++)
    {
        visitado[i] = false; // marcando todos nodes como não visitados
    }

	if(this->weighted_node) // só pode grafo com node com peso
    {
        vector<vector<Node>> *vectorNode = new vector<vector<Node>>; //Note space between "> >" // vetor de vetores de node
        vectorNode->reserve(this->order);
    
        for(int i=0;i<p;i++) {
            vectorNode->push_back(*criaVector()); // criando os vetores de node;
            vectorNode->at(i).reserve(this->order);
        }

        //srand(time(0)); // semente aleatoria

        /*for(int i=0;i<p;i++) 
        {
            Node *nodeAux;
            do {
                int x = 1 + (rand() % this->order-1); // escolhendo número aleatorio
                //output_file << " x : " << x << endl;
                nodeAux = this->getNodeId(x); // pegando node referente a esse número
                //output_file << "Id: " << nodeAux->getId() << " -- " << " Grau entrada: " << nodeAux->getInDegree() << " -- " << " Grau saida: " << nodeAux->getTotal_Edge() << endl;
            } while(visitado[nodeAux->getIdNode()]); // se o node já tiver sido colocado ele troca
            visitado[nodeAux->getIdNode()] = true; // marcando como visitado

            // Verifica se o vértice nodeAux possui algum vizinho de grau 1
            for(Edge *edgeAux = nodeAux->getFirstEdge(); edgeAux != nullptr; edgeAux = edgeAux->getNextEdge())
            {
                if(getNode(edgeAux->getTargetId())->getInDegree() == 1) // verificando se a aresta ao nó escolhido só tem o nó escolhido como vizinho
                {
                    vectorNode->at(i).emplace_back(*getNode(edgeAux->getTargetId())); // Coloca o vizinho de grau 1 na lista
                    visitado[edgeAux->getTargetIdNode()] = true;  // Coloca o node vizinho como já utilizado
                    visitado[nodeAux->getIdNode()] = true; // coloca o node escolhido como já utilizado
                    //output_file << "Esse vertice: " << vectorNode[i].at(0).getId() << endl;
                    output_file << "Entrou no 1" << endl;
                }    

            }

            if(nodeAux->getInDegree() == 1) { // se o nó escolhido tem grau de entrada 1 já pega o vizinho dele junto
                vectorNode->at(i).emplace_back(*nodeAux);  // caso o node só tenha uma aresta a gente vai inserir o único vizinho direto na lista que o vizinho tá
                vectorNode->at(i).emplace_back(*getNode(nodeAux->getFirstEdge()->getTargetId())); // único vizinho direto já pode pegar direto no getFirstEdge()
                visitado[nodeAux->getIdNode()] = true;  // Coloca o node como já utilizado
                visitado[nodeAux->getFirstEdge()->getTargetIdNode()] = true; // coloca o vizinho do node como já utilizado
                //output_file << "Esse vertice: " << vectorNode[i].at(0).getId() << endl;
                //output_file << "Esse vertice: " << vectorNode[i].at(1).getId() << endl;
                output_file << "Entrou no 2" << endl;
            } else {
                vectorNode->at(i).emplace_back(*nodeAux); // inserindo esse node na lista da posição i do vector
                visitado[nodeAux->getIdNode()] = true;   // Coloca o vértice como já utilizado
                //output_file << "Esse vertice: " << vectorNode[i].at(0).getId() << endl;
                output_file << "Entrou no 3" << endl;
            }
        }*/

        vectorNode->at(0).emplace_back(*getNode(0));
        vectorNode->at(1).emplace_back(*getNode(5));
        vectorNode->at(2).emplace_back(*getNode(1));
        vectorNode->at(3).emplace_back(*getNode(14));
        visitado[getNode(0)->getIdNode()] = true;
        visitado[getNode(5)->getIdNode()] = true;
        visitado[getNode(1)->getIdNode()] = true;
        visitado[getNode(14)->getIdNode()] = true;
        vectorNode->at(1).begin()->setCor(1);
        vectorNode->at(2).begin()->setCor(2);
        vectorNode->at(3).begin()->setCor(3);


        for(int q=0;q<vectorNode->size();q++) {
            getNode(vectorNode->at(q).begin()->getId())->setCor(q);
        }

        /*Quando estiver escolhendo node aleatorio usa isso aqui ao inves das linhas de cima
        for(int q=0;q<vectorNode->size();q++) {
            for(int l=0;l<vectorNode->at(q).size();l++) {
                vectorNode->at(q).at(l).setCor(q);
                getNode(vectorNode->at(q).at(l).getId())->setCor(q);
                visitado[vectorNode->at(q).at(l).getIdNode()] = true;
            }
        }*/
     
        /*getNode(0)->setCor(0);
        getNode(5)->setCor(1);*/

        //vector<Node> *vectorWeight = new vector<Node>(); // vector de pesos dos nodes
        //vector<Node> *vectorEdge = new vector<Node>(); // vector de grau dos nodes
        vector<Node> *vectorWeightEdge = new vector<Node>();
        vector<vector<float>> *listRank = new vector<vector<float>>; // vector de ranqueamento dos nodes
        //vectorWeightEdge->reserve(this->order-p);
        vectorWeightEdge->reserve(this->order);
        listRank->reserve(p); // reservando espaço para o total de clusters nesse vector 
        for(int i=0;i<p;i++) {
            listRank->push_back(*criaVetorRank(p)); // alocando vectors em cada posição da listRank
            //listRank->at(i).reserve(this->order-2); // reservando espaço para o total de nodes em cada posição da listRank
            listRank->at(i).reserve(this->order);
        }
        //vectorWeight->reserve(this->order-p); // reservando espaço para o total de nodes nesse vector 
        //vectorEdge->reserve(this->order-p); // reservando espaço para o total de nodes nesse vector 

        //Node *node = this->first_node;

        for(Node *node = this->first_node;node != nullptr;node = node->getNextNode())
        {
            if((node->getId() != vectorNode->at(0).at(0).getId()) && (node->getId() != vectorNode->at(1).at(0).getId()) && !visitado[node->getIdNode()])
            {       
                //vectorWeight->emplace_back(*node);
                //vectorEdge->emplace_back(*node);
                vectorWeightEdge->emplace_back(*node);
            }
        }

        vector<vector<int>> *listMaiorMenorPeso = new vector<vector<int>>;
        listMaiorMenorPeso->reserve(p);

        for(int i=0;i<p;i++) {
            listMaiorMenorPeso->emplace_back(*criaVetorMaiorMenor());
            listMaiorMenorPeso->at(i).reserve(2);
            listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).begin(),-1);
            listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).end(),1000000);
        }

        /*for(Node *i = this->first_node; i != nullptr; i = i->getNextNode())
        {
            output_file << " Node : " << i->getId() << endl;
            for(Edge *e = i->getFirstEdge(); e != nullptr; e = e->getNextEdge())
            {
                output_file << " ----- Aresta : " << e->getTargetId() << endl;
            }       
        }

        output_file << " --------------------------------- " << endl;
        output_file << endl;*/

         for(int i =0; i < vectorWeightEdge->size(); i++)
        {
            output_file << " Posicao: " << i << " Node : " << vectorWeightEdge->at(i).getId() << endl;     
        }
        output_file << " --------------------------------- " << endl;
        output_file << endl;

        do {
            for(int i=0;i<p;i++) {
                float menorVal;   
                int contPosicao = 0;
                float gap = 0;
                float maiorValor = vectorNode->at(i).at(0).getWeight();
                float menorValor = vectorNode->at(i).at(0).getWeight();
                //getMaiorMenorVal(&maiorValor, &menorValor, vectorNode->at(i), i, p);
                for(int j=0;j<vectorNode->at(i).size();j++) {
                    if(maiorValor < vectorNode->at(i).at(j).getWeight()) {
                        maiorValor = vectorNode->at(i).at(j).getWeight();
                    } else if(menorValor > vectorNode->at(i).at(j).getWeight()) {
                        menorValor = vectorNode->at(i).at(j).getWeight();
                    }
                } // possivelmente isso vai sair daqui
                //listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).begin(), maiorValor);
                //listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).end(), menorValor);
                listMaiorMenorPeso->at(i).at(0) = maiorValor;
                listMaiorMenorPeso->at(i).at(1) = menorValor;
                
                gap = maiorValor - menorValor;
                //listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).begin(), 18);
                //listMaiorMenorPeso->at(i).at(1) = 17;
                for(int j=0;j<vectorWeightEdge->size();j++) {
                    
                    float gapNode;
                    float gapFinal;
                    if(vectorWeightEdge->at(j).getWeight() > maiorValor) {
                        gapNode = vectorWeightEdge->at(j).getWeight() - menorValor;
                    } else if(vectorWeightEdge->at(j).getWeight() < menorValor) {
                        gapNode = maiorValor - vectorWeightEdge->at(j).getWeight();
                    } else {
                        gapNode = 0;
                    }
                    //gapNode = gapNode - gap;
                    if(gapNode < 0) {
                        gapNode *= -1;
                    }
                    gapFinal = gapNode / vectorWeightEdge->at(j).getTotal_Edge();
                    listRank->at(i).emplace_back(gapFinal);
                    if(j == 0) {
                        menorVal = gapFinal;
                    } else {
                        if(menorVal > gapFinal) {
                            menorVal = gapFinal;
                            contPosicao = j; // armazena a posição com menor gap;
                        }
                    }
                    //sort(listRank->at(i).begin(), listRank->at(i).end(), greater<float>());
                }

                if((vectorWeightEdge->size() > 0) && !visitado[vectorWeightEdge->at(contPosicao).getIdNode()])
                {
                    vectorWeightEdge->at(contPosicao).setCor(i);
                    getNode(vectorWeightEdge->at(contPosicao).getId())->setCor(i);
                    visitado[vectorWeightEdge->at(contPosicao).getIdNode()] = true;
                
                    if(vectorWeightEdge->at(contPosicao).getWeight() > listMaiorMenorPeso->at(i).at(0)) {
                        listMaiorMenorPeso->at(i).at(0) = vectorWeightEdge->at(contPosicao).getWeight();
                    } else if(vectorWeightEdge->at(contPosicao).getWeight() < listMaiorMenorPeso->at(i).at(1)) {
                        listMaiorMenorPeso->at(i).at(1) = vectorWeightEdge->at(contPosicao).getWeight();
                    }

                    vectorNode->at(i).emplace_back(vectorWeightEdge->at(contPosicao));  
                    for(Edge *edge = vectorWeightEdge->at(contPosicao).getFirstEdge();edge != nullptr;edge = edge->getNextEdge()) {
                        if((getNode(edge->getTargetId())->getInDegree() == 1) && !visitado[getNode(edge->getTargetId())->getIdNode()] ) {                      
                            getNode(edge->getTargetId())->setCor(i);
                            visitado[getNode(edge->getTargetId())->getIdNode()] = true;
                            for(int i=0;i<vectorWeightEdge->size();i++) {
                                if(vectorWeightEdge->at(i).getId() == getNode(edge->getTargetId())->getId()) {
                                    vectorWeightEdge->erase(vectorWeightEdge->begin() + i);
                                }
                            }

                            if(getNode(edge->getTargetId())->getWeight() > listMaiorMenorPeso->at(i).at(0)) {
                                listMaiorMenorPeso->at(i).at(0) = getNode(edge->getTargetId())->getWeight();
                            } else if(getNode(edge->getTargetId())->getWeight() < listMaiorMenorPeso->at(i).at(1)) {
                                listMaiorMenorPeso->at(i).at(1) = getNode(edge->getTargetId())->getWeight();
                            }

                            vectorNode->at(i).emplace_back(*getNode(edge->getTargetId()));
                        }
                    }
                    int posicaoNode = getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getIdNode();
                    if((vectorWeightEdge->at(contPosicao).getInDegree() == 1) && !visitado[getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getIdNode()]) {
                        getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->setCor(i);
                        vectorNode->at(i).emplace_back(*getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId()));

                        if(getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getWeight() > listMaiorMenorPeso->at(i).at(0)) {
                            listMaiorMenorPeso->at(i).at(0) = getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getWeight();
                        } else if(getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getWeight() < listMaiorMenorPeso->at(i).at(1)) {
                            listMaiorMenorPeso->at(i).at(1) = getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getWeight();
                        }

                        for(int vecCont = 0; vecCont < vectorWeightEdge->size(); vecCont++)
                        {
                            if(vectorWeightEdge->at(vecCont).getId() == vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())
                            {
                                vectorWeightEdge->erase(vectorWeightEdge->begin() + vecCont);
                            }
                        }
                        if(contPosicao != 0 )
                        {
                            contPosicao--;
                        }
                        
                        visitado[getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getIdNode()] = true;
                    }
                    //vector<Node>::iterator id;
                    vector<Node>::iterator n;
                    
                    n = vectorWeightEdge->begin(); 

                    advance(n, contPosicao);

                    output_file << " Posicao : " << contPosicao << endl;
                    output_file << " Elemento que esta sendo apagado : " << vectorWeightEdge->at(contPosicao).getId() << endl;
                    vectorWeightEdge->erase(n);
                    //vectorWeightEdge->resize(vectorWeightEdge->size()-1);

                    cout << " PASSOU " << endl;
                    listRank->at(i).clear();
                    listRank->reserve(listRank->capacity()-1);
                
                }
            }
        } while(!vectorWeightEdge->empty());

        cout << " SAIU DO WHILE " << endl;

       /*
        cout << " SAIU DO WHILE " << endl;
    
        delete vectorWeightEdge;

        for(int i =0;i<p;i++)
        {
            int contadoraSubCluster = 0;
            bool *verificados = new bool[this->order];
            int contClusterAux = 1;
            for(int j =0;j<this->order;j++) {
                verificados[j] = false;
            }
            vector<vector<int>> *maiorMenorValSubCluster = new vector<vector<int>>;
            maiorMenorValSubCluster->reserve(this->order);
            vector<vector<Node>> *vetorClusterNodes = new vector<vector<Node>>();
            vetorClusterNodes->reserve(this->order);
            for(int j=0;j<this->order;j++) {
                //vetorClusterNodes->push_back(*criaVector());
                vetorClusterNodes->emplace_back(*criaVector());
                vetorClusterNodes->at(j).reserve(vectorNode->at(i).size());
                maiorMenorValSubCluster->emplace_back(*criaVetorMaiorMenor());
                maiorMenorValSubCluster->at(j).reserve(2);
                //maiorMenorValSubCluster->at(j).insert(maiorMenorValSubCluster->at(j).begin(), -1);
                //maiorMenorValSubCluster->at(j).insert(maiorMenorValSubCluster->at(j).end(), -1);
            }
            vetorClusterNodes->at(0).insert(vetorClusterNodes->at(0).begin(),vectorNode->at(i).at(0));      
            vectorNode->at(i).erase(vectorNode->at(i).begin());

            int contSameCluster;
            int contSubCluster = 1;
   
            //while(!vectorNode->at(i).empty()) {
            for(int k=0;k<contClusterAux;k++) 
            {
                contadoraSubCluster++;
                contSameCluster = 0;
                maiorMenorValSubCluster->at(k).front() = vetorClusterNodes->at(k).at(0).getWeight();
                maiorMenorValSubCluster->at(k).back() = vetorClusterNodes->at(k).at(0).getWeight();
                //maiorMenorValSubCluster->at(k).at(0) = vetorClusterNodes->at(k).at(0).getWeight();
                //maiorMenorValSubCluster->at(k).at(1) = vetorClusterNodes->at(k).at(0).getWeight();
                int maior = vetorClusterNodes->at(k).at(0).getWeight();
                int menor = vetorClusterNodes->at(k).at(0).getWeight();       
                for(int j=0;j<vetorClusterNodes->at(k).size();j++) 
                {
                    Node *node = &vetorClusterNodes->at(k).at(j);
                    verificados[node->getIdNode()] = true;                     
                    int tam = node->getTotal_Edge();
                    int *vizinhos = new int[tam];
                    int contAuxVizinhos = 0;
                    bool inseriu = false;
                    for(Edge *edge = node->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) {
                        if((getNode(edge->getTargetId())->getCor() == node->getCor()) && !verificados[edge->getTargetIdNode()]) {                         
                            vetorClusterNodes->at(k).emplace_back(*getNode(edge->getTargetId()));
                            vizinhos[contAuxVizinhos] = edge->getTargetId();
                            contAuxVizinhos++;
                            inseriu = true;
                        }
                    }
                    //vizinhos[contAuxVizinhos] = node->getId();
                    //contAuxVizinhos++;

                    //talvez volte com isso;
                    for(int l=0;l<contAuxVizinhos;l++) {
                        for(int aux=0;aux<vectorNode->at(i).size();aux++) {
                            if(vectorNode->at(i).at(aux).getId() == vizinhos[l]) {                          
                                vectorNode->at(i).erase(vectorNode->at(i).begin() + aux);
                            }
                        }
                    }
                    //talvez volte com isso 

                    if((inseriu == false) && !vectorNode->at(i).empty()) {
                        if(vetorClusterNodes->at(k).size()-(j+1) <= 0) {
                            vetorClusterNodes->at(k+1).emplace_back(*vectorNode->at(i).begin());
                            contClusterAux++;
                            vectorNode->at(i).erase(vectorNode->at(i).begin());
                        }
                    }
                    */

                    /* if(vetorClusterNodes->at(k).at(j).getWeight() > listMaiorMenorPeso->at(k).front()) {
                        maiorMenorValSubCluster->at(i).front() = vetorClusterNodes->at(k).at(j).getWeight();
                    } else if(vetorClusterNodes->at(k).at(j).getWeight() < listMaiorMenorPeso->at(k).back()) {
                        maiorMenorValSubCluster->at(i).back() = vetorClusterNodes->at(k).at(j).getWeight();
                    }
                    */
     /*                   if(inseriu)
                        {
                            if(maior < vetorClusterNodes->at(k).at(j+1).getWeight())
                            {
                                maior = vetorClusterNodes->at(k).at(j+1).getWeight();
                            }
                            else if( menor > vetorClusterNodes->at(k).at(j+1).getWeight())
                            {
                                menor = vetorClusterNodes->at(k).at(j+1).getWeight();
                            }
                        }

                }
                    
                //maiorMenorValSubCluster->at(k).front() = maior;
                //maiorMenorValSubCluster->at(k).back() = menor;
                maiorMenorValSubCluster->at(k).insert(maiorMenorValSubCluster->at(k).begin(), maior);
                maiorMenorValSubCluster->at(k).insert(maiorMenorValSubCluster->at(k).end(), menor);
                //maiorMenorValSubCluster->at(k).at(0) = maior;
                //maiorMenorValSubCluster->at(k).at(1) = menor;

                /*if(!vectorNode->at(i).empty()) {
                    contClusterAux++;
                }*/

                //Testando sem isso aq pq estou fazendo contClusterAux++ lá em cima
                /*if(!vectorNode->at(i).empty()) {
                    contClusterAux++;
                }
 
            } */
            //}

      /*    getNode(vetorClusterNodes->at(0).at(0).getId())->setCor(2);

            vetorClusterNodes->resize(contadoraSubCluster);
            //maiorMenorValSubCluster->shrink_to_fit();             

            int maiorSubCluster = vetorClusterNodes->at(0).size(); // pegando o size do primeiro subcluster de cada cor(cada vez que o for com i < p roda)
            //vector<int> *posicoesDosMaiores = new vector<int>;
            for(int e=0;e<vetorClusterNodes->size();e++) {
                if(maiorSubCluster < vetorClusterNodes->at(e).size()) {
                    maiorSubCluster = vetorClusterNodes->at(e).size(); // verificando qual o maior subcluster de cada cor(cada vez que o for com i < p roda)
                }
            }
            bool entrou = false;
            int gapFinalSubCluster = -1;
            int posicaoMaiorSubCluster;
            for(int e=0;e<vetorClusterNodes->size();e++) { // e < tamanho de subclusters existentes
                if(maiorSubCluster == vetorClusterNodes->at(e).size()) { // salvando o gap do maior subcluster(maior no sentido de vértices presentes)
                    if(entrou == false) { // primeira vez a entrar
                        entrou = true;
                        gapFinalSubCluster = maiorMenorValSubCluster->at(e).at(0) - maiorMenorValSubCluster->at(e).at(1);
                        posicaoMaiorSubCluster = e;
                    } else { // buscando o maior subcluster com o menor gap
                        if(gapFinalSubCluster > (maiorMenorValSubCluster->at(e).at(0) - maiorMenorValSubCluster->at(e).at(1))) {
                            gapFinalSubCluster = maiorMenorValSubCluster->at(e).at(0) - maiorMenorValSubCluster->at(e).at(1);
                            posicaoMaiorSubCluster = e;
                        }
                    }
                }
            }
            vector<vector<int>> *arestas = new vector<vector<int>>;
            arestas->reserve(vetorClusterNodes->size()); // no i = 0 tá reservando tamanho 4
            for(int e=0;e<vetorClusterNodes->size();e++) {  // e < 4
                arestas->push_back(*criaVetorMaiorMenor());
                arestas->at(e).reserve(vetorClusterNodes->at(e).size()); // reservando tamanhos 2 1 2 1
            }
            vector<bool> *corNode = new vector<bool>;
            corNode->reserve(p); // reservando as cores conforme o número de clusters solicitados
            corNode->insert(corNode->begin(), true);
            for(int n = 0;n<p;n++) { 
                //corNode->at(n) = true; // marcando todas as cores como true, posivel de visitar
                corNode->insert(corNode->begin() + n, true);
            }
            corNode->at(i) = false; // marcando a cor atual como false, pois não queremos ligar um subcluster em outro de mesma cor dele
            vector<vector<int>> *coresPossiveis = new vector<vector<int>>;
            //vector<Node> *vectorWeightEdge = new vector<Node>();
            coresPossiveis->reserve(this->order);
            for(int x=0;x<contSubCluster;x++) {
                coresPossiveis->push_back(*criaVetorMaiorMenor());
                coresPossiveis->at(x).reserve(2); // 2 posições, 1° com a cor e a 2° com o gap
                coresPossiveis->at(x).insert(coresPossiveis->at(x).begin(), 1000000);
                coresPossiveis->at(x).insert(coresPossiveis->at(x).end(), x);
            }
            //vector<vector<int>> *valores = new vector<vector<int>>;
            vector<vector<int>> *menorOuMaior = new vector<vector<int>>;
            menorOuMaior->reserve(this->order);
            for(int e=0;e<this->order;e++) {
                menorOuMaior->push_back(*criaVetorMaiorMenor());
                menorOuMaior->at(e).reserve(2);
            }
            int contEntradas = 0;
            //int menorOuMaior = -1;
            if(i == 0) {
                for(int e=0;e<vetorClusterNodes->size();e++) { // e < que o total de subclusters no i(cor atual)
                    if(e != posicaoMaiorSubCluster) { // e sendo diferente do subcluster que a gente quer manter(no caso a posição dele no vetorClusterNodes->at(e))
                        for(int z = 0;z < vetorClusterNodes->at(e).size();z++) { // z < que a quantidade de nodes presentes em cada subcluster
                            for(Edge *edge = vetorClusterNodes->at(e).at(z).getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) { // verificando as arestas de cada subvertice
                                if(corNode->at(getNode(edge->getTargetId())->getCor())) { // se a cor do node estiver como true, ou seja não foi verificada ainda e nem é a cor do i
                                    corNode->at(getNode(edge->getTargetId())->getCor()) = false; // marca a cor como visitada
                                    if(maiorMenorValSubCluster->at(e).front() > listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).front() && maiorMenorValSubCluster->at(e).back() < listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).back()) {
                                        coresPossiveis->at(z).push_back(maiorMenorValSubCluster->at(e).front() - maiorMenorValSubCluster->at(e).back()); // salvando o gap pra cor atual
                                        coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor()); // salvando a cor do vizinho
                                        menorOuMaior->at(contEntradas).front() = z;
                                        menorOuMaior->at(contEntradas).back() = 0;
                                        //menorOuMaior->insert(menorOuMaior->begin() + z, 0);
                                        contEntradas++;
                                    } else if(maiorMenorValSubCluster->at(e).front() > listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).at(0)) {
                                        coresPossiveis->at(z).push_back(maiorMenorValSubCluster->at(e).front() - listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).front()); // salvando o gap pra cor atual
                                        coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor());  // salvando a cor do vizinho
                                        menorOuMaior->at(contEntradas).front() = z;
                                        menorOuMaior->at(contEntradas).back() = 1;
                                        //menorOuMaior->at(z) = 1;
                                        //menorOuMaior->insert(menorOuMaior->begin() + z, 1);
                                        contEntradas++;
                                    } else if(maiorMenorValSubCluster->at(e).back() < listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).back()) {
                                        coresPossiveis->at(z).push_back(listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster->at(e).back()); // salvando o gap pra cor atual
                                        coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor()); // salvando a cor do vizinho
                                        menorOuMaior->at(contEntradas).front() = z;
                                        menorOuMaior->at(contEntradas).back() = 2;
                                        //menorOuMaior->at(z) =  2;
                                        //menorOuMaior->insert(menorOuMaior->begin() + z, 2);
                                        contEntradas++;
                                    } else {
                                        //coresPossiveis->at(z).push_back(0); // nesse caso o maior e menor valor do subcluster não impactam no gap do cluster
                                        //coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor()); // salvando a cor do node
                                        //coresPossiveis->at(z).insert(coresPossiveis->at(z).begin(), 0);
                                        //coresPossiveis->at(z).insert(coresPossiveis->at(z).end(), getNode(edge->getTargetId())->getCor());
                                        // coresPossiveis->at(z).front() = getNode(edge->getTargetId())->getCor();
                                        //coresPossiveis->at(z).back() = 0;
                                        coresPossiveis->at(z).front() = 0;
                                        coresPossiveis->at(z).back() = getNode(edge->getTargetId())->getCor();
                                        menorOuMaior->at(contEntradas).front() = z;
                                        menorOuMaior->at(contEntradas).back() = -1;
                                        //menorOuMaior->at(z) = -1;
                                        //menorOuMaior->insert(menorOuMaior->begin() + z, -1);
                                        contEntradas++;
                                    }
                                }
                            }    
                        }
                        //int menor = coresPossiveis->at(z).front();
                        int menor = coresPossiveis->at(0).front(); // pegando o primeiro gap
                        int corSelecionado = coresPossiveis->at(0).back(); // pegando a primeira cor selecionada
                        int contPosicaoSubCluster = 0;
                        //for(int z = 0;z < coresPossiveis->size();z++) { // z < que o número de vértices presentes em cada subcluster
                        //for(int z=0;z<vetorClusterNodes->at(e).size();z++) { // esse aqui tava usando por ultimo
                        for(int z=0;z<contEntradas;z++) {
                        //if(menor > coresPossiveis->at(z).size()) { //
                            if(menor > coresPossiveis->at(z).front()) {// pegando o menor valor de gap presente
                                menor = coresPossiveis->at(z).front(); // salvando esse valor de gap como o menor
                                corSelecionado = coresPossiveis->at(z).back(); // salvando a cor desse gap
                                contPosicaoSubCluster = z; // salvando essa posição escolhida com o menor gap

                            }
                        }
                        for(int z=0;z<vetorClusterNodes->at(e).size();z++) {
                            getNode(vetorClusterNodes->at(e).at(z).getId())->setCor(corSelecionado);
                            if(corSelecionado > i) {
                                //vectorNode->at(e).at(z).setCor(corSelecionado);
                                for(Edge *node = getNode(13)->getFirstEdge();node != nullptr;node = node->getNextEdge()) {
                                }
                                vetorClusterNodes->at(e).at(z).setCor(corSelecionado);
                                vectorNode->at(corSelecionado).emplace_back(*getNode(vetorClusterNodes->at(e).at(z).getId()));
                                verificados[vetorClusterNodes->at(e).at(z).getIdNode()] = false;
                                for(Edge *node = getNode(13)->getFirstEdge();node != nullptr;node = node->getNextEdge()) {
                                }
                            }
                        }

                        //vetorClusterNodes->at(e).clear();
                        for(int z=0;z<coresPossiveis->size();z++) {
                            coresPossiveis->at(z).clear();
                        }

                        //if(coresPossiveis->at(contPosicaoSubCluster).back() != 0) {
                        int contSelecionado = 0;
                        for(int z = 0;z<contEntradas;z++) {
                            if(menorOuMaior->at(z).front() == contPosicaoSubCluster) {
                                contSelecionado = z;
                            } 
                        } 
                        if(menorOuMaior->at(contSelecionado).back() == 0) {
                            //listMaiorMenorPeso->at(corSelecionado) = menor;//coresPossiveis->at(contPosicaoSubCluster);
                            //listMaiorMenorPeso->insert(listMaiorMenorPeso->begin() + corSelecionado, { menor });
                            listMaiorMenorPeso->at(corSelecionado).at(0) = maiorMenorValSubCluster->at(e).at(0);
                            listMaiorMenorPeso->at(corSelecionado).at(1) = maiorMenorValSubCluster->at(e).at(1);
                        } else if(menorOuMaior->at(contSelecionado).back() == 1) {
                            listMaiorMenorPeso->at(corSelecionado).at(0) = maiorMenorValSubCluster->at(e).at(0);
                        } else if(menorOuMaior->at(contSelecionado).back() == 2) {
                            listMaiorMenorPeso->at(corSelecionado).at(1) = maiorMenorValSubCluster->at(e).at(1);
                        }
                        for(int z=0;z<contEntradas;z++) {
                            menorOuMaior->at(z).clear();
                        }
                        contEntradas = 0;
                    }
                }
            }

            /*
                //int menor = coresPossiveis->at(z).front();
                int menor = coresPossiveis->at(0).front(); // pegando o primeiro gap
                int corSelecionado = coresPossiveis->at(0).back(); // pegando a primeira cor selecionada
                for(int z = 0;z < coresPossiveis->size();z++) { // z < que o número de vértices presentes em cada subcluster
                    //if(menor > coresPossiveis->at(z).size()) { // 
                    if(menor > coresPossiveis->at(z).front()) {// pegando o menor valor de gap presente
                        menor = coresPossiveis->at(z).front(); // salvando esse valor de gap como o menor
                        corSelecionado = coresPossiveis->at(z).back(); // salvando a cor desse gap
                    }
                }

                for(int z=0;z<vetorClusterNodes->at(e).size()) {
                    getNode(vetorClusterNodes->at(e).at(z).getId())->setCor() = corSelecionado;
                }
            */

     /*         if( i == (p-1))
            {
                for(int g=0; g < vetorClusterNodes->size(); g++ )
                {
                    delete &vetorClusterNodes->at(g);
                }

                for(int s =0; s < maiorMenorValSubCluster->size(); s++)
                {
                    delete &maiorMenorValSubCluster->at(s);
                }

                delete maiorMenorValSubCluster;
                delete vetorClusterNodes;
                delete verificados;
                //delete menorOuMaior;
            }
        }

        for(int h =0; h < listRank->size();h++)
        {
            delete &listRank->at(h);
        } 

        delete listMaiorMenorPeso;
        delete vectorNode; 
        delete visitado;
        delete vectorWeightEdge;
        delete listRank;
       
      */
    } 
    else {
        output_file << "O grafo não tem peso nas arestas" << endl;
    }  
    
}

/*
vector<float> Graph::geraRank(vector<vector<Node>> vectorCluster, int idCluster, vector<int> nodeWeight, vector<int> nodeGrau )
{   
    
    int vet[2];
    int resultado;
    vector<float> rank;
    int maior=0;
    int menor=0;

    vector<Node> = vectorCluster[idCluster];


    
    for(int i=0; i<vectorCluster[idCluster].size(); i++)
    {
        vectorCluster[idCluster][i];
        if(vectorCluster[idCluster][i] >= maior)
        {
            maior = vectorCluster[idCluster][i]->getId();
        }

        if(vectorCluster[idCluster][i] <= menor)
        {
            menor = vectorCluster[idCluster][i];
        }
    }

    // vet[0] =  ;
    // vet[1] =  ;


        for(int i=0;i<this->order;i++)
        {
            
            if(nodeWeight[i] >= vet[1]  )
            {
               resultado = (nodeWeight[i] - vet[0])/nodeGrau[i]; 
            }
            else if( nodeWeight[i] <= vet[0])
            {
                resultado = (nodeWeight[i] - vet[1])/nodeGrau[i];
            }
            else if( nodeWeight[i] >= vet[0] && nodeWeight[i] <= vet[1])
            {
                resultado = 0;
            }
            
            if(resultado < 0)
            {
                resultado = resultado * -1;
            }
            
             rank[i] = resultado;

        }

    return rank;

}

*/


vector<Node> *Graph::criaVector() {
    //vector<Node> vector;
    vector<Node> *vectorNode = new vector<Node>();
    //vectorNode->reserve(this->order);
    return vectorNode;
}

vector<float> *Graph::criaVetorRank(int p) {
    vector<float> *vectorNode = new vector<float>;
    //vectorNode->reserve(this->order-p);
    return vectorNode;
}

vector<int> *Graph::criaVetorMaiorMenor() {
    vector<int> *vectorNode = new vector<int>;
    return vectorNode;
}

/*void Graph::getMaiorMenorVal(float *maiorValor, float *menorValor, vector<Node> *vectorNode, int i, int p) {
    for(int j=0;j<vectorNode->at(p).size();j++) {
        if(*maiorValor < vectorNode->at(i).at(j).getWeight()) {
            *maiorValor = vectorNode->at(i).at(j).getWeight();
        } else if(*menorValor > vectorNode->at(i).at(j).getWeight()) {
            *menorValor = vectorNode->at(i).at(j).getWeight();
        }
    }
}*/


/// SEGUNDA ETAPA DO TRAB /////////////////////////////////




/// FIM DA SEGUNDA ETAPA DO TRAB //////////////////////////
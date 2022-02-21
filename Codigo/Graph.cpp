#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <fstream>
/*#include <stack>
#include <queue>
#include <list>
#include <vector>*/
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <iomanip>
#include <iterator>
#include <algorithm> /// fun��o find
#include <ctime>
//#include <Windows.h>
#include <chrono>
#include <thread>
using namespace std;
//#define INFINITO 1000000000;
#include <limits.h>
constexpr int FLOAT_MIN = 0;
constexpr int FLOAT_MAX = 1;
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
void Graph::deleteGraph()
{
    Node *aux;
    Node *node = this->first_node;
    while(node->getNextNode() != nullptr)
    {
        aux = node->getNextNode();
        delete node;        
    }
}

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

void Graph::Guloso(ofstream &output_file, int p, int numIter)
{
    auto start = std::chrono::high_resolution_clock::now();
    int valRep = numIter;
    int *vetIter = new int[valRep];
    int menorGap = 0;
    for(int e=0;e<valRep;e++) {

        bool *visitado = new bool[this->order];  // vetor para verificar os vértices já utilizados

        for(int i=0;i<this->order;i++)
        {
            visitado[i] = false; // marcando todos nodes como não visitados
        }

        if(this->weighted_node) // só pode grafo com node com peso
        {

            vector<vector<Node*>> vectorNode; // vetor de vetores de node
            vectorNode.reserve(p); // reservando tamanho para o vector = número de clusters passado
            for(int i=0;i<p;i++) {
                vector<Node*> vectorNodes;
                vectorNode.push_back(vectorNodes); // criando os vetores de node;
                vectorNode.at(i).reserve(this->order); 
            }
            

            //unsigned seed = time(0);
            //srand(seed);
            //float semente = rand();
            //output_file << "semente: " << rand() <<endl;
            //Sleep(1000);
            //srand( (unsigned)time(NULL) );
            //srand(time(0)); // semente aleatoria

            for(int i=0;i<p;i++) // montando os cluster iniciais  
            {
                //output_file << "i: " << i << endl;
                Node *nodeAux; 
                bool vizinho = false;
                do {
                    int x = 1 + (rand() % this->order-1); // escolhendo número aleatorio
                    nodeAux = this->getNodeId(x); // pegando node referente a esse número
                    vizinho = false;
                    if(i > 0) // não deixar nodes vizinhos juntos
                    {
                        if(!visitado[nodeAux->getIdNode()]) // caso o node escolhido ainda não tenha sido visitado
                        {
                            for(Edge *edgeAux = nodeAux->getFirstEdge(); edgeAux!=nullptr ; edgeAux = edgeAux->getNextEdge()) // percorrendo a aresta para ver se os vertices escolhidos até então não são vizinhos
                            {
                                for(int g=0;g<i;g++)
                                {
                                    for(int f=0;f<vectorNode.at(g).size();f++)
                                    {
                                        if(edgeAux->getTargetId() == vectorNode.at(g).at(f)->getId())
                                        {
                                            vizinho = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                } while(visitado[nodeAux->getIdNode()] || vizinho); // se o node já tiver sido colocado ele troca
                visitado[nodeAux->getIdNode()] = true; // marcando como visitado

                for(Edge *edgeAux = nodeAux->getFirstEdge(); edgeAux != nullptr; edgeAux = edgeAux->getNextEdge())
                {
                    if(getNode(edgeAux->getTargetId())->getInDegree() == 1) // verificando se a aresta ao nó escolhido só tem o nó escolhido como vizinho
                    {
                        vectorNode.at(i).emplace_back(getNode(edgeAux->getTargetId())); // Coloca o vizinho de grau 1 na lista
                        visitado[edgeAux->getTargetIdNode()] = true;  // Coloca o node vizinho como já utilizado
                        visitado[nodeAux->getIdNode()] = true; // coloca o node escolhido como já utilizado

                    }    

                }

                if(nodeAux->getInDegree() == 1) { // se o nó escolhido tem grau de entrada 1 já pega o vizinho dele junto
                    vectorNode.at(i).emplace_back(nodeAux);  // caso o node só tenha uma aresta a gente vai inserir o único vizinho direto na lista que o vizinho tá
                    vectorNode.at(i).emplace_back(getNode(nodeAux->getFirstEdge()->getTargetId())); // único vizinho direto já pode pegar direto no getFirstEdge()
                    visitado[nodeAux->getIdNode()] = true;  // Coloca o node como já utilizado
                    visitado[nodeAux->getFirstEdge()->getTargetIdNode()] = true; // coloca o vizinho do node como já utilizado

                } else {
                    vectorNode.at(i).emplace_back(nodeAux); // inserindo esse node na lista da posição i do vector
                    visitado[nodeAux->getIdNode()] = true;   // Coloca o vértice como já utilizado

                }
            }

            for(int q=0;q<vectorNode.size();q++) { 
                for(int l=0;l<vectorNode.at(q).size();l++) {
                    vectorNode.at(q).at(l)->setCor(q);
                    visitado[vectorNode.at(q).at(l)->getIdNode()] = true; // marcando como visitado os nodes inseridos aleatoriamente no começo

                }
            }

            vector<Node*> vectorWeightEdge; // nodes que ainda não foram inseridos em nenhum cluster
            vectorWeightEdge.reserve(this->order-p);
            vector<vector<float>> listRank; // vector de ranqueamento dos nodes
            int contadora = 0;
            listRank.reserve(p); // reservando espaço para o total de clusters nesse vector 
            for(int i=0;i<p;i++) {
                //vector<float> *rank = new vector<float>;
                vector<float> rank;
                listRank.push_back(rank);
                listRank.at(i).reserve(this->order-p);
            }

            //Adicionando os vertices que não foram sorteados para a vectorWeightEdge 
            for(Node *node = this->first_node;node != nullptr;node = node->getNextNode())
            {
                if(!visitado[node->getIdNode()])
                {       
                    vectorWeightEdge.emplace_back(node);
                }
            }

            vector<vector<int>> listMaiorMenorPeso; // lista com maior e menor peso de cada cluster
            listMaiorMenorPeso.reserve(p);

            for(int i=0;i<p;i++) {
                //vector<int> *rank = new vector<int>;
                vector<int> rank;
                listMaiorMenorPeso.emplace_back(rank);
                listMaiorMenorPeso.at(i).reserve(2); // para cada cluster existem 2 posicoes, uma com o maior e outra com o menor peso de cada cluster
                listMaiorMenorPeso.at(i).insert(listMaiorMenorPeso.at(i).begin(),-1);
                listMaiorMenorPeso.at(i).insert(listMaiorMenorPeso.at(i).end(),1000000);
            }
            do { // fazendo enquanto existirem vertices para ser alocado em qualquer cluster
                for(int i=0;i<p;i++) { // fazendo isso para todos os p(cluster)
                    contadora = 0;
                    int *idNodesAux = new int[vectorWeightEdge.size()];  
                    float *idRazao = new float[vectorWeightEdge.size()];
                    float menorVal; 
                    int contPosicao = 0;
                    float maiorValor = vectorNode.at(i).at(0)->getWeight();
                    float menorValor = vectorNode.at(i).at(0)->getWeight();
                    for(int j=0;j<vectorNode.at(i).size();j++) { // salvando o maior e menor valor de cada cluster até o momento
                        if(maiorValor < vectorNode.at(i).at(j)->getWeight()) { 
                            maiorValor = vectorNode.at(i).at(j)->getWeight();
                        } else if(menorValor > vectorNode.at(i).at(j)->getWeight()) {
                            menorValor = vectorNode.at(i).at(j)->getWeight();
                        }
                    }

                    listMaiorMenorPeso.at(i).at(0) = maiorValor; // passando esses valores para a lista com maior e menor peso de cada cluster
                    listMaiorMenorPeso.at(i).at(1) = menorValor; //
                    
                    for(int j=0;j<vectorWeightEdge.size();j++) {
                        
                        float gapNode;
                        float gapFinal;

                        if(vectorWeightEdge.at(j)->getWeight() > maiorValor) { // verificando para cada node não selecionado ainda a diferença de peso entre eles e o maior,menor peso presente em cada cluster
                            gapNode = vectorWeightEdge.at(j)->getWeight() - menorValor; 
                        } else if(vectorWeightEdge.at(j)->getWeight() < menorValor) {
                            gapNode = maiorValor - vectorWeightEdge.at(j)->getWeight();
                        } else {
                            gapNode = 0;
                        }

                        if(gapNode < 0) {
                            gapNode *= -1;
                        }
                        gapFinal = gapNode / vectorWeightEdge.at(j)->getTotal_Edge(); // seleção final busca selecionar o menor gapFinal que é o gapNode/ número de arestas de cada node

                        listRank.at(i).emplace_back(gapFinal); // lista de ranqueamento de vértices
                        idNodesAux[contadora] = vectorWeightEdge.at(j)->getId();
                        idRazao[contadora] = gapFinal;
                        contadora++;

                    }
                    int indiceR = 0;
                    if(listRank.at(i).size() > 1) {
                        listRank.shrink_to_fit();
                        sort(listRank.at(i).begin(), listRank.at(i).end());
                        selection_sort(idRazao,idNodesAux,listRank.at(i).size());
                        
                        contPosicao = idNodesAux[0];
                    }
                    for(int q=0;q<vectorWeightEdge.size();q++)
                    {
                        if(contPosicao == vectorWeightEdge.at(q)->getId())
                        {
                            contPosicao = q;
                        }
                    }
                    if((vectorWeightEdge.size() > 0) && !visitado[vectorWeightEdge.at(contPosicao)->getIdNode()]) // caso a lista não esteja vazia
                    {
                        vectorWeightEdge.at(contPosicao)->setCor(i); // setando a cor para a cor atual do cluster

                        visitado[vectorWeightEdge.at(contPosicao)->getIdNode()] = true; // marcando o node escolhido como true

                        if(vectorWeightEdge.at(contPosicao)->getWeight() > listMaiorMenorPeso.at(i).at(0)) { // atualizando os valores de maior e menor de cada cluster

                            listMaiorMenorPeso.at(i).at(0) = vectorWeightEdge.at(contPosicao)->getWeight();
                        } else if(vectorWeightEdge.at(contPosicao)->getWeight() < listMaiorMenorPeso.at(i).at(1)) {

                            listMaiorMenorPeso.at(i).at(1) = vectorWeightEdge.at(contPosicao)->getWeight();
                        }

                        vectorNode.at(i).emplace_back(vectorWeightEdge.at(contPosicao));  // adicionando o node esclhido ao cluster

                        for(Edge *edge = vectorWeightEdge.at(contPosicao)->getFirstEdge();edge != nullptr;edge = edge->getNextEdge()) { // verificando se existe vizinho com in degree == 1
                            //verificando se algum vizinho do node escolhido tem grau de entrada 1 e nao foi visitado ainda
                            if((getNode(edge->getTargetId())->getInDegree() == 1) && !visitado[getNode(edge->getTargetId())->getIdNode()] ) {

                                visitado[getNode(edge->getTargetId())->getIdNode()] = true;
                                for(int i=0;i<vectorWeightEdge.size();i++) {
                                    if(vectorWeightEdge.at(i)->getId() == getNode(edge->getTargetId())->getId()) {
                                        vectorWeightEdge.erase(vectorWeightEdge.begin() + i);
                                    }
                                }

                                //Atualizando o maior e o menor peso de cada cluster
                                if(getNode(edge->getTargetId())->getWeight() > listMaiorMenorPeso.at(i).at(0)) {
                                    listMaiorMenorPeso.at(i).at(0) = getNode(edge->getTargetId())->getWeight();
                                } else if(getNode(edge->getTargetId())->getWeight() < listMaiorMenorPeso.at(i).at(1)) {
                                    listMaiorMenorPeso.at(i).at(1) = getNode(edge->getTargetId())->getWeight();
                                }
                                // adicionando o node ao cluster
                                vectorNode.at(i).emplace_back(getNode(edge->getTargetId()));
                            }
                        }
                        int posicaoNode = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode();
                        // Verificando se o node atual tem in degree == 1
                        if((vectorWeightEdge.at(contPosicao)->getInDegree() == 1) && !visitado[getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode()]) {
                            
                            vectorNode.at(i).emplace_back(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId()));
                            // atualizando maior e menor peso de cada cluster
                            if(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight() > listMaiorMenorPeso.at(i).at(0)) {
                                listMaiorMenorPeso.at(i).at(0) = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight();
                            } else if(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight() < listMaiorMenorPeso.at(i).at(1)) {
                                listMaiorMenorPeso.at(i).at(1) = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight();
                            }
                            //retirando o vizinho do node com in degree == 1 da lista de nodes que ainda não entraram
                            for(int vecCont = 0; vecCont < vectorWeightEdge.size(); vecCont++)
                            {
                                if(vectorWeightEdge.at(vecCont)->getId() == vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())
                                {
                                    //retirando o node selecionado da lista de nodes ainda nao selecionados
                                    vectorWeightEdge.erase(vectorWeightEdge.begin() + vecCont);
                                }

                            }
                            if(contPosicao != 0 )
                            {
                                contPosicao--;
                            }
                            // atualizando a lista de visitado
                            visitado[getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode()] = true;
                        }

                        vector<Node*>::iterator n;
                        
                        n = vectorWeightEdge.begin(); 

                        advance(n, contPosicao);
                        // retirando o node escolhido no ranqueamento da vectorWeightEdge
                        vectorWeightEdge.erase(n);
                        vectorWeightEdge.shrink_to_fit();
                        //limpando a lista para fazer novamente
                        listRank.at(i).clear();
                        listRank.reserve(listRank.capacity()-1);
                    }
                }
            // enquanto ainda existirem nodes no vector 
            } while(!vectorWeightEdge.empty());
            
            listRank.clear(); // não serão mais utilizadas
            vectorWeightEdge.clear(); // não serão mais utilizadas

            // A partir daqui estou montando os subcluster(partições individuais de cada conjunto de nodes dentro de um mesmo cluster)
            for(int i =0;i<p;i++)
            { 
                int contadoraSubCluster = 0;
                bool *verificados = new bool[this->order]; // vetor de verificados
                int contClusterAux = 1; // numero de subCluster
                for(int j =0;j<this->order;j++) {
                    verificados[j] = false;
                }
                vector<vector<int>> maiorMenorValSubCluster; // maior e menor val de cada subcluster serao salvos aqui
                maiorMenorValSubCluster.reserve(this->order);
                vector<vector<Node*>> vetorClusterNodes; // cada subcluster sera armazenado aqui
                vetorClusterNodes.reserve(this->order);
                for(int j=0;j<this->order;j++) {
                    //vector<int> *rank = new vector<int>;
                    vector<int> rank;
                    vector<Node*> vectorNodes;
                    vetorClusterNodes.emplace_back(vectorNodes);
                    maiorMenorValSubCluster.emplace_back(rank);
                    maiorMenorValSubCluster.at(j).reserve(2);

                }
                vetorClusterNodes.at(0).insert(vetorClusterNodes.at(0).begin(), vectorNode.at(i).at(0)); // inserindo o primeiro node no cluster atual     
                vectorNode.at(i).erase(vectorNode.at(i).begin()); // excluindo o primeiro node da lista com os cluster iniciais

                int contSameCluster; 
                int contSubCluster = 1;

                for(int k=0;k<contClusterAux;k++) // responsavel por criar um novo subcluster 
                {
                    contadoraSubCluster++;
                    contSameCluster = 0;
                    
                    // bloco responsavel por atualizar os valores de maior e menor de cada subcluster
                    maiorMenorValSubCluster.at(k).front() = vetorClusterNodes.at(k).at(0)->getWeight(); 
                    maiorMenorValSubCluster.at(k).back() = vetorClusterNodes.at(k).at(0)->getWeight();

                    int maior = vetorClusterNodes.at(k).at(0)->getWeight();
                    int menor = vetorClusterNodes.at(k).at(0)->getWeight();

                    for(int j=0;j<vetorClusterNodes.at(k).size();j++) // j começa menor que 1 e vai atualizando o tamanho dentro do for 
                    {
                        Node *node = vetorClusterNodes.at(k).at(j); // passando o node atual
                        verificados[node->getIdNode()] = true; // marcando o node como true
                        int tam = node->getTotal_Edge(); // pegando o total de arestas do node
                        int *vizinhos = new int[tam]; 
                        int contAuxVizinhos = 0;
                        bool inseriu = false;

                        for(Edge *edge = node->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) {
                            // verificando se o node atual tem vizinhos da mesma cor e que não foram verificados
                            if((getNode(edge->getTargetId())->getCor() == node->getCor()) && !verificados[edge->getTargetIdNode()]) {
                                    
                                vetorClusterNodes.at(k).emplace_back(getNode(edge->getTargetId()));
                                vizinhos[contAuxVizinhos] = edge->getTargetId();
                                contAuxVizinhos++;
                                inseriu = true;
                            }
                        }

                        // retirando os vizinhos da vectorNode(vetor de cluster)
                        for(int l=0;l<contAuxVizinhos;l++) {
                            for(int aux=0;aux<vectorNode.at(i).size();aux++) {
                                if(vectorNode.at(i).at(aux)->getId() == vizinhos[l]) {
                                    vectorNode.at(i).erase(vectorNode.at(i).begin() + aux);
                                }
                            }
                        }

                        // atualizando em mais 1 o numero de cluster 
                        if((inseriu == false) && !vectorNode.at(i).empty()) {
                            if(vetorClusterNodes.at(k).size()-(j+1) <= 0) {
                                vetorClusterNodes.at(k+1).emplace_back(vectorNode.at(i).at(0));
                                contClusterAux++;
                                vectorNode.at(i).erase(vectorNode.at(i).begin());
                            }
                        }
                        // atualizando o maior e menor valores
                        if(inseriu)
                        {
                            for(int y = j;y<=vetorClusterNodes.at(k).size()-1;y++) {
                                if(maior < vetorClusterNodes.at(k).at(y)->getWeight())
                                {
                                    maior = vetorClusterNodes.at(k).at(y)->getWeight();
                                } 
                                else if( menor > vetorClusterNodes.at(k).at(y)->getWeight())
                                {
                                    menor = vetorClusterNodes.at(k).at(y)->getWeight();
                                }
                            }
                        }
                        delete vizinhos;   
                    }
                    maiorMenorValSubCluster.at(k).insert(maiorMenorValSubCluster.at(k).begin(), maior);
                    maiorMenorValSubCluster.at(k).insert(maiorMenorValSubCluster.at(k).end(), menor);
                } 

                vetorClusterNodes.resize(contadoraSubCluster);

                int maiorSubCluster = vetorClusterNodes.at(0).size(); // pegando o size do primeiro subcluster de cada cor(cada vez que o for com i < p roda)
                for(int e=0;e<vetorClusterNodes.size();e++) {
                    if(maiorSubCluster < vetorClusterNodes.at(e).size()) {
                        maiorSubCluster = vetorClusterNodes.at(e).size(); // verificando qual o maior subcluster de cada cor(cada vez que o for com i < p roda)
                    }
                }    

                bool entrou = false;
                int gapFinalSubCluster = -1;
                int posicaoMaiorSubCluster;
                // escolhendo o maior subcluster(numero de nodes) e com o menor gap para manter
                for(int e=0;e<vetorClusterNodes.size();e++) { // e < tamanho de subclusters existentes
                    if(maiorSubCluster == vetorClusterNodes.at(e).size()) { // salvando o gap do maior subcluster(maior no sentido de vértices presentes)
                        if(entrou == false) { // primeira vez a entrar
                            entrou = true;
                            gapFinalSubCluster = maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1);
                            posicaoMaiorSubCluster = e;
                        } else { // buscando o maior subcluster com o menor gap
                            if(gapFinalSubCluster > (maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1))) {
                                gapFinalSubCluster = maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1);
                                posicaoMaiorSubCluster = e;
                            }
                        }
                    }
                }

                vector<bool> corNode; //salva todas as cores menos a do cluster atual como true;
                corNode.reserve(p); // reservando as cores conforme o número de clusters solicitados
                corNode.insert(corNode.begin(), true);

                for(int n = 0;n<p;n++) { 
                    // marcando todas as cores como true, posivel de visitar
                    corNode.insert(corNode.begin() + n, true);
                }
                corNode.at(i) = false; // marcando a cor atual como false, pois não queremos ligar um subcluster em outro de mesma cor dele

                vector<vector<int>> coresPossiveis; // 1° com a cor e a 2° com o gap, vai ajudar a reajustar os subcluster 
                coresPossiveis.reserve(1);

                for(int x=0;x<1;x++) {
                    //vector<int> *rank = new vector<int>;
                    vector<int> rank;
                    coresPossiveis.push_back(rank);
                    coresPossiveis.at(x).reserve(2); // 2 posições, 1° com a cor e a 2° com o gap
                    coresPossiveis.at(x).insert(coresPossiveis.at(x).begin(), 1000000);
                    coresPossiveis.at(x).insert(coresPossiveis.at(x).end(), x);
                }

            
                vector<vector<int>> menorOuMaior; // auxilia para mudar o valor final de cada cluster 
                menorOuMaior.reserve(1);
                for(int e=0;e<1;e++) {
                    //vector<int> *rank = new vector<int>;
                    vector<int> rank;
                    menorOuMaior.push_back(rank);
                    menorOuMaior.at(e).reserve(2);
                }

                int contEntradasArestas;
                int contEntradas = 0;
                int gap;
                for(int e=0;e<vetorClusterNodes.size();e++) { // e < que o total de subclusters no i(cor atual)
                    if(e != posicaoMaiorSubCluster) { // e sendo diferente do subcluster que a gente quer manter(no caso a posição dele no vetorClusterNodes->at(e))
                        gap = 10000000;
                        for(int z = 0;z < vetorClusterNodes.at(e).size();z++) { // z < que a quantidade de nodes presentes em cada subcluster

                            contEntradasArestas = 0;
                                
                            for(Edge *edge = vetorClusterNodes.at(e).at(z)->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) { // verificando as arestas de cada subvertice
                                    
                                if(corNode.at(getNode(edge->getTargetId())->getCor())) { // se a cor do node estiver como true, ou seja não foi verificada ainda e nem é a cor do i

                                    corNode.at(getNode(edge->getTargetId())->getCor()) = false; // marca a cor como visitada
                                        
                                    if(maiorMenorValSubCluster.at(e).front() > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() && maiorMenorValSubCluster.at(e).back() < listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back()) {
                                            
                                        if(gap > maiorMenorValSubCluster.at(e).front() - maiorMenorValSubCluster.at(e).back()) {
                                                
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = 0;
                                            gap = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front();
                                            gap += listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back() - maiorMenorValSubCluster.at(e).back(); 
                                                
                                            coresPossiveis.at(0).at(0) = gap;
                                            coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                                
                                            contEntradas++;
                                        }
                                    } else if(maiorMenorValSubCluster.at(e).front() > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(0)) {
                                            
                                        if(gap > maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(0)) {
                                                
                                            coresPossiveis.at(0).at(0) = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front();
                                            coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = 1;

                                            gap = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(1);
                                            contEntradas++;
                                        }
                                    } else if(maiorMenorValSubCluster.at(e).back() < listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back()) {
                                            
                                        if(gap > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster.at(e).back()) {
                                                
                                            coresPossiveis.at(0).at(0) = listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back() - maiorMenorValSubCluster.at(e).back();
                                            coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = 2;

                                            contEntradas++;
                                            gap = listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster.at(e).back();
                                        }
                                    } else {

                                        if(gap > 0) {
                                            coresPossiveis.at(0).front() = 0;
                                            coresPossiveis.at(0).back() = getNode(edge->getTargetId())->getCor();
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = -1;
                                            contEntradas++;
                                            gap = 0;
                                        }
                                    }
                                }
                            }    
                        }

                        int menor = coresPossiveis.at(0).front(); // pegando o primeiro gap
                        int corSelecionado = coresPossiveis.at(0).back(); // pegando a primeira cor selecionada
                        int contPosicaoSubCluster = 0;

                        // atualizando a cor dos nodes para os novos cluster e adicionando os nodes nesses novos clusters
                        for(int z=0;z<vetorClusterNodes.at(e).size();z++) {
                            getNode(vetorClusterNodes.at(e).at(z)->getId())->setCor(corSelecionado);
                            if(corSelecionado > i) {
                                //trocando a cor dos nodes
                                vetorClusterNodes.at(e).at(z)->setCor(corSelecionado);
                                //passando para o cluster selecionado os novos nodes
                                vectorNode.at(corSelecionado).emplace_back(getNode(vetorClusterNodes.at(e).at(z)->getId()));
                                // voltando o verificado para false para que eles possam ser visitados novamente nos novos cluster
                                verificados[vetorClusterNodes.at(e).at(z)->getIdNode()] = false;

                            }
                        }

                        int contSelecionado = 0;

                        //verificando se o conjunto de node do subcluster altera de alguma forma o gap do novo cluster que eles vão ser inseridos
                        if(menorOuMaior.at(contSelecionado).back() == 0) {

                            listMaiorMenorPeso.at(corSelecionado).at(0) = maiorMenorValSubCluster.at(e).at(0);
                            listMaiorMenorPeso.at(corSelecionado).at(1) = maiorMenorValSubCluster.at(e).at(1);
                        } else if(menorOuMaior.at(contSelecionado).back() == 1) {
    
                            listMaiorMenorPeso.at(corSelecionado).at(0) = maiorMenorValSubCluster.at(e).at(0);
                        } else if(menorOuMaior.at(contSelecionado).back() == 2) {

                            listMaiorMenorPeso.at(corSelecionado).at(1) = maiorMenorValSubCluster.at(e).at(1);
                        }

                        menorOuMaior.at(0).clear();

                        contEntradas = 0;


                        for(int z=0;z<corNode.size();z++) {
                            if(z != i) {
                                corNode.at(z) = true;
                            }
                        }
                    }
                }
                // atualizando a lista com maior e menor peso de cada cluster
                listMaiorMenorPeso.at(i).front() = maiorMenorValSubCluster.at(posicaoMaiorSubCluster).front();
                listMaiorMenorPeso.at(i).back() = maiorMenorValSubCluster.at(posicaoMaiorSubCluster).back();
                coresPossiveis.clear();
                menorOuMaior.clear();
                if(i == p-1)
                {
                    delete verificados;
                }
            }
            
            int gapTotal = 0;

            for(int h=0;h<listMaiorMenorPeso.size();h++) {
                gapTotal += listMaiorMenorPeso.at(h).at(0) - listMaiorMenorPeso.at(h).at(1);
            }

            delete visitado;

            /*for(int u=0;u<p;u++) {
                output_file << "Cluster é: " << u <<endl;
                for(Node *n = this->first_node;n != nullptr; n = n->getNextNode()) {
                    if(n->getCor() == u) {
                        output_file << "Cor do node: " << n->getCor() << " Id do node: " << n->getId() << " Peso: " << n->getWeight() << endl;
                    }
                }
                output_file << "Maior dessa posicao: " << listMaiorMenorPeso.at(u).front() << " Menor dessa posicao: " << listMaiorMenorPeso.at(u).back() << endl;
                output_file << "Gap dessa posicao: " << listMaiorMenorPeso.at(u).front() - listMaiorMenorPeso.at(u).back() << endl;
            }*/
            vetIter[e] = gapTotal;
            if(e == 0) {
                menorGap = gapTotal;
            } else {
                if(menorGap > gapTotal ) {
                    menorGap = gapTotal;
                }
            }
            listMaiorMenorPeso.clear();
        } else {
            output_file << "O grafo não tem peso nas arestas" << endl;
        }
        delete visitado;
    }

    int soma = 0;
    for(int i = 0;i<valRep;i++) 
    {
        soma += vetIter[i];
        //output_file << "Posicao " << i << " Gap: " << vetIter[i] << endl;
    }
    soma = soma/valRep;
    delete vetIter;
    output_file << "Valor da media: " << soma << endl;
    output_file << "Menor gap é: " << menorGap << endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    output_file << "Tempo de compilação:  " << float_ms.count() << " milliseconds" << std::endl;
}

void Graph::GulosoRandomizado(ofstream &output_file, int p, float alfa, int numIter)
{
    auto start = std::chrono::high_resolution_clock::now();
    int *gapFinais = new int[numIter];
    int menorGap = 0;
    for(int e = 0;e<numIter;e++) {

        bool *visitado = new bool[this->order];  // vetor para verificar os vértices já utilizados

        for(int i=0;i<this->order;i++)
        {
            visitado[i] = false; // marcando todos nodes como não visitados
        }

        if(this->weighted_node) // só pode grafo com node com peso
        {
            vector<vector<Node*>> vectorNode; //Note space between "> >" // vetor de vetores de node
            vectorNode.reserve(p); // reservando tamanho para o vector = número de clusters passado
            for(int i=0;i<p;i++) {
                vectorNode.push_back(criaVectorTeste()); // criando os vetores de node;
                vectorNode.at(i).reserve(this->order); 
            }

            //srand(time(0)); // semente aleatoria
            //Sleep(1000);
            for(int i=0;i<p;i++) // montando os cluster iniciais  
            {
                Node *nodeAux; 
                bool vizinho = false;
                do {
                    int x = 1 + (rand() % this->order-1); // escolhendo número aleatorio
                    nodeAux = this->getNodeId(x); // pegando node referente a esse número
                    vizinho = false;
                    if(i > 0) // não deixar nodes vizinhos juntos
                    {
                        if(!visitado[nodeAux->getIdNode()]) // caso o node escolhido ainda não tenha sido visitado
                        {
                            for(Edge *edgeAux = nodeAux->getFirstEdge(); edgeAux!=nullptr ; edgeAux = edgeAux->getNextEdge()) // percorrendo a aresta para ver se os vertices escolhidos até então não são vizinhos
                            {
                                for(int g=0;g<i;g++)
                                {
                                    for(int f=0;f<vectorNode.at(g).size();f++)
                                    {
                                        if(edgeAux->getTargetId() == vectorNode.at(g).at(f)->getId())
                                        {
                                            vizinho = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                } while(visitado[nodeAux->getIdNode()] || vizinho); // se o node já tiver sido colocado ele troca
                visitado[nodeAux->getIdNode()] = true; // marcando como visitado

                for(Edge *edgeAux = nodeAux->getFirstEdge(); edgeAux != nullptr; edgeAux = edgeAux->getNextEdge())
                {
                    if(getNode(edgeAux->getTargetId())->getInDegree() == 1) // verificando se a aresta ao nó escolhido só tem o nó escolhido como vizinho
                    {
                        vectorNode.at(i).emplace_back(getNode(edgeAux->getTargetId())); // Coloca o vizinho de grau 1 na lista
                        visitado[edgeAux->getTargetIdNode()] = true;  // Coloca o node vizinho como já utilizado
                        visitado[nodeAux->getIdNode()] = true; // coloca o node escolhido como já utilizado

                    }    

                }

                if(nodeAux->getInDegree() == 1) { // se o nó escolhido tem grau de entrada 1 já pega o vizinho dele junto
                    vectorNode.at(i).emplace_back(nodeAux);  // caso o node só tenha uma aresta a gente vai inserir o único vizinho direto na lista que o vizinho tá
                    vectorNode.at(i).emplace_back(getNode(nodeAux->getFirstEdge()->getTargetId())); // único vizinho direto já pode pegar direto no getFirstEdge()
                    visitado[nodeAux->getIdNode()] = true;  // Coloca o node como já utilizado
                    visitado[nodeAux->getFirstEdge()->getTargetIdNode()] = true; // coloca o vizinho do node como já utilizado

                } else {
                    vectorNode.at(i).emplace_back(nodeAux); // inserindo esse node na lista da posição i do vector
                    visitado[nodeAux->getIdNode()] = true;   // Coloca o vértice como já utilizado

                }
            }

            for(int q=0;q<vectorNode.size();q++) {
                for(int l=0;l<vectorNode.at(q).size();l++) {
                    vectorNode.at(q).at(l)->setCor(q);
                    visitado[vectorNode.at(q).at(l)->getIdNode()] = true;
                }
            }
        
            vector<Node*> vectorWeightEdge; // nodes que ainda não foram inseridos em nenhum cluster
            vectorWeightEdge.reserve(this->order-p);
            vector<vector<float>> listRank;// vector de ranqueamento dos nodes
            int contadora = 0;
            listRank.reserve(p); // reservando espaço para o total de clusters nesse vector 
            for(int i=0;i<p;i++) {
                vector<float> *rank = new vector<float>;
                listRank.push_back(*rank);
                listRank.at(i).reserve(this->order-p);
            }

            // Adicionando os vertices que nao foram inseridos inicialmente na vectorWeightEdge
            for(Node *node = this->first_node;node != nullptr;node = node->getNextNode())
            {
                if(!visitado[node->getIdNode()])
                {       
                    vectorWeightEdge.emplace_back(node);
                }
            }

            vector<vector<int>> listMaiorMenorPeso; // lista com maior e menor peso de cada cluster
            listMaiorMenorPeso.reserve(p);

            for(int i=0;i<p;i++) {
                vector<int> *rank = new vector<int>;
                listMaiorMenorPeso.emplace_back(*rank);
                listMaiorMenorPeso.at(i).reserve(2); // para cada cluster existem 2 posicoes, uma com o maior e outra com o menor peso de cada cluster
                listMaiorMenorPeso.at(i).insert(listMaiorMenorPeso.at(i).begin(),-1);
                listMaiorMenorPeso.at(i).insert(listMaiorMenorPeso.at(i).end(),1000000);
            }

            do { // fazendo enquanto existirem vertices para ser alocado em qualquer cluster
                for(int i=0;i<p;i++) { // fazendo isso para todos os p(cluster)
                    contadora = 0;
                    int *idNodesAux = new int[vectorWeightEdge.size()];  
                    float *idRazao = new float[vectorWeightEdge.size()];
                    float menorVal;   
                    int contPosicao = 0;
                    float gap = 0;
                    float maiorValor = vectorNode.at(i).at(0)->getWeight();
                    float menorValor = vectorNode.at(i).at(0)->getWeight();
                    for(int j=0;j<vectorNode.at(i).size();j++) { // salvando o maior e menor valor de cada cluster até o momento
                        if(maiorValor < vectorNode.at(i).at(j)->getWeight()) {
                            maiorValor = vectorNode.at(i).at(j)->getWeight();
                        } else if(menorValor > vectorNode.at(i).at(j)->getWeight()) {
                            menorValor = vectorNode.at(i).at(j)->getWeight();
                        }
                    }

                    listMaiorMenorPeso.at(i).at(0) = maiorValor; // passando esses valores para a lista com maior e menor peso de cada cluster
                    listMaiorMenorPeso.at(i).at(1) = menorValor; //
                    
                    gap = maiorValor - menorValor;

                    for(int j=0;j<vectorWeightEdge.size();j++) {
                        
                        float gapNode;
                        float gapFinal;

                        if(vectorWeightEdge.at(j)->getWeight() > maiorValor) { // verificando para cada node não selecionado ainda a diferença de peso entre eles e o maior,menor peso presente em cada cluster
                            gapNode = vectorWeightEdge.at(j)->getWeight() - menorValor;
                        } else if(vectorWeightEdge.at(j)->getWeight() < menorValor) {
                            gapNode = maiorValor - vectorWeightEdge.at(j)->getWeight();
                        } else {
                            gapNode = 0;
                        }
                        //gapNode = gapNode - gap;
                        if(gapNode < 0) {
                            gapNode *= -1;
                        }
                        gapFinal = gapNode / vectorWeightEdge.at(j)->getTotal_Edge(); // seleção final busca selecionar o menor gapFinal que é o gapNode/ número de arestas de cada node

                        listRank.at(i).emplace_back(gapFinal); // lista de ranqueamento de vértices
                        idNodesAux[contadora] = vectorWeightEdge.at(j)->getId();
                        idRazao[contadora] = gapFinal;
                        contadora++;
                    }
                    int indiceR = 0;
                    if(listRank.at(i).size() > 1) {
                        listRank.shrink_to_fit();
                        sort(listRank.at(i).begin(), listRank.at(i).end());
                        /*float auxRazao;
                        int auxId;
                        int q, c;
                        
                        //Ordenando a lista ranqueada 
                        /*for(int q=0; q<listRank.at(i).size(); q++ ){
                            for(int c=q+1; c<listRank.at(i).size(); c++ ){
                                if( idRazao[q] > idRazao[c] ){
                                    auxRazao = idRazao[c];
                                    idRazao[c] = idRazao[q];
                                    idRazao[q] = auxRazao;
                                    auxId = idNodesAux[c];
                                    idNodesAux[c] = idNodesAux[q];
                                    idNodesAux[q] = auxId; 
                                }    
                            }
                        }
                        /*for(int h=0;h<listRank.at(i).size();h++)
                        {
                            //output_file << "ListRank " << i << ": " << listRank.at(i).at(h) << endl;
                        }*/
                        selection_sort(idRazao,idNodesAux,listRank.at(i).size());
                        indiceR = (rand() % (listRank.at(i).size()-1)*alfa); // escolhendo número aleatorio
                        
                        contPosicao = idNodesAux[indiceR];
                    }
                    for(int q=0;q<vectorWeightEdge.size();q++)
                    {
                        if(contPosicao == vectorWeightEdge.at(q)->getId())
                        {
                            contPosicao = q;
                        }
                    }
                    if((vectorWeightEdge.size() > 0) && !visitado[vectorWeightEdge.at(contPosicao)->getIdNode()]) // caso a lista não esteja vazia
                    {
                        vectorWeightEdge.at(contPosicao)->setCor(i); // setando a cor para a cor atual do cluster

                        visitado[vectorWeightEdge.at(contPosicao)->getIdNode()] = true; // marcando o node escolhido como true

                        if(vectorWeightEdge.at(contPosicao)->getWeight() > listMaiorMenorPeso.at(i).at(0)) { // atualizando os valores de maior e menor de cada cluster

                            listMaiorMenorPeso.at(i).at(0) = vectorWeightEdge.at(contPosicao)->getWeight();
                        } else if(vectorWeightEdge.at(contPosicao)->getWeight() < listMaiorMenorPeso.at(i).at(1)) {

                            listMaiorMenorPeso.at(i).at(1) = vectorWeightEdge.at(contPosicao)->getWeight();
                        }

                        vectorNode.at(i).emplace_back(vectorWeightEdge.at(contPosicao)); // adicionando o node esclhido ao cluster 

                        for(Edge *edge = vectorWeightEdge.at(contPosicao)->getFirstEdge();edge != nullptr;edge = edge->getNextEdge()) { // verificando se existe vizinho com in degree == 1
                            //verificando se algum vizinho do node escolhido tem grau de entrada 1 e nao foi visitado ainda
                            if((getNode(edge->getTargetId())->getInDegree() == 1) && !visitado[getNode(edge->getTargetId())->getIdNode()] ) {

                                visitado[getNode(edge->getTargetId())->getIdNode()] = true;
                                for(int i=0;i<vectorWeightEdge.size();i++) {
                                    if(vectorWeightEdge.at(i)->getId() == getNode(edge->getTargetId())->getId()) {
                                        vectorWeightEdge.erase(vectorWeightEdge.begin() + i);
                                    }
                                }

                                //Atualizando o maior e o menor peso de cada cluster
                                if(getNode(edge->getTargetId())->getWeight() > listMaiorMenorPeso.at(i).at(0)) {
                                    listMaiorMenorPeso.at(i).at(0) = getNode(edge->getTargetId())->getWeight();
                                } else if(getNode(edge->getTargetId())->getWeight() < listMaiorMenorPeso.at(i).at(1)) {
                                    listMaiorMenorPeso.at(i).at(1) = getNode(edge->getTargetId())->getWeight();
                                }
                                // adicionando o node ao cluster
                                vectorNode.at(i).emplace_back(getNode(edge->getTargetId()));
                            }
                        }
                        int posicaoNode = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode();
                        // Verificando se o node atual tem in degree == 1
                        if((vectorWeightEdge.at(contPosicao)->getInDegree() == 1) && !visitado[getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode()]) {
                            
                            vectorNode.at(i).emplace_back(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId()));
                            // atualizando maior e menor peso de cada cluster
                            if(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight() > listMaiorMenorPeso.at(i).at(0)) {
                                listMaiorMenorPeso.at(i).at(0) = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight();
                            } else if(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight() < listMaiorMenorPeso.at(i).at(1)) {
                                listMaiorMenorPeso.at(i).at(1) = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight();
                            }
                            //retirando o vizinho do node com in degree == 1 da lista de nodes que ainda não entraram
                            for(int vecCont = 0; vecCont < vectorWeightEdge.size(); vecCont++)
                            {
                                if(vectorWeightEdge.at(vecCont)->getId() == vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())
                                {
                                    //retirando o node selecionado da lista de nodes ainda nao selecionados
                                    vectorWeightEdge.erase(vectorWeightEdge.begin() + vecCont);
                                }

                            }
                            if(contPosicao != 0 )
                            {
                                contPosicao--;
                            }
                            // atualizando a lista de visitado
                            visitado[getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode()] = true;
                        }

                        vector<Node*>::iterator n;
                        
                        n = vectorWeightEdge.begin(); 

                        advance(n, contPosicao);
                        // retirando o node escolhido no ranqueamento da vectorWeightEdge
                        vectorWeightEdge.erase(n);
                        vectorWeightEdge.shrink_to_fit();
                        //limpando a lista para fazer novamente
                        listRank.at(i).clear();
                        listRank.reserve(listRank.capacity()-1);
                        
                    }

                    delete idNodesAux;  
                    delete idRazao;

                }
            // enquanto ainda existirem nodes no vector
            } while(!vectorWeightEdge.empty());
        
            listRank.clear(); // não serão mais utilizadas
            vectorWeightEdge.clear(); // não serão mais utilizadas

            // A partir daqui estou montando os subcluster(partições individuais de cada conjunto de nodes dentro de um mesmo cluster)
            for(int i =0;i<p;i++)
            {

                int contadoraSubCluster = 0;
                bool *verificados = new bool[this->order]; // vetor de verificados
                int contClusterAux = 1; // numero de subCluster
                for(int j =0;j<this->order;j++) {
                    verificados[j] = false;
                }
                vector<vector<int>> maiorMenorValSubCluster; // maior e menor val de cada subcluster serao salvos aqui
                maiorMenorValSubCluster.reserve(this->order);
                vector<vector<Node*>> vetorClusterNodes; // cada subcluster sera armazenado aqui
                vetorClusterNodes.reserve(this->order);
                for(int j=0;j<this->order;j++) {
                    vector<int> *rank = new vector<int>;
                    vetorClusterNodes.emplace_back(criaVectorTeste());
                    maiorMenorValSubCluster.emplace_back(*rank);
                    maiorMenorValSubCluster.at(j).reserve(2);

                }
                vetorClusterNodes.at(0).insert(vetorClusterNodes.at(0).begin(), vectorNode.at(i).at(0));      
                vectorNode.at(i).erase(vectorNode.at(i).begin());

                int contSameCluster;
                int contSubCluster = 1;

                for(int k=0;k<contClusterAux;k++) // responsavel por criar um novo subcluster 
                {
                    contadoraSubCluster++;
                    contSameCluster = 0;

                    // bloco responsavel por atualizar os valores de maior e menor de cada subcluster
                    maiorMenorValSubCluster.at(k).front() = vetorClusterNodes.at(k).at(0)->getWeight();
                    maiorMenorValSubCluster.at(k).back() = vetorClusterNodes.at(k).at(0)->getWeight();

                    int maior = vetorClusterNodes.at(k).at(0)->getWeight();
                    int menor = vetorClusterNodes.at(k).at(0)->getWeight();
        
                    for(int j=0;j<vetorClusterNodes.at(k).size();j++) // j começa menor que 1 e vai atualizando o tamanho dentro do for  
                    {
                        Node *node = vetorClusterNodes.at(k).at(j); // passando o node atual
                        verificados[node->getIdNode()] = true; // marcando o node como true                    
                        int tam = node->getTotal_Edge(); // pegando o total de arestas do node
                        int *vizinhos = new int[tam];
                        int contAuxVizinhos = 0;
                        bool inseriu = false;

                        for(Edge *edge = node->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) {
                            // verificando se o node atual tem vizinhos da mesma cor e que não foram verificados
                            if((getNode(edge->getTargetId())->getCor() == node->getCor()) && !verificados[edge->getTargetIdNode()]) {
                                    
                                vetorClusterNodes.at(k).emplace_back(getNode(edge->getTargetId()));
                                vizinhos[contAuxVizinhos] = edge->getTargetId();
                                contAuxVizinhos++;
                                inseriu = true;
                            }
                        }

                        // retirando os vizinhos da vectorNode(vetor de cluster)
                        for(int l=0;l<contAuxVizinhos;l++) {
                            for(int aux=0;aux<vectorNode.at(i).size();aux++) {
                                if(vectorNode.at(i).at(aux)->getId() == vizinhos[l]) {
                                    vectorNode.at(i).erase(vectorNode.at(i).begin() + aux);
                                }
                            }
                        }

                        // atualizando em mais 1 o numero de cluster 
                        if((inseriu == false) && !vectorNode.at(i).empty()) {
                            if(vetorClusterNodes.at(k).size()-(j+1) <= 0) {
                                vetorClusterNodes.at(k+1).emplace_back(vectorNode.at(i).at(0));
                                contClusterAux++;
                                vectorNode.at(i).erase(vectorNode.at(i).begin());
                            }
                        }
                        // atualizando o maior e menor valores
                        if(inseriu)
                        {
                            for(int y = j;y<=vetorClusterNodes.at(k).size()-1/*contAuxVizinhos*/;y++) {
                                if(maior < vetorClusterNodes.at(k).at(y)->getWeight())
                                {
                                    maior = vetorClusterNodes.at(k).at(y)->getWeight();
                                } 
                                else if( menor > vetorClusterNodes.at(k).at(y)->getWeight())
                                {
                                    menor = vetorClusterNodes.at(k).at(y)->getWeight();
                                }
                            }
                        } 
                        delete vizinhos;  
                    }
                    maiorMenorValSubCluster.at(k).insert(maiorMenorValSubCluster.at(k).begin(), maior);
                    maiorMenorValSubCluster.at(k).insert(maiorMenorValSubCluster.at(k).end(), menor);  
                } 
                
                vetorClusterNodes.resize(contadoraSubCluster);

                int maiorSubCluster = vetorClusterNodes.at(0).size(); // pegando o size do primeiro subcluster de cada cor(cada vez que o for com i < p roda)
                for(int e=0;e<vetorClusterNodes.size();e++) {
                    if(maiorSubCluster < vetorClusterNodes.at(e).size()) {
                        maiorSubCluster = vetorClusterNodes.at(e).size(); // verificando qual o maior subcluster de cada cor(cada vez que o for com i < p roda)
                    }
                }     

                bool entrou = false;
                int gapFinalSubCluster = -1;
                int posicaoMaiorSubCluster;
                // escolhendo o maior subcluster(numero de nodes) e com o menor gap para manter
                for(int e=0;e<vetorClusterNodes.size();e++) { // e < tamanho de subclusters existentes
                    if(maiorSubCluster == vetorClusterNodes.at(e).size()) { // salvando o gap do maior subcluster(maior no sentido de vértices presentes)
                        if(entrou == false) { // primeira vez a entrar
                            entrou = true;
                            gapFinalSubCluster = maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1);
                            posicaoMaiorSubCluster = e;
                        } else { // buscando o maior subcluster com o menor gap
                            if(gapFinalSubCluster > (maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1))) {
                                gapFinalSubCluster = maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1);
                                posicaoMaiorSubCluster = e;
                            }
                        }
                    }
                }

                vector<bool> corNode; //salva todas as cores menos a do cluster atual como true;
                corNode.reserve(p); // reservando as cores conforme o número de clusters solicitados
                corNode.insert(corNode.begin(), true);

                for(int n = 0;n<p;n++) { 
                    // marcando todas as cores como true, posivel de visitar
                    corNode.insert(corNode.begin() + n, true);
                }
                corNode.at(i) = false; // marcando a cor atual como false, pois não queremos ligar um subcluster em outro de mesma cor dele


                vector<vector<int>> coresPossiveis; // 1° com a cor e a 2° com o gap, vai ajudar a reajustar os subcluster 
                coresPossiveis.reserve(1);

                for(int x=0;x<1;x++) {
                    vector<int> *rank = new vector<int>;
                    coresPossiveis.push_back(*rank);
                    coresPossiveis.at(x).reserve(2); // 2 posições, 1° com a cor e a 2° com o gap
                    coresPossiveis.at(x).insert(coresPossiveis.at(x).begin(), 1000000);
                    coresPossiveis.at(x).insert(coresPossiveis.at(x).end(), x);
                }

                vector<vector<int>> menorOuMaior; // auxilia para mudar o valor final de cada cluster
                menorOuMaior.reserve(1);
                for(int e=0;e<1;e++) {
                    vector<int> *rank = new vector<int>;
                    menorOuMaior.push_back(*rank);
                    menorOuMaior.at(e).reserve(2);
                }

                int contEntradasArestas;
                int contEntradas = 0;
                int gap;
                
                for(int e=0;e<vetorClusterNodes.size();e++) { // e < que o total de subclusters no i(cor atual)
                    if(e != posicaoMaiorSubCluster) { // e sendo diferente do subcluster que a gente quer manter(no caso a posição dele no vetorClusterNodes->at(e))
                        gap = 10000000;
                        for(int z = 0;z < vetorClusterNodes.at(e).size();z++) { // z < que a quantidade de nodes presentes em cada subcluster

                            contEntradasArestas = 0;

                            for(Edge *edge = vetorClusterNodes.at(e).at(z)->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) { // verificando as arestas de cada subvertice
                                    
                                if(corNode.at(getNode(edge->getTargetId())->getCor())) { // se a cor do node estiver como true, ou seja não foi verificada ainda e nem é a cor do i
                                    corNode.at(getNode(edge->getTargetId())->getCor()) = false; // marca a cor como visitada
                                        
                                    if(maiorMenorValSubCluster.at(e).front() > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() && maiorMenorValSubCluster.at(e).back() < listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back()) {

                                        if(gap > maiorMenorValSubCluster.at(e).front() - maiorMenorValSubCluster.at(e).back()) {
                                                
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = 0;
                                            gap = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front();
                                            gap += listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back() - maiorMenorValSubCluster.at(e).back(); 
                                            coresPossiveis.at(0).at(0) = gap;
                                            coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();

                                        }
                                    } else if(maiorMenorValSubCluster.at(e).front() > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(0)) {
                                        if(gap > maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(0)) {
                                            coresPossiveis.at(0).at(0) = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front();
                                            coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = 1;
                                            gap = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(1);

                                        }
                                    } else if(maiorMenorValSubCluster.at(e).back() < listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back()) {
                                        if(gap > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster.at(e).back()) {
                                            coresPossiveis.at(0).at(0) = listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back() - maiorMenorValSubCluster.at(e).back();
                                            coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = 2;

                                            gap = listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster.at(e).back();
                                        }
                                    } else {
                                        if(gap > 0) {
                                            coresPossiveis.at(0).front() = 0;
                                            coresPossiveis.at(0).back() = getNode(edge->getTargetId())->getCor();
                                            menorOuMaior.at(0).front() = z;
                                            menorOuMaior.at(0).back() = -1;
                                            contEntradas++;
                                            gap = 0;
                                        }
                                    }
                                }
                            }    
                        }

                        int menor = coresPossiveis.at(0).front(); // pegando o primeiro gap
                        int corSelecionado = coresPossiveis.at(0).back(); // pegando a primeira cor selecionada
                        int contPosicaoSubCluster = 0;

                        // atualizando a cor dos nodes para os novos cluster e adicionando os nodes nesses novos clusters
                        for(int z=0;z<vetorClusterNodes.at(e).size();z++) {
                            getNode(vetorClusterNodes.at(e).at(z)->getId())->setCor(corSelecionado);
                            if(corSelecionado > i) {
                                //trocando a cor dos nodes
                                vetorClusterNodes.at(e).at(z)->setCor(corSelecionado);
                                //passando para o cluster selecionado os novos nodes
                                vectorNode.at(corSelecionado).emplace_back(getNode(vetorClusterNodes.at(e).at(z)->getId()));
                                // voltando o verificado para false para que eles possam ser visitados novamente nos novos cluster
                                verificados[vetorClusterNodes.at(e).at(z)->getIdNode()] = false;

                            }
                        }

                        int contSelecionado = 0;

                        //verificando se o conjunto de node do subcluster altera de alguma forma o gap do novo cluster que eles vão ser inseridos
                        if(menorOuMaior.at(contSelecionado).back() == 0) {
                            listMaiorMenorPeso.at(corSelecionado).at(0) = maiorMenorValSubCluster.at(e).at(0);
                            listMaiorMenorPeso.at(corSelecionado).at(1) = maiorMenorValSubCluster.at(e).at(1);
                                
                        } else if(menorOuMaior.at(contSelecionado).back() == 1) {
                            listMaiorMenorPeso.at(corSelecionado).at(0) = maiorMenorValSubCluster.at(e).at(0);
                        } else if(menorOuMaior.at(contSelecionado).back() == 2) {

                            listMaiorMenorPeso.at(corSelecionado).at(1) = maiorMenorValSubCluster.at(e).at(1);
                        }

                        menorOuMaior.at(0).clear();

                        contEntradas = 0;

                        for(int z=0;z<corNode.size();z++) {
                            if(z != i) {
                                corNode.at(z) = true;
                            }
                        }
                    }
                }
                // atualizando a lista com maior e menor peso de cada cluster    
                listMaiorMenorPeso.at(i).front() = maiorMenorValSubCluster.at(posicaoMaiorSubCluster).front();
                listMaiorMenorPeso.at(i).back() = maiorMenorValSubCluster.at(posicaoMaiorSubCluster).back();
                coresPossiveis.clear();
                menorOuMaior.clear();
                if(i == p-1)
                {
                    delete verificados;
                }
            }

            int gapTotal = 0;

            for(int h=0;h<listMaiorMenorPeso.size();h++) {
                gapTotal += listMaiorMenorPeso.at(h).at(0) - listMaiorMenorPeso.at(h).at(1);
            }
             
            delete visitado;
            
            gapFinais[e] = gapTotal;
            if(e == 0) 
            {
                menorGap = gapTotal;
            } else {
                if(menorGap > gapTotal) {
                    menorGap = gapTotal;
                }
            }

            listMaiorMenorPeso.clear();
        } else {
            output_file << "O grafo não tem peso nas arestas" << endl;
        }
    }
    int soma = 0;
    for(int i = 0;i<numIter;i++) 
    {
        soma += gapFinais[i];
        //output_file << "Posicao " << i << " Gap: " << gapFinais[i] << endl;
    }
    soma = soma/numIter;
    delete gapFinais;
    output_file << "Valor da media: " << soma << endl;
    output_file << "Menor gap é: " << menorGap << endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    //output_file << "Tempo de compilação " << float_ms.count() << " milliseconds" << std::endl;
    output_file << "Tempo de compilação " << float_ms.count() << "seconds: " << std::endl;
}

void Graph::GulosoRandomizadoReativo(ofstream &output_file, int p, float *alfa, int numIter, int blocoIter, int m)
{
    auto start = std::chrono::high_resolution_clock::now();
    int menorGap = INT_MAX; // melhor gap entre todos os alfas;
    float *medias = new float[m]; // media dos gaps encontrados para cada alfa;
    float *vetProbAlfa = new float[m]; // probabilidade cada alfa(começa em 1 para todos)
    float *qAlfa = new float[m]; // q para cada alfa
    float somaGap; // somar os gaps de cada bloco
    int indiceAlfa = 0;
    int *qtdAlfa = new int[m];
    float melhorAlfa;
    for(int i=0;i<m;i++)
    {
        qtdAlfa[i] = 0;
        medias[i] = 1;
        vetProbAlfa[i] = 1;
        qAlfa[i] = 0;
    }
    int *gapFinais = new int[numIter];
    for(int e = 0;e<numIter;e++) {
        somaGap = 0;
        //output_file << "e = " << e << " ---------------------: " << endl;
        if(e != 0){
            if(e < blocoIter)
            {   
                indiceAlfa = (rand() % m-1);
                qtdAlfa[indiceAlfa]++;
                /*
                Como a probabilidade inicial é a mesma para todos, antes de atualizar o vet de probabilidades,
                basta escolher um indice de 0 até m-1 
                */
            }
            else {
                /*for(int x=0;x<m;x++)
                {
                    output_file << "Probabilidades " << x << " : " << vetProbAlfa[x] << endl;
                }*/
                indiceAlfa = escolheAlfa(output_file, vetProbAlfa, m);
                /*
                Agora estamos escolhendo o alfa com as probabilidades atualizadas
                */
                //output_file << "Alfa escolhido: " << alfa[indiceAlfa] << endl;
                qtdAlfa[indiceAlfa]++; 
            }
        }
        if(e % blocoIter == 0 && e != 0)
        {
            atualizaProbabilidades(output_file,vetProbAlfa, qAlfa, menorGap, medias, m);
        }    
            bool *visitado = new bool[this->order];  // vetor para verificar os vértices já utilizados

            for(int i=0;i<this->order;i++)
            {
                visitado[i] = false; // marcando todos nodes como não visitados
            }

            if(this->weighted_node) // só pode grafo com node com peso
            {
                vector<vector<Node*>> vectorNode;  // vetor de vetores de node
                vectorNode.reserve(p); // reservando tamanho para o vector = número de clusters passado
                for(int i=0;i<p;i++) {
                    vectorNode.push_back(criaVectorTeste()); // criando os vetores de node;
                    vectorNode.at(i).reserve(this->order);
                }

                //srand(time(0)); // semente aleatoria
                //Sleep(1000);
            for(int i=0;i<p;i++) // montando os cluster iniciais  
            {
                Node *nodeAux; 
                bool vizinho = false;
                do {
                    int x = 1 + (rand() % this->order-1); // escolhendo número aleatorio
                    nodeAux = this->getNodeId(x); // pegando node referente a esse número
                    vizinho = false;
                    if(i > 0) // não deixar nodes vizinhos juntos
                    {
                        if(!visitado[nodeAux->getIdNode()]) // caso o node escolhido ainda não tenha sido visitado
                        {
                            for(Edge *edgeAux = nodeAux->getFirstEdge(); edgeAux!=nullptr ; edgeAux = edgeAux->getNextEdge())
                            {
                                for(int g=0;g<i;g++)
                                {
                                    for(int f=0;f<vectorNode.at(g).size();f++)
                                    {
                                        if(edgeAux->getTargetId() == vectorNode.at(g).at(f)->getId())
                                        {
                                            vizinho = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                } while(visitado[nodeAux->getIdNode()] || vizinho); // se o node já tiver sido colocado ele troca
                visitado[nodeAux->getIdNode()] = true; // marcando como visitado

                for(Edge *edgeAux = nodeAux->getFirstEdge(); edgeAux != nullptr; edgeAux = edgeAux->getNextEdge())
                {
                    if(getNode(edgeAux->getTargetId())->getInDegree() == 1) // verificando se a aresta ao nó escolhido só tem o nó escolhido como vizinho
                    {
                        vectorNode.at(i).emplace_back(getNode(edgeAux->getTargetId())); // Coloca o vizinho de grau 1 na lista
                        visitado[edgeAux->getTargetIdNode()] = true;  // Coloca o node vizinho como já utilizado
                        visitado[nodeAux->getIdNode()] = true; // coloca o node escolhido como já utilizado

                    }    

                }

                if(nodeAux->getInDegree() == 1) { // se o nó escolhido tem grau de entrada 1 já pega o vizinho dele junto
                    vectorNode.at(i).emplace_back(nodeAux);  // caso o node só tenha uma aresta a gente vai inserir o único vizinho direto na lista que o vizinho tá
                    vectorNode.at(i).emplace_back(getNode(nodeAux->getFirstEdge()->getTargetId())); // único vizinho direto já pode pegar direto no getFirstEdge()
                    visitado[nodeAux->getIdNode()] = true;  // Coloca o node como já utilizado
                    visitado[nodeAux->getFirstEdge()->getTargetIdNode()] = true; // coloca o vizinho do node como já utilizado

                } else {
                    vectorNode.at(i).emplace_back(nodeAux); // inserindo esse node na lista da posição i do vector
                    visitado[nodeAux->getIdNode()] = true;   // Coloca o vértice como já utilizado

                }
            }

                for(int q=0;q<vectorNode.size();q++) {
                    for(int l=0;l<vectorNode.at(q).size();l++) {
                        vectorNode.at(q).at(l)->setCor(q);
                        visitado[vectorNode.at(q).at(l)->getIdNode()] = true;
                    }
                }
            
                vector<Node*> vectorWeightEdge; // nodes que ainda não foram inseridos em nenhum cluster
                vectorWeightEdge.reserve(this->order-p);
                vector<vector<float>> listRank; // vector de ranqueamento dos nodes
                int contadora = 0;
                listRank.reserve(p); // reservando espaço para o total de clusters nesse vector 
                for(int i=0;i<p;i++) {
                    vector<float> *rank = new vector<float>;
                    listRank.push_back(*rank);
                    listRank.at(i).reserve(this->order-p);
                }

                //Adicionando os vertices que não foram sorteados para a vectorWeightEdge 
                for(Node *node = this->first_node;node != nullptr;node = node->getNextNode())
                {
                    if(!visitado[node->getIdNode()])
                    {       
                        vectorWeightEdge.emplace_back(node);
                    }
                }

                vector<vector<int>> listMaiorMenorPeso; // lista com maior e menor peso de cada cluster
                listMaiorMenorPeso.reserve(p);

                for(int i=0;i<p;i++) {
                    vector<int> *rank = new vector<int>;
                    listMaiorMenorPeso.emplace_back(*rank);
                    listMaiorMenorPeso.at(i).reserve(2); // para cada cluster existem 2 posicoes, uma com o maior e outra com o menor peso de cada cluster
                    listMaiorMenorPeso.at(i).insert(listMaiorMenorPeso.at(i).begin(),-1);
                    listMaiorMenorPeso.at(i).insert(listMaiorMenorPeso.at(i).end(),1000000);
                }

                do { // fazendo enquanto existirem vertices para ser alocado em qualquer cluster
                    for(int i=0;i<p;i++) { // fazendo isso para todos os p(cluster)
                        contadora = 0;
                        int *idNodesAux = new int[vectorWeightEdge.size()];  
                        float *idRazao = new float[vectorWeightEdge.size()];
                        float menorVal;   
                        int contPosicao = 0;
                        float gap = 0;
                        float maiorValor = vectorNode.at(i).at(0)->getWeight();
                        float menorValor = vectorNode.at(i).at(0)->getWeight();
                        for(int j=0;j<vectorNode.at(i).size();j++) { // salvando o maior e menor valor de cada cluster até o momento
                            if(maiorValor < vectorNode.at(i).at(j)->getWeight()) {
                                maiorValor = vectorNode.at(i).at(j)->getWeight();
                            } else if(menorValor > vectorNode.at(i).at(j)->getWeight()) {
                                menorValor = vectorNode.at(i).at(j)->getWeight();
                            }
                        } 

                        listMaiorMenorPeso.at(i).at(0) = maiorValor; // passando esses valores para a lista com maior e menor peso de cada cluster
                        listMaiorMenorPeso.at(i).at(1) = menorValor; //
                        
                        gap = maiorValor - menorValor;

                        for(int j=0;j<vectorWeightEdge.size();j++) {
                            
                            float gapNode;
                            float gapFinal;

                            if(vectorWeightEdge.at(j)->getWeight() > maiorValor) { // verificando para cada node não selecionado ainda a diferença de peso entre eles e o maior,menor peso presente em cada cluster
                                gapNode = vectorWeightEdge.at(j)->getWeight() - menorValor;
                            } else if(vectorWeightEdge.at(j)->getWeight() < menorValor) {
                                gapNode = maiorValor - vectorWeightEdge.at(j)->getWeight();
                            } else {
                                gapNode = 0;
                            }
                            //gapNode = gapNode - gap;
                            if(gapNode < 0) {
                                gapNode *= -1;
                            }
                            gapFinal = gapNode / vectorWeightEdge.at(j)->getTotal_Edge(); // seleção final busca selecionar o menor gapFinal que é o gapNode/ número de arestas de cada node

                            listRank.at(i).emplace_back(gapFinal); // lista de ranqueamento de vértices
                            idNodesAux[contadora] = vectorWeightEdge.at(j)->getId();
                            idRazao[contadora] = gapFinal;
                            contadora++;
                        }
                        int indiceR = 0;
                        if(listRank.at(i).size() > 1) {
                            listRank.shrink_to_fit();
                            sort(listRank.at(i).begin(), listRank.at(i).end());
                            float auxRazao;
                            int auxId;
                            int q, c;

                            //Ordenando a lista ranqueada 
                            for(int q=0; q<listRank.at(i).size(); q++ ){
                                for(int c=q+1; c<listRank.at(i).size(); c++ ){
                                    if( idRazao[q] > idRazao[c] ){
                                        auxRazao = idRazao[c];
                                        idRazao[c] = idRazao[q];
                                        idRazao[q] = auxRazao;
                                        auxId = idNodesAux[c];
                                        idNodesAux[c] = idNodesAux[q];
                                        idNodesAux[q] = auxId;
                                    }    
                                }
                            }
                            /*if(i == 0)
                            {
                                for(int n=0;n<vectorWeightEdge.size();n++)
                                {
                                    output_file << "idRazao[" << n << "]: " << idRazao[n] << endl;
                                    output_file << "listRank.at(0).at(" << n << "): " << listRank.at(i).at(n) << endl; 
                                    output_file << "idNodesAux[" << n << "]: " << idNodesAux[n] << endl;
                                }
                            }*/

                            //output_file << "alfa[indiceAlfa]: " << alfa[indiceAlfa] << endl;
                            indiceR = (rand() % (listRank.at(i).size()-1)*(alfa[indiceAlfa])); // escolhendo número aleatorio
                            contPosicao = idNodesAux[indiceR];
                        }

                        for(int q=0;q<vectorWeightEdge.size();q++)
                        {
                            if(contPosicao == vectorWeightEdge.at(q)->getId())
                            {
                                contPosicao = q;
                            }
                        }

                        if((vectorWeightEdge.size() > 0) && !visitado[vectorWeightEdge.at(contPosicao)->getIdNode()]) // caso a lista não esteja vazia
                        {
                            vectorWeightEdge.at(contPosicao)->setCor(i); // setando a cor para a cor atual do cluster

                            visitado[vectorWeightEdge.at(contPosicao)->getIdNode()] = true; // marcando o node escolhido como true

                            if(vectorWeightEdge.at(contPosicao)->getWeight() > listMaiorMenorPeso.at(i).at(0)) { // atualizando os valores de maior e menor de cada cluster

                                listMaiorMenorPeso.at(i).at(0) = vectorWeightEdge.at(contPosicao)->getWeight();
                            } else if(vectorWeightEdge.at(contPosicao)->getWeight() < listMaiorMenorPeso.at(i).at(1)) {

                                listMaiorMenorPeso.at(i).at(1) = vectorWeightEdge.at(contPosicao)->getWeight();
                            }

                            vectorNode.at(i).emplace_back(vectorWeightEdge.at(contPosicao)); // adicionando o node esclhido ao cluster  

                            for(Edge *edge = vectorWeightEdge.at(contPosicao)->getFirstEdge();edge != nullptr;edge = edge->getNextEdge()) { // verificando se existe vizinho com in degree == 1
                                //verificando se algum vizinho do node escolhido tem grau de entrada 1 e nao foi visitado ainda
                                if((getNode(edge->getTargetId())->getInDegree() == 1) && !visitado[getNode(edge->getTargetId())->getIdNode()] ) {

                                    visitado[getNode(edge->getTargetId())->getIdNode()] = true;
                                    for(int i=0;i<vectorWeightEdge.size();i++) {
                                        if(vectorWeightEdge.at(i)->getId() == getNode(edge->getTargetId())->getId()) {
                                            vectorWeightEdge.erase(vectorWeightEdge.begin() + i);
                                        }
                                    }

                                    //Atualizando o maior e o menor peso de cada cluster
                                    if(getNode(edge->getTargetId())->getWeight() > listMaiorMenorPeso.at(i).at(0)) {
                                        listMaiorMenorPeso.at(i).at(0) = getNode(edge->getTargetId())->getWeight();
                                    } else if(getNode(edge->getTargetId())->getWeight() < listMaiorMenorPeso.at(i).at(1)) {
                                        listMaiorMenorPeso.at(i).at(1) = getNode(edge->getTargetId())->getWeight();
                                    }

                                    // adicionando o node ao cluster
                                    vectorNode.at(i).emplace_back(getNode(edge->getTargetId()));
                                }
                            }
                            int posicaoNode = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode();
                            // Verificando se o node atual tem in degree == 1
                            if((vectorWeightEdge.at(contPosicao)->getInDegree() == 1) && !visitado[getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode()]) {
                                
                                vectorNode.at(i).emplace_back(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId()));
                                // atualizando maior e menor peso de cada cluster
                                if(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight() > listMaiorMenorPeso.at(i).at(0)) {
                                    listMaiorMenorPeso.at(i).at(0) = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight();
                                } else if(getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight() < listMaiorMenorPeso.at(i).at(1)) {
                                    listMaiorMenorPeso.at(i).at(1) = getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getWeight();
                                }
                                //retirando o vizinho do node com in degree == 1 da lista de nodes que ainda não entraram
                                for(int vecCont = 0; vecCont < vectorWeightEdge.size(); vecCont++)
                                {
                                    if(vectorWeightEdge.at(vecCont)->getId() == vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())
                                    {
                                        //retirando o node selecionado da lista de nodes ainda nao selecionados
                                        vectorWeightEdge.erase(vectorWeightEdge.begin() + vecCont);
                                    }

                                }
                                if(contPosicao != 0 )
                                {
                                    contPosicao--;
                                }
                                // atualizando a lista de visitado
                                visitado[getNode(vectorWeightEdge.at(contPosicao)->getFirstEdge()->getTargetId())->getIdNode()] = true;
                            }

                            vector<Node*>::iterator n;
                            
                            n = vectorWeightEdge.begin(); 

                            advance(n, contPosicao);
                            // retirando o node escolhido no ranqueamento da vectorWeightEdge
                            vectorWeightEdge.erase(n);
                            vectorWeightEdge.shrink_to_fit();
                            //limpando a lista para fazer novamente
                            listRank.at(i).clear();
                            listRank.reserve(listRank.capacity()-1);
                            
                        }
                    
                        delete idNodesAux;  
                        delete idRazao;

                    }
                // enquanto ainda existirem nodes no vector 
                } while(!vectorWeightEdge.empty());

                listRank.clear(); // não serão mais utilizadas
                vectorWeightEdge.clear(); // não serão mais utilizadas
                
                for(int i =0;i<p;i++)
                {
                    int contadoraSubCluster = 0;
                    bool *verificados = new bool[this->order]; // vetor de verificados
                    int contClusterAux = 1; // numero de subCluster
                    for(int j =0;j<this->order;j++) {
                        verificados[j] = false;
                    }
                    vector<vector<int>> maiorMenorValSubCluster; // maior e menor val de cada subcluster serao salvos aqui
                    maiorMenorValSubCluster.reserve(this->order);
                    vector<vector<Node*>> vetorClusterNodes; // cada subcluster sera armazenado aqui
                    vetorClusterNodes.reserve(this->order);
                    for(int j=0;j<this->order;j++) {
                        vector<int> *rank = new vector<int>;
                        vetorClusterNodes.emplace_back(criaVectorTeste());
                        //vetorClusterNodes.at(i).reserve(this->order); // adicionei agora
                        maiorMenorValSubCluster.emplace_back(*rank);
                        maiorMenorValSubCluster.at(j).reserve(2);
                    }

                    vetorClusterNodes.at(0).insert(vetorClusterNodes.at(0).begin(), vectorNode.at(i).at(0)); // inserindo o primeiro node no cluster atual          
                    vectorNode.at(i).erase(vectorNode.at(i).begin()); // excluindo o primeiro node da lista com os cluster iniciais

                    int contSameCluster;
                    int contSubCluster = 1;

                    for(int k=0;k<contClusterAux;k++) // responsavel por criar um novo subcluster  
                    {
                        contadoraSubCluster++;
                        contSameCluster = 0;

                        // bloco responsavel por atualizar os valores de maior e menor de cada subcluster
                        maiorMenorValSubCluster.at(k).front() = vetorClusterNodes.at(k).at(0)->getWeight();
                        maiorMenorValSubCluster.at(k).back() = vetorClusterNodes.at(k).at(0)->getWeight();

                        int maior = vetorClusterNodes.at(k).at(0)->getWeight();
                        int menor = vetorClusterNodes.at(k).at(0)->getWeight();
            
                        for(int j=0;j<vetorClusterNodes.at(k).size();j++) // j começa menor que 1 e vai atualizando o tamanho dentro do for 
                        {
                            Node *node = vetorClusterNodes.at(k).at(j); // passando o node atual
                            verificados[node->getIdNode()] = true; // marcando o node como true                    
                            int tam = node->getTotal_Edge(); // pegando o total de arestas do node
                            int *vizinhos = new int[tam];
                            int contAuxVizinhos = 0;
                            bool inseriu = false;

                            for(Edge *edge = node->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) {
                                // verificando se o node atual tem vizinhos da mesma cor e que não foram verificados
                                if((getNode(edge->getTargetId())->getCor() == node->getCor()) && !verificados[edge->getTargetIdNode()]) {
                                        
                                    vetorClusterNodes.at(k).emplace_back(getNode(edge->getTargetId()));
                                    vizinhos[contAuxVizinhos] = edge->getTargetId();
                                    contAuxVizinhos++;
                                    inseriu = true;
                                }
                            }
                            
                            // retirando os vizinhos da vectorNode(vetor de cluster)
                            for(int l=0;l<contAuxVizinhos;l++) {
                                for(int aux=0;aux<vectorNode.at(i).size();aux++) {
                                    if(vectorNode.at(i).at(aux)->getId() == vizinhos[l]) {
                                        vectorNode.at(i).erase(vectorNode.at(i).begin() + aux);
                                    }
                                }
                            }

                            // atualizando em mais 1 o numero de cluster 
                            if((inseriu == false) && !vectorNode.at(i).empty()) {
                                if(vetorClusterNodes.at(k).size()-(j+1) <= 0) {
                                    vetorClusterNodes.at(k+1).emplace_back(vectorNode.at(i).at(0));
                                    contClusterAux++;
                                    vectorNode.at(i).erase(vectorNode.at(i).begin());
                                }
                            }
                            // atualizando o maior e menor valores
                            if(inseriu)
                            {
                                for(int y = j;y<=vetorClusterNodes.at(k).size()-1;y++) {
                                    if(maior < vetorClusterNodes.at(k).at(y)->getWeight())
                                    {
                                        maior = vetorClusterNodes.at(k).at(y)->getWeight();
                                    } 
                                    else if( menor > vetorClusterNodes.at(k).at(y)->getWeight())
                                    {
                                        menor = vetorClusterNodes.at(k).at(y)->getWeight();
                                    }
                                }
                            }   
                            delete vizinhos;  
                        }
                        maiorMenorValSubCluster.at(k).insert(maiorMenorValSubCluster.at(k).begin(), maior);
                        maiorMenorValSubCluster.at(k).insert(maiorMenorValSubCluster.at(k).end(), menor);
                    } 
                    
                    vetorClusterNodes.resize(contadoraSubCluster);

                    int maiorSubCluster = vetorClusterNodes.at(0).size(); // pegando o size do primeiro subcluster de cada cor(cada vez que o for com i < p roda)
                    for(int e=0;e<vetorClusterNodes.size();e++) {
                        if(maiorSubCluster < vetorClusterNodes.at(e).size()) {
                            maiorSubCluster = vetorClusterNodes.at(e).size(); // verificando qual o maior subcluster de cada cor(cada vez que o for com i < p roda)
                        }
                    }     

                    bool entrou = false;
                    int gapFinalSubCluster = -1;
                    int posicaoMaiorSubCluster;
                    // escolhendo o maior subcluster(numero de nodes) e com o menor gap para manter
                    for(int e=0;e<vetorClusterNodes.size();e++) { // e < tamanho de subclusters existentes
                        if(maiorSubCluster == vetorClusterNodes.at(e).size()) { // salvando o gap do maior subcluster(maior no sentido de vértices presentes)
                            if(entrou == false) { // primeira vez a entrar
                                entrou = true;
                                gapFinalSubCluster = maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1);
                                posicaoMaiorSubCluster = e;
                            } else { // buscando o maior subcluster com o menor gap
                                if(gapFinalSubCluster > (maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1))) {
                                    gapFinalSubCluster = maiorMenorValSubCluster.at(e).at(0) - maiorMenorValSubCluster.at(e).at(1);
                                    posicaoMaiorSubCluster = e;
                                }
                            }
                        }
                    }

                    vector<bool> corNode; //salva todas as cores menos a do cluster atual como true;
                    corNode.reserve(p); // reservando as cores conforme o número de clusters solicitados
                    corNode.insert(corNode.begin(), true);

                    for(int n = 0;n<p;n++) { 
                        // marcando todas as cores como true, posivel de visitar
                        corNode.insert(corNode.begin() + n, true);
                    }
                    corNode.at(i) = false; // marcando a cor atual como false, pois não queremos ligar um subcluster em outro de mesma cor dele


                    vector<vector<int>> coresPossiveis; // 1° com a cor e a 2° com o gap, vai ajudar a reajustar os subcluster 
                    coresPossiveis.reserve(1);

                    coresPossiveis.reserve(this->order);
                    for(int x=0;x<1;x++) {
                        vector<int> *rank = new vector<int>;
                        coresPossiveis.push_back(*rank);
                        coresPossiveis.at(x).reserve(2); // 2 posições, 1° com a cor e a 2° com o gap
                        coresPossiveis.at(x).insert(coresPossiveis.at(x).begin(), 1000000);
                        coresPossiveis.at(x).insert(coresPossiveis.at(x).end(), x);
                    }

                    vector<vector<int>> menorOuMaior; // auxilia para mudar o valor final de cada cluster 
                    menorOuMaior.reserve(1);
                    for(int e=0;e<1;e++) {
                        vector<int> *rank = new vector<int>;
                        menorOuMaior.push_back(*rank);
                        menorOuMaior.at(e).reserve(2);
                    }

                    int contEntradasArestas;
                    int contEntradas = 0;
                    int gap;
                    
                    for(int e=0;e<vetorClusterNodes.size();e++) { // e < que o total de subclusters no i(cor atual)
                        if(e != posicaoMaiorSubCluster) { // e sendo diferente do subcluster que a gente quer manter(no caso a posição dele no vetorClusterNodes->at(e))
                            gap = 10000000;
                            for(int z = 0;z < vetorClusterNodes.at(e).size();z++) { // z < que a quantidade de nodes presentes em cada subcluster

                                contEntradasArestas = 0;

                                for(Edge *edge = vetorClusterNodes.at(e).at(z)->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) { // verificando as arestas de cada subvertice
                                        
                                    if(corNode.at(getNode(edge->getTargetId())->getCor())) { // se a cor do node estiver como true, ou seja não foi verificada ainda e nem é a cor do i
                                        corNode.at(getNode(edge->getTargetId())->getCor()) = false; // marca a cor como visitada
                                            
                                        if(maiorMenorValSubCluster.at(e).front() > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() && maiorMenorValSubCluster.at(e).back() < listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back()) {

                                            if(gap > maiorMenorValSubCluster.at(e).front() - maiorMenorValSubCluster.at(e).back()) {
                                                    
                                                menorOuMaior.at(0).front() = z;
                                                menorOuMaior.at(0).back() = 0;
                                                gap = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front();
                                                gap += listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back() - maiorMenorValSubCluster.at(e).back(); 
                                                coresPossiveis.at(0).at(0) = gap;
                                                coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                                contEntradas++;
                                            }
                                        } else if(maiorMenorValSubCluster.at(e).front() > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(0)) {
                                            if(gap > maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(0)) {
                                                coresPossiveis.at(0).at(0) = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front();
                                                coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                                menorOuMaior.at(0).front() = z;
                                                menorOuMaior.at(0).back() = 1;
                                                gap = maiorMenorValSubCluster.at(e).front() - listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).at(1);
                                                contEntradas++;
                                            }
                                        } else if(maiorMenorValSubCluster.at(e).back() < listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back()) {
                                            if(gap > listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster.at(e).back()) {
                                                coresPossiveis.at(0).at(0) = listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).back() - maiorMenorValSubCluster.at(e).back();
                                                coresPossiveis.at(0).at(1) = getNode(edge->getTargetId())->getCor();
                                                menorOuMaior.at(0).front() = z;
                                                menorOuMaior.at(0).back() = 2;
                                                contEntradas++;
                                                gap = listMaiorMenorPeso.at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster.at(e).back();
                                            }
                                        } else {
                                            if(gap > 0) {
                                                coresPossiveis.at(0).front() = 0;
                                                coresPossiveis.at(0).back() = getNode(edge->getTargetId())->getCor();
                                                menorOuMaior.at(0).front() = z;
                                                menorOuMaior.at(0).back() = -1;
                                                contEntradas++;
                                                gap = 0;
                                            }
                                        }
                                    }
                                }    
                            }

                            int menor = coresPossiveis.at(0).front(); // pegando o primeiro gap
                            int corSelecionado = coresPossiveis.at(0).back(); // pegando a primeira cor selecionada
                            int contPosicaoSubCluster = 0;

                            for(int z=0;z<vetorClusterNodes.at(e).size();z++) {
                                //getNode(vetorClusterNodes.at(e).at(z)->getId())->setCor(corSelecionado);
                                if(corSelecionado > i) {

                                    vetorClusterNodes.at(e).at(z)->setCor(corSelecionado);
                                    vectorNode.at(corSelecionado).emplace_back(getNode(vetorClusterNodes.at(e).at(z)->getId()));
                                    verificados[vetorClusterNodes.at(e).at(z)->getIdNode()] = false;

                                }
                            }

                            int contSelecionado = 0;

                
                            if(menorOuMaior.at(contSelecionado).back() == 0) {
                                listMaiorMenorPeso.at(corSelecionado).at(0) = maiorMenorValSubCluster.at(e).at(0);
                                listMaiorMenorPeso.at(corSelecionado).at(1) = maiorMenorValSubCluster.at(e).at(1);
                                    
                            } else if(menorOuMaior.at(contSelecionado).back() == 1) {
                                listMaiorMenorPeso.at(corSelecionado).at(0) = maiorMenorValSubCluster.at(e).at(0);
                            } else if(menorOuMaior.at(contSelecionado).back() == 2) {

                                listMaiorMenorPeso.at(corSelecionado).at(1) = maiorMenorValSubCluster.at(e).at(1);
                            }

                            menorOuMaior.at(0).clear();
                            contEntradas = 0;

                            for(int z=0;z<corNode.size();z++) {
                                if(z != i) {
                                    corNode.at(z) = true;
                                }
                            }
                        }
                    }
                    // atualizando a lista com maior e menor peso de cada cluster    
                    listMaiorMenorPeso.at(i).front() = maiorMenorValSubCluster.at(posicaoMaiorSubCluster).front();
                    listMaiorMenorPeso.at(i).back() = maiorMenorValSubCluster.at(posicaoMaiorSubCluster).back();
                    coresPossiveis.clear();
                    menorOuMaior.clear();
                    if(i == p-1)
                    {
                        delete verificados;
                    }
                }

                int gapTotal = 0;
                for(int h=0;h<listMaiorMenorPeso.size();h++) {
                    gapTotal += listMaiorMenorPeso.at(h).at(0) - listMaiorMenorPeso.at(h).at(1);
                }
                
                delete visitado;
                /*for(int u=0;u<p;u++) {
                    output_file << "Cluster é: " << u <<endl;
                    for(Node *n = this->first_node;n != nullptr; n = n->getNextNode()) {
                        if(n->getCor() == u) {
                            output_file << "Cor do node: " << n->getCor() << " Id do node: " << n->getId() << " Peso: " << n->getWeight() << endl;
                        }
                    }
                    output_file << "Maior dessa posicao: " << listMaiorMenorPeso.at(u).front() << " Menor dessa posicao: " << listMaiorMenorPeso.at(u).back() << endl;
                    output_file << "Gap dessa posicao: " << listMaiorMenorPeso.at(u).front() - listMaiorMenorPeso.at(u).back() << endl;
                }*/
                gapFinais[e] = gapTotal;
                //output_file << "Posicao memoria: " << &gapFinais[0] << endl;
                if(e == 0) 
                {
                    melhorAlfa = alfa[0];
                    menorGap = gapTotal;
                } else {
                    if(menorGap > gapTotal) {
                        menorGap = gapTotal;
                        melhorAlfa = indiceAlfa;
                    }
                }
                somaGap += gapTotal;
                for(int u=0;u<p;u++) {
                    for(Node *n = this->first_node;n != nullptr; n = n->getNextNode()) {
                        if(n->getCor() == u) {
                            //output_file << "Cor do node: " << n->getCor() << " Id do node: " << n->getId() << " Peso: " << n->getWeight() << endl;
                            }
                        }
                }
                //output_file << "Gap total: " << gapTotal << endl;
                //output_file << "Posicao memoria: " << &gapFinais[0] << endl;
                /*for(int d = 0;d<=e;d++) 
                {
                    output_file << "Posicao " << d << " Gap: " << gapFinais[d] << endl;
                }*/
                listMaiorMenorPeso.clear();
            } else {
                output_file << "O grafo não tem peso nas arestas" << endl;
            }
            delete visitado;
            medias[indiceAlfa] = somaGap/blocoIter;

    }
    int soma = 0;
    for(int i = 0;i<numIter;i++) 
    {
        soma += gapFinais[i];
        //output_file << "Posicao " << i << " Gap: " << gapFinais[i] << endl;
    }
    soma = soma/numIter;
    output_file << "Valor da media: " << soma << endl;
    output_file << "Menor gap: " << menorGap << endl;
    //output_file << "Melhor alfa: " << melhorAlfa << endl;
    int maior = qtdAlfa[0];
    //output_file << "Alfa[0] foi selecionado: " << qtdAlfa[0] << endl;
    int idDoMaior = 0;
    for(int i=1;i<m;i++)
    {   
        //output_file << "Alfa[" << i << "] foi selecionado: " << qtdAlfa[i] << endl;
        if(qtdAlfa[i] > maior)
        {
            maior = qtdAlfa[i];
            idDoMaior = i;
        }
    }
    output_file << "Alfa mais escolhido: " << alfa[idDoMaior] << endl; 
    //delete medias;
    delete vetProbAlfa;
    delete qAlfa;
    delete gapFinais;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    output_file << "Tempo de compilação: " << float_ms.count() << " milliseconds" << std::endl;
    //guloso
    //arroz
    /*for(int i = 0;i<numIter;i++) 
    {
        output_file << "Posicao " << i << " Gap: " << gapFinais[i] << endl;
    }*/
    //output_file << "Menor gap é: " << menorGap << endl;
    
}

void Graph::atualizaProbabilidades(ofstream &output_file, float *vetProbAlfa, float *qAlfa, float melhorGap, float *medias, int m) 
{
    //qAlfa;
    float somas = 0;
    //output_file << "Melhor gap: " << melhorGap << endl;
    /*for(int i=0;i<m;i++)
    {
        output_file << "Medias: " << i << ": " << medias[i] << endl;
    }*/

    for(int i=0;i<m;i++)
    {
        qAlfa[i] = pow((melhorGap/medias[i]),10);
        //output_file << "qAlfa: " << qAlfa[i] << endl;
        somas += qAlfa[i];
    }
    for(int i=0;i<m;i++)
    {
        vetProbAlfa[i] = qAlfa[i]/somas;
        //output_file << "somas: " << somas << endl;
    }
    for(int i=0;i<m;i++)
    {
        //output_file << "Probilidade escolhida: " << vetProbAlfa[i] << endl;
    }

}

int Graph::escolheAlfa(ofstream &output_file, float *vetProbAlfa, int m)
{
    float *vet = new float[m];
    float soma = 0;
    for(int i=0;i<m;i++)
    {
        soma += vetProbAlfa[i];
        vet[i] = soma;
    }
    //0.75, 0.10, 0.15;
    //0.75, 0.85, 1;
    float val = FLOAT_MIN + (float)(rand()) / ((float)(RAND_MAX/(FLOAT_MAX - FLOAT_MIN)));
    //output_file << "Valor do val: " << val << endl;
    int indice = 0;
    for(int i=0;i<m;i++)
    {
        //output_file << "Vet[" << i << "]: " << vet[i] << endl;
        if(vet[i] >= val)
        {
            //output_file << "Vet de prob: " << vet[i] << endl;
            indice = i;
            break;
        }
    }
    delete vet;
    //output_file << "Indice retornado: " << indice << endl;
    return indice;
}

void Graph::selection_sort(float *v1,int *v2,int tam)
{
    int i, j, min, auxInt;
    float auxFloat;
    for(int i=0;i<(tam-1);i++)
    {
        min = i;
        for(j = (i+1); j<tam;j++)
        {
            if(v1[j] < v1[min])
            {
                min = j;
            }
        }
        if(v1[i] != v1[min])
        {
            auxFloat = v1[i];
            v1[i] = v1[min];
            v1[min] = auxFloat;
            auxInt = v2[i];
            v2[i] = v2[min];
            v2[min] = auxInt;
        }
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

void Graph::teste(ofstream &output_file) {
    // funcionando
    /*
    vector<vector<Node*>> vectorNode;
    vector<Node*> aa;
    aa.emplace_back(getNode(13));
    output_file << "Triste" << endl;
    vectorNode.emplace_back(aa);
    output_file << "Feliz" << endl;
    output_file << "Local 1: " << aa.at(0) << endl;
    output_file << "Local 2: " << getNode(13) << endl;
    output_file << "Local 3: " << vectorNode.at(0).at(0) << endl;
    */

    // testando
    vector<vector<Node*>> vectorNode;
    output_file << "vectorNode.size()" << vectorNode.size() << endl;
    for(int i=0;i<3;i++) {
        vectorNode.push_back(criaVectorTeste()); // criando os vetores de node;
        vectorNode.at(i).insert(vectorNode.at(i).end(), getNode(i));
        //cout << "Tamanho i: " << i << " " << vectorNode->end()->capacity() << endl;
    }
    output_file << "vectorNode.size()" << vectorNode.size() << endl;
    for(int i=0;i<vectorNode.size();i++) {
        output_file << "Id: " << vectorNode.at(i).at(0)->getId() << " posição na memória: " << vectorNode.at(i).at(0) << endl;
        output_file << "Id: " << getNode(vectorNode.at(i).at(0)->getId())->getId() << " posição na memória: " << getNode(vectorNode.at(i).at(0)->getId()) << endl;
    }
    vectorNode.at(0).at(0)->setCor(1);
    getNode(vectorNode.at(0).at(0)->getId())->getCor();
    output_file << "Node: " << vectorNode.at(0).at(0)->getId() << " vectorNode.at(0).at(0)->getCor: " << vectorNode.at(0).at(0)->getCor() << endl;
    output_file << "Node: " << getNode(vectorNode.at(0).at(0)->getId())->getId() << "getNode(vectorNode.at(i).at(0)->getId())->getCor(): " << getNode(vectorNode.at(0).at(0)->getId())->getCor() << endl;
    Node *node = new Node();
    node = *vectorNode.at(0).begin();
    output_file << "Node: " << vectorNode.at(0).at(0)->getId() << "Local de memoria: " << *vectorNode.at(0).begin() << endl;
    output_file << "Node: " << vectorNode.at(0).at(0)->getId() << "Local de memoria: " << vectorNode.at(0).at(0) << endl;
    output_file << "Node: " << node->getId() << "Local de memoria: " << node << endl;
    
    vector<Node*> aa;
    aa.emplace_back(getNode(13));
    vectorNode.at(0).emplace_back(aa.at(0)); 
    output_file << "vectorNode.at(0).at(1): " << vectorNode.at(0).at(1)->getId() << " Local de memoria: " << vectorNode.at(0).at(1) << endl;
    output_file << "aa.at(0)->getId(): " << aa.at(0)->getId() << " Local de memoria: " << aa.at(0) << endl;
    output_file << "getNode(aa.at(0))->getId()" << getNode(aa.at(0)->getId())->getId() << " Local de memoria: " << getNode(aa.at(0)->getId()) << endl ;  

    output_file << "Id: " << vectorNode.at(0).at(0)->getId() << " Local de memoria: " << vectorNode.at(0).at(0) << endl;

    vectorNode.at(0).erase(vectorNode.at(0).begin());
    for(int i=0;i<vectorNode.at(0).size();i++) {
        output_file << "Id: " << vectorNode.at(0).at(i)->getId() << " Local de memoria: " << vectorNode.at(0).at(i) << endl;
    }

    aa.emplace_back(vectorNode.at(0).at(0));

    output_file << "Id: " << aa.at(1)->getId() << " Local de memoria: " << aa.at(1) << endl;
    aa.erase(aa.begin());
    output_file << "Id: " << aa.at(0)->getId() << " Local de memoria: " << aa.at(0) << endl;
    output_file << "Id: " << getNode(aa.at(0)->getId())->getId() << " Local de memoria: " << getNode(aa.at(0)->getId()) << endl;
}

vector<Node*> Graph::criaVectorTeste() {
    //vector<Node> vector;
    vector<Node*> vectorNode;
    //vectorNode->reserve(this->order);
    return vectorNode;
}


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
    cout << "Valor do i denovo: " << i << endl;
    cout << "Valor do p denovo: " << p << endl;
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
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

// INICIO FECHO TRANSITIVO DIRETO ///////////////////////////////

void Graph::fechoTransitivoDireto(ofstream &output_file, int id)
{
    //cria um vetor que marca quais nodes ja foram analisados
    bool *visitados = new bool[this->order];
    //cria o vetor fecho transitivo direto
    bool *vet_ftd = new bool[this->order];
    //cria uma fila que diz quais vertices ainda precisam ser analisados
    queue<int> fila;
    //adiciona o vertice inicial na fila
    fila.push(id);
    for (int i = 0; i < this->order; i++)
    {
        visitados[i] = false;
        vet_ftd[i] = false;
    }

    //começa iteração (enquanto a fila não estiver vazia)
    while (!(fila.empty()))
    {
        //pega um node a ser analisado da fila
        int IdVet = getNode(fila.front())->getIdNode(); // já que os vetores começam da posição 0, isso equivale a passar a posição equivalente do id do vertice no vetor
        Node *V = getNode(fila.front());

        fila.pop();
        //verifica se o node a ser analisado ja foi analisado. (se ele ja foi acaba essa iteração)
        if (visitados[IdVet] == false)
        {
            //marca o node como visitado;
            visitados[IdVet] = true;
            //adiciona ele no vetor fecho transitivo direto
            vet_ftd[IdVet] = true;
            //adiciona todos os nodes adjacentes a ele na fila
            for (Edge *it = V->getFirstEdge(); it != NULL; it = it->getNextEdge())
            {
                int verticeAdj = it->getTargetId(); // aqui ele passa o id do node com o qual it(ou seja V) está ligado pela aresta e que tem como id o node alvo
                fila.push(verticeAdj);
            }
        }
    }
    
    //imprime o fecho transtivio direto do id
    output_file << "O fecho transitivo direto de " << id << " é: {"; 
    int cont = 0;
    for (int i = 0; i < this->order; i++)
    {
        if (vet_ftd[i] == true)
        {
            cont++;
        }
    }
    for (int i = 0; i < this->order; i++)
    {
        if (vet_ftd[i] == true)
        {
            if (cont - 1 > 0)
            {
                output_file << i + 1 << ", ";
                cont--;
            }
            else if (cont - 1 == 0)
            {
                output_file << i + 1;
            }
        }
    }
    output_file << "}" << endl;
}

// FIM FECHO TRANSITIVO DIRETO //////////////////////////////////


// INICIO FECHO TRANSITIVO INDIRETO

void Graph::fechoTransitivoIndireto(ofstream &output_file, int id)
{
    //cout << this->order;
    bool *fti = new bool[this->order];    // vetor para verificar o fecho transitivo indireto
    bool *node = new bool[this->order];   // vetor para verificar os vizinhos
    int *vetFti = new int[this->order];
    int *vetId = new int[this->order];
    int cont = 0;

    for (int i = 0; i < this->order; i++) // passando false para tudo antes de começaar
    {
        fti[i] = false;
        node[i] = false;
    }

    int conta = 0; // auxilia a descobrir qual o ultimo vertice para quando a gente for printar n�o colocar uma "," depois do ultimo

    for (Node *p = this->first_node; p != nullptr; p = p->getNextNode()) /// percorre todos os vertices
    {

        if (!node[p->getIdNode()]) // se a posição do vetor que equivale ao indice do vertice-1 já que a posição do vetor começa do 0, se ela for false o código ocorre, pois ainda não passamos por esse vertice
        {
            node[p->getIdNode()] = true;                            // passa true para a posição atual
            fti[p->getIdNode()] = deephFirstSearch(id, p->getId()); // chama a busca em profundidade passando o id que queremos e o id equivalente ao no que estamos no for
            if (fti[p->getIdNode()])                                  // se true, ou seja se é possivel desse node p chegar ao vertice "id" que é parametro da função então conta++ para auxiliar com a impressão, assim como está escrito ali em cima;
            {
                conta++;
                vetFti[cont] = p->getIdNode(); 
                vetId[cont] = p->getId();
                cont++;
            }
        }
    }
    output_file << "O fecho transitivo indireto de " << id << " é: ";
    output_file << "{ ";

    int aux = 0;
    for (int i = 0; i < cont; i++)
    {
        if (fti[vetFti[i]])
        {
            if (aux == conta - 1)
            {
                output_file << (getNode(vetId[i])->getId());
                aux++;
            }
            else
            {
                output_file << (getNode(vetId[i])->getId()) << ", ";
                aux++;
            }
        }
    }
    output_file << " }";
}

bool Graph::deephFirstSearch(int id, int start)
{
    
    //Criando vetor para verificar e tamb�m vetor predecessor de profundidade
    bool *node = new bool[this->order]; // vetor do tamanho do grafo
    int conta = 0;
    int idParametro; // equivale a posição do id do vertice no vetor;
    for (int i = 0; i < this->order; i++)
    {
        node[i] = false; // passa false para todas as posi��es
    }
    // cria vetor para auxiliar
    Node *p;

    //Para todo v em G;
    p = getNode(start);           // passa o primeiro v�rtice do grafo para o Node p;
    idParametro = p->getIdNode(); /// passa a posi��o equivalente do id de p em rela��o ao vetor
    //Se v n�o visitado ent�o

    if (id != p->getId()) // se o v�rtice que foi passado como parametro nessa fun��o que � chamada pela fecho transitivo indireta
    {                     // n�o for igual ao id, ele faz isso, caso contrario obviamente � pois j� estamos no v�rtice que queremos buscar
        auxDeepthFirstSearch(node, p); // passa o vetor e o node que estamos come�ando, pode ser 0,1,2 ... depende de onde o for
    }                                    // da fecho transitivo indireto est� chamando
    else
    {
        return true; // retorna true caso o v�rtice em que estamos � o vertice que queremos
    }

    //Se encontrou
    if (node[getNode(id)->getIdNode()]) // dps de passar pela aux ele verifica se foi mudado por parametro a posi��o equivalente
    {                                     // ao id que queremos no vetor, caso a gente queira o vertice 3 e passamos o vertice 8 que est� ligado
                                          // aos v�rtices 9 e 10, somente as posi��es 7,8,9 receberiam true, ou seja 8 n�o chega ao v�rtice 3, ou seja n�o entre nesse if
        delete[] node;
        return true;
    }
    delete[] node;
    return false;
}

void Graph::auxDeepthFirstSearch(bool node[], Node *v)
{

    int idParametro = v->getIdNode(); /// pega a posição equivalente do id desse node no vetor;

    Node *aux;

    //Marca v como visitado;
    node[idParametro] = true; /// marca o node que estamos como visitado;

    //Para todo node em Adj(v)
    for (Edge *p = v->getFirstEdge(); p != NULL; p = p->getNextEdge())
    {
        idParametro = p->getTargetIdNode(); /// pega a posição no vetor dos vizinhos que a gente quer verificar
        
        //Se o node !visitado então
        if (!node[idParametro])
        {
            
            aux = getNode(p->getTargetId());
            
            auxDeepthFirstSearch(node, aux);
        }
    }
    /*
        Explicando essa função acima: se por exemplo nós chamamos atraves da deepthFirstSearch o node 8
        que é ligado aos nodes 9 e 10, porém estamos buscando o node 3, ele retornara para a fecho
        transitivo indireto falso, pois na linha 355 nós passamos para todos os node falso, na chamada
        da função na qual o node 8 foi chamado, logo após nós chamamos essa função aq passando o vetor com
        tudo falso e o node 8, como ele está ligado ao 9 e 10, somente esses 3 irão receber true, oq acabara
        fazendo com que a posição 2 ou seja (3-1) que equivale a posição do vetor, continue false

    */
}

// FIM FECHO TRANSITIVO INDIRETO ////////////////////////////////

// INICIO DIJKSTRA /////////////////////////////////////////////

void Graph::caminhoMin_djkstra(ofstream &output_file, int orig, int dest) {

    if(!this->negative_edge)
    {
        if(this->weighted_edge)
        {
                if(this->directed)
                {
                    int *pa = new int[this->order];
                    int *dist = new int[this->order];
                    bool *mature = new bool[this->order];

                    for (int i=0; i<this->order; i++)
                        pa[i] = -1, mature[i] = false, dist[i] = INT_MAX;

                    int auxPa = getNode(orig)->getIdNode();
                    int auxDest = getNode(dest)->getIdNode();


                    pa[auxPa] = orig;
                    dist[auxPa] = 0;

                        while (true) 
                        {
                            int min = INT_MAX;
                            Node *y;

                            for (Node *z = this->first_node; z != nullptr; z = z->getNextNode()) 
                            {
                                if (mature[z->getIdNode()]) continue;
                                    if (dist[z->getIdNode()] < min) 
                                    {
                                        min = dist[z->getIdNode()];
                                        y = z;
                                    }                               
                            }

                            if (min == INT_MAX) break;
                            // atualização de dist[] e pa[]:
                            for (Edge *a = y->getFirstEdge(); a != nullptr; a = a->getNextEdge()) {
                                if (mature[a->getTargetIdNode()]) continue;
                                if (dist[y->getIdNode()] + a->getWeight() < dist[a->getTargetIdNode()]) {
                                    dist[a->getTargetIdNode()] = dist[y->getIdNode()] + a->getWeight();
                                    pa[a->getTargetIdNode()] = y->getId();
                                }
                            }
                            mature[y->getIdNode()] = true;
                        }

                        if(dist[auxDest] == INT_MAX )
                        output_file << " Nao existe caminho entre o vertice " << orig << " ao vertice " << dest << endl;
                        else  
                        output_file << " A distancia do vertice " << orig << " ao vertice " << dest << " sera: " << dist[auxDest] << endl;

                } else {
                    cout << "Chegou aqui";
                    int *pa = new int[this->order];
                    int *dist = new int[this->order];
                    bool *mature = new bool[this->order];

                    for (int i=0; i<this->order; i++)
                        pa[i] = -1, mature[i] = false, dist[i] = INT_MAX;

                    int auxPa = getNode(orig)->getIdNode();
                    int auxDest = getNode(dest)->getIdNode();


                    pa[auxPa] = orig;
                    dist[auxPa] = 0;

                        while (true) 
                        {
                            // escolha de y:
                            int min = INT_MAX;
                            Node *y;

                            for (Node *z = this->first_node; z != nullptr; z = z->getNextNode()) 
                            {
                                if (mature[z->getIdNode()]) continue;
                                    if (dist[z->getIdNode()] < min) 
                                    {
                                        min = dist[z->getIdNode()];
                                        y = z;
                                    }
                                    
                            }

                            if (min == INT_MAX) break;
                            // atualização de dist[] e pa[]:
                            for (Edge *a = y->getFirstEdge(); a != nullptr; a = a->getNextEdge()) {
                                if (mature[a->getTargetIdNode()]) continue;
                                if (dist[y->getIdNode()] + a->getWeight() < dist[a->getTargetIdNode()]) {
                                    dist[a->getTargetIdNode()] = dist[y->getIdNode()] + a->getWeight();
                                    pa[a->getTargetIdNode()] = y->getId();
                                }
                            }
                            mature[y->getIdNode()] = true;
                        }   
                        
                        int dist2 = -1;
                        dist2 = auxCaminhoMin_djkstra(dest,orig);
                        if(dist[auxDest] == INT_MAX )
                        {
                            output_file << " Nao existe caminho entre o vertice " << orig << " ao vertice " << dest << endl;
                        }
                        else if (dist2 == dist[auxDest] && dist2 != -1)
                        {  
                            if(dist2 == 0)
                            {
                                output_file << " A distancia do vertice " << orig << " ao vertice " << dest << " sera: " << dist[auxDest]<< endl;
                            }
                            else
                            {
                                output_file << " A distancia do vertice " << orig << " ao vertice " << dest << " sera: " << dist2 << endl;
                            }
                        } 
                        else if(dist2 == -1 )
                        {
                            output_file << " Nao existe caminho entre o vertice " << orig << " ao vertice " << dest << endl;
                        }                
                }
        }
        else
        {
            output_file << " A distancia do vertice " << orig << " ao vertice " << dest << " sera: " << 0 << endl;
        }
    } 
    else
    {
        output_file << " Não foi possível pois existem arestas com peso negativo" << endl;
    }

}


int Graph::auxCaminhoMin_djkstra(int orig, int dest)
{
    int *pa = new int[this->order];
    int *dist = new int[this->order];
    bool *mature = new bool[this->order];

        for (int i=0; i<this->order; i++)
            pa[i] = -1, mature[i] = false, dist[i] = INT_MAX;

        int auxPa = getNode(orig)->getIdNode();
        int auxDest = getNode(dest)->getIdNode();


        pa[auxPa] = orig;
        dist[auxPa] = 0;

            while (true) 
            {
                // escolha de y:
                int min = INT_MAX;
                Node *y;

                for (Node *z = this->first_node; z != nullptr; z = z->getNextNode()) 
                {
                    if (mature[z->getIdNode()]) continue;
                        if (dist[z->getIdNode()] < min) 
                        {
                            min = dist[z->getIdNode()];
                            y = z;
                        }
                        
                }

                if (min == INT_MAX) break;
                // atualização de dist[] e pa[]:
                for (Edge *a = y->getFirstEdge(); a != nullptr; a = a->getNextEdge()) {
                    if (mature[a->getTargetIdNode()]) continue;
                    if (dist[y->getIdNode()] + a->getWeight() < dist[a->getTargetIdNode()]) {
                        dist[a->getTargetIdNode()] = dist[y->getIdNode()] + a->getWeight();
                        pa[a->getTargetIdNode()] = y->getId();
                    }
                }
                mature[y->getIdNode()] = true;
            }

    if(!this->negative_edge)
    {
        if(dist[auxDest] != INT_MAX)
        {
          return dist[auxDest];
        } 
        else
        {
            return -1;
        }
    }
    else
    {
        if(dist[auxDest] != INT_MAX)
        {
          return dist[auxDest];
        } 
        else
        {
           exit(0);
        } 
    }
     
}

/////////// FIM DIJKSTRA /////////////////////////////////////////////


// INICIO FLOYD ////////////////////////////////////////////////////////////

void Graph::existeCaminho(bool *verifica,int idSource,int idTarget) {
    Node *node = getNode(idSource);
    for(Edge *aux = node->getFirstEdge(); aux != nullptr; aux = aux->getNextEdge()) {
        if(aux != nullptr) {
            if(aux->getTargetId() == idTarget) {
                *verifica = true;
            } else {
                existeCaminho(verifica, aux->getTargetId(), idTarget);
            }
        }
    }
}

void Graph::caminhoMin_floyd(ofstream &output_file, int idSource, int idTarget)
{
    if(this->directed)
    {
        Node *node = getNode(idSource);
        bool aux = false;
        bool *verifica;
        verifica = &aux;
        existeCaminho(verifica,idSource,idTarget);
        if(*verifica == true)
        {
            int ordem = this->order;               // recebe ordem do grafo
            int ** dist = new int *[ordem]; // inicializando matriz que recebe vetor
            dist = constroiMat_floyd(ordem, dist);             // dist recebe funcao floyd
            Node *node1 = getNode(idSource);
            Node *node2 = getNode(idTarget);
            output_file << "O menor caminho entre o No[" << idSource << "] e o No[" << idTarget << "] e: [" << dist[node1->getIdNode()][node2->getIdNode()] << "]" << endl;
        }
        else
        {
            output_file << " Nao existe caminho entre o No[" << idSource << "] e o No[" << idTarget << "]" << endl;
        }
    }
    else
    {
        output_file << " O grafo nao é direcionado " << endl;
    }
}

int **Graph::constroiMat_floyd(int ordem, int **dist)
{ // fucnao para utilizar lista de adj e para usar o alg de floyd

    dist = new int *[ordem];
    for (int i = 0; i < ordem; i++)
    {
        dist[i] = new int[this->order];
    }

    Node *auxA = this->first_node; // ponteiro do primeiro nó recebe primeiro no do grafo
    Node *auxB;                    // ponteiro auxiliar para um no
    int pesoEdge = 1;                  // peso da aresta
    // matriz com os valores de cada aresta entre os nos
    for (int i = 0; auxA != NULL; auxA = auxA->getNextNode(), i++)
    {
        auxB = this->first_node;

        for (int j = 0; auxB != NULL; auxB = auxB->getNextNode(), j++)
        {
            Edge *aux = auxA->hasEdgeBetween(auxB->getId());

            if (this->weighted_edge && aux != NULL)
                pesoEdge = aux->getWeight();

            if (auxA->getId() == auxB->getId())
                dist[i][j] = 0;

            else if (aux != NULL)

                dist[i][j] = pesoEdge;

            else
                dist[i][j] = INT_MAX / 2;
        }
    }
    for (int k = 0; k < ordem; k++)
    {
        // Escolhendo todos os vértices como fonte, um por um
        for (int i = 0; i < ordem; i++)
        {
            if (i != k)
            { // Escolhendo todos os vértices como destino
                for (int j = 0; j < ordem; j++)
                {
                    //Se o vértice c estiver no caminho mais curto de i para j, em seguida, atualize o valor de dist [i] [j]
                    if (dist[i][j] > dist[i][k] + dist[k][j] && dist[i][k] + dist[i][j] > 0)
                        dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }
    return dist;
}

// FIM FLOYD ////////////////////////////////////////////////////



Graph* Graph::getVertexInduced(int *listIdNodes, int tam)
{

    Graph *subGrafo = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);

    Node *nodeAux;

    for (int i = 0; i < tam; i++)
    {
        if (this->searchNode(listIdNodes[i]))
        {
            subGrafo->insertNode(listIdNodes[i],0);
        }
    }

    Node *node;
    Node *inicio;
    Node *inicio2;
    Edge *aux;
    Edge *aux2;
    bool verifica = false;
    bool verifica2 = false;

    //para todo noh do subgrafo,
    

    for (node = subGrafo->getFirstNode(); node != nullptr; node = node->getNextNode())
    {

             inicio = this->getNode(node->getId()); 

           for (aux = inicio->getFirstEdge(); aux != nullptr; aux = aux->getNextEdge())
           {

            // se a aresta do vertice pra onde ela aponta existir

                verifica = subGrafo->searchNode(aux->getTargetId());
                verifica2 = inicio->searchEdge(aux->getTargetId());

                if (verifica && verifica2)
                {
                    
                    // incluir a aresta no noh do subgrafo;
                    if(node->searchEdge(aux->getTargetId()) == false)
                    {
                            node->insertEdge(aux->getTargetId(), aux->getWeight(), aux->getTargetIdNode()); 
                            getNode(aux->getTargetId())->insertEdge(node->getId(), aux->getWeight(), node->getIdNode()); 
                            subGrafo->number_edges++;
                    }
                } 

           }
        //verificar as arestas no grafo original. 
    }

    return subGrafo;
}

// INICIO PRIM ////////////////////////////////////////////////////////////

Graph *Graph::arvGMin_Prim(ofstream &output_file)
{

    if(this->directed && !weighted_edge ) {
        return nullptr;
    }  

    int num, vert;
 
    cout << "Digite o numero de vértices de 1 a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> num;
    int *nodes = new int[num];

    for (int i = 0; i < num; i++)
    {
        cout << "Digite o vértice de numero " << i + 1 << ": " << endl;
        cin >> vert;
        nodes[i] = vert;
    }

    int *posicoes = new int[num];
    int *aux = new int[num];

    int cont = 0;

    Graph *grafoA;

    grafoA = this->getVertexInduced(nodes, num);
    // criando subGrafoVeti

   int *pa = new int[this->order];
   bool *tree = new bool[this->order];
   int *preco = new int[this->order];

   // inicialização:
   for (int i = 0; i < grafoA->order; i++)
   {
       pa[i] = -1;
       tree[i] = false;
       preco[i] = INT_MAX; 
   }   

    Node *auxN = grafoA->getFirstNode();
    tree[auxN->getIdNode()] = true;
    posicoes[auxN->getIdNode()] = auxN->getId();
    aux[cont] = auxN->getIdNode();
    pa[cont] = auxN->getId();

   for (Edge *auxE = auxN->getFirstEdge(); auxE != nullptr; auxE=auxE->getNextEdge())
   {
       cont++;
       pa[auxE->getTargetIdNode()] = auxN->getId();
       posicoes[auxE->getTargetIdNode()] = auxE->getTargetId();
       aux[cont] = auxE->getTargetIdNode(); 
       preco[auxE->getTargetIdNode()] = auxE->getWeight();
   }
 

   while (true)
    { 
      
      int min = INT_MAX;
      Node *y; 

       for (Node *w = grafoA->getFirstNode(); w != nullptr ; w = w->getNextNode())
       {

         if (!tree[w->getIdNode()] && preco[w->getIdNode()] < min)
         {
              min = preco[w->getIdNode()];
               y = w;
         } 
       }

      if (min == INT_MAX) break;
      // a aresta pa[y]-y é a mais barata da franja
      tree[y->getIdNode()] = true;
      // atualização dos preços e ganchos: 
      for (Edge *a = y->getFirstEdge(); a != nullptr; a = a->getNextEdge())
       {

         if (!tree[a->getTargetIdNode()] && a->getWeight() < preco[a->getTargetIdNode()]) 
         {
            cont++;
            preco[a->getTargetIdNode()] = a->getWeight();
            pa[a->getTargetIdNode()] = y->getId();
            posicoes[a->getTargetIdNode()] = a->getTargetId();
            aux[cont] = a->getTargetIdNode();
         }
      }
   }

    // montando subArv e printando

    Graph *arvPrim = new Graph(num, this->directed, this->weighted_edge, this->weighted_node);

     int aux2;
     for(int i=0;i<=cont;i++)
     {
        aux2 = aux[i];
        arvPrim->insertNode(posicoes[aux2],0); 
     }

     int auxP;
     int total = 0;

      for(int i=0;i<=cont;i++)
     {

         Node *e = getNode(pa[aux[i]]);
         
            for(Edge *x = e->getFirstEdge(); x != nullptr; x = x->getNextEdge())
            {
                if(x->getTargetId() == posicoes[aux[i]])
                {
                   auxP = x->getWeight();
                   total = total + auxP;
                }
            }

         if(pa[aux[i]] != posicoes[aux[i]])
         {
           arvPrim->insertEdge(pa[aux[i]],posicoes[aux[i]],auxP);
         }

     }

     total = total/2;

    output_file << " O peso total da Arvore Geradora Minina pelo algoritmo de Prim sera: " << endl;
    output_file << total << endl;
    output_file << "Arvore Geradora Minina pelo algoritmo de Prim: " << endl;

    return arvPrim;

}

// FIM PRIM ////////////////////////////////////////////////////

// INICIO KRUSKAL ////////////////////////////////////////////////////////////

Graph *Graph::arvGMin_Kruskal(ofstream &output_file)
{
    if(this->directed && !weighted_edge) {
        return nullptr;
    }

       
    int num, v;
    cout << "Digite o numero de vértices de '1' a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> num;
    int *nodes = new int[num];
    for (int i = 0; i < num; i++)
    {
        nodes[i] = -1;
    }
    for (int i = 0; i < num; i++)
    {
        cout << "Digite o vértice numero " << i + 1 << ": " << endl;
        cin >> v;
        nodes[i] = v;
    }
    //pre-requisitos pra fazer a ordenacao

    Graph *grafoA;
    grafoA = this->getVertexInduced(nodes, num);

    Graph *grafoB = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node); //vai vira o grafoVI

    int *EdgeNode = new int[3];
    int totalEdge = 0;

     totalEdge = grafoA->getNumberEdges();

    list<pair<int, int>> listP;

    Node *sup;
    Node *p;
    Edge *aux;

    for (sup = grafoA->getFirstNode(); sup != nullptr; sup = sup->getNextNode())
    {
        grafoB->insertNode(sup->getId(),sup->getWeight());
    }
    
        
    //Criar uma lista L com as arestas ordenadas em
    //ordem crescente de pesos.
    for (int i = 0; i < totalEdge; i++)
    {
        // acha a aresta de menor peso
        grafoA->getWeithlessEdge(EdgeNode);
        //insere a aresta de menor peso
        listP.push_back(make_pair(EdgeNode[0], EdgeNode[1]));
        //retira a aresta do grafo pra evitar repetir a mesma aresta;
        sup = grafoA->getNode(EdgeNode[0]);
        p = grafoA->getNode(EdgeNode[1]);
        if (!this->directed)
        {
            sup->removeEdge(p->getId(), this->directed, p);
            p->removeEdge(sup->getId(), this->directed, sup);
            sup->removeEdge(p->getId(), this->directed, p);
            p->removeEdge(sup->getId(), this->directed, sup);
        }
        else
        {
            sup->removeEdge(p->getId(), this->directed, p);
        }
        //adiciona a a resta num grafo auxiliar.
        grafoB->insertEdge(EdgeNode[0], EdgeNode[1], EdgeNode[2]);
    }
   
      
    //Organizar a lista;

    //Criar |V| subárvores contendo cada uma um nó
    //isolado.
    Graph *agMin = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);
    for (sup = grafoB->getFirstNode(); sup != nullptr; sup = sup->getNextNode())
    {
        agMin->insertNode(sup->getId(), sup->getWeight());
    }

    //Cria lista vazia
    list<pair<int, int>> listaAux;

    //contador ¬ 0
    int cont = 0;
    int numMaxAresta = agMin->getOrder() - 1;
    bool *verificado = new bool[this->order];

    for (int i = 0; i < this->order; i++)
    {
        verificado[i] = false;
    }
      
    while (cont < numMaxAresta && !listP.empty())
    {
        pair<int, int> dist_no = listP.front(); //copia par (id do vertice e distancia) do topo
        int v1 = dist_no.first;
        int v2 = dist_no.second;

        listP.pop_front();
        //Se v1 e v2 não estão na mesma subárvore então
        if (!verificaSubarvore(v1, v2, agMin))
        {
            //preenche a lista;
            listaAux.push_back(make_pair(v1, v2));
            //busca o peso da aresta
            int peso = getWeightFromEdgeNodeCombo(v1, v2, grafoB);

            //Unir as subárvores que contêm v1 e v2.
            agMin->insertEdge(v1, v2, peso);
            //contador ¬ contador + 1
            cont++;
        }
    }
   
    
    int pesoT = 0;
    while (!listaAux.empty())
    {
        pair<int, int> dist_no = listaAux.front(); //copia par (id do vertice e distancia) do topo
        int v1 = dist_no.first;
        int v2 = dist_no.second;
        pesoT = pesoT + getWeightFromEdgeNodeCombo(v1, v2, agMin);
        listaAux.pop_front();
    }
    output_file << "Peso da Arvore Geradora Minima: " << pesoT << endl;


    return agMin;

}

// FIM KRUSKAL ////////////////////////////////////////////////////


// INICIO BUSCA EM LARGURA ////////////////////////////////////////////////////////////

void Graph::arv_Buscalargura(ofstream &output_file, int id)
{

  int *num = new int[this->order];
  int *pa = new int[this->order];
  int *vetAd = new int[this->order];
  int *posicoes= new int[this->order];
  bool entrou = false;
  int cont = 0;
  int cont2 = 0;


    for (int i = 0; i < this->order; i++)
        num[i] = pa[i] = -1;

        
    list<Node*> listN;  
    Node *node1 = getNode(id);
    num[node1->getIdNode()] = cont++; 
    pa[node1->getIdNode()] = id;
    listN.push_back(getNode(id));
    vetAd[cont2] = id;
    posicoes[0] = id;

    while (!listN.empty()) 
    {
        entrou = false;
        Node *aux = listN.front();

        listN.pop_front(); 

        for (Edge *auxE = aux->getFirstEdge(); auxE!=NULL; auxE=auxE->getNextEdge())
        {
             if (num[auxE->getTargetIdNode()] == -1) {
                 posicoes[cont] = aux->getId();
                num[auxE->getTargetIdNode()] = cont; 
                cont++;
                pa[auxE->getTargetIdNode()] = aux->getId();
                listN.push_back(getNode(auxE->getTargetId()));
                cont2++;
                vetAd[cont2] = auxE->getTargetId();
            }

        }
     
    }

      Graph *arvBL = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);

     for(int i=0;i<=cont2;i++)
     {
        arvBL->insertNode(vetAd[i],0); 
     }

    int auxP;
    int auxId;

     for(int i=1;i<=cont2;i++)
     {
            Node *e = getNode(posicoes[i]);
            for(Edge *x = e->getFirstEdge(); x != nullptr; x = x->getNextEdge())
            {
                if(x->getTargetId() == vetAd[i])
                {
                   auxP = x->getWeight();
                }
            }
            arvBL->insertEdge(posicoes[i],vetAd[i],auxP);

     }

    output_file << "Arvore dada pelo caminhamento em lagura: ";

    arvBL->printGraph(output_file);
 
}


// FIM BUSCA EM LARGURA ////////////////////////////////////////////////////

void Graph::getWeithlessEdge(int *nohAresta)
{

    Node *p = this->first_node;
    Edge *aux = p->getFirstEdge();
    int menor = 9999999;
    while (p != NULL)
    {

        aux = p->getFirstEdge();
        while (aux != NULL)
        {
            if (aux->getWeight() < menor)
            {
                nohAresta[0] = p->getId();
                nohAresta[1] = aux->getTargetId();
                nohAresta[2] = aux->getWeight();
                menor = aux->getWeight();
            }
            aux = aux->getNextEdge();
        }
        p = p->getNextNode();
    }
}

// usa os mecanismos da busca em profundidade para indicar a alcansabilidade de um vertice a outro
bool Graph::verificaSubarvore(int v1, int v2, Graph *subGrafo)
{
    //vetor de alcansabilidade -- se tem caminho ou nao
    bool *fti = new bool[this->order];

    for (int i = 0; i < this->order; i++)
    {
        fti[i] = false;
    }
    // verifica para todos os nohs se tem caminho ou nao
    for (Node *p = subGrafo->getFirstNode(); p != NULL; p = p->getNextNode())
    {

        fti[p->getId() - 1] = subGrafo->deephFirstSearch(v2, p->getId());
    }

    return fti[v1 - 1];
}

//pega o peso da aresta atravez do int idNoh, int idAresta, Graph *subGrafo
int Graph::getWeightFromEdgeNodeCombo(int idNoh, int idAresta, Graph *subGrafo)
{
    Node *p = subGrafo->getNode(idNoh);
    Edge *aux;
    for (aux = p->getFirstEdge(); aux != NULL; aux = aux->getNextEdge())
    {
        if (aux->getTargetId() == idAresta)
        {
            break;
        }
    }
    return aux->getWeight();
}


void Graph::printGraph(ofstream &output_file)
{
    Node *p = this->first_node;
    Edge *aux = p->getFirstEdge();
    if (!directed)
    {
        output_file << "strict graph{"<<endl;
        while (p != NULL)
        {

            aux = p->getFirstEdge();
            while (aux != NULL)
            {

                output_file << p->getId() << " -- " << aux->getTargetId() <<" PESO : " << aux->getWeight() << endl;
                aux = aux->getNextEdge();
            }
            p = p->getNextNode();
        }
        output_file <<"}"<<endl;
    }
    else
    {
        output_file << "digraph{"<<endl;
        while (p != NULL)
        {

            aux = p->getFirstEdge();
            while (aux != NULL)
            {

                output_file << p->getId() << " -> " << aux->getTargetId() <<" PESO : " << aux->getWeight() << endl;
                aux = aux->getNextEdge();
            }
            p = p->getNextNode();
        }
        output_file <<"}"<<endl;
    }
        output_file << endl;
    output_file << endl;
}

/////////// INICIO DA ORDENAÇÃO TOPOLOGICA///////////////////////////

void Graph::ord_Topologica(ofstream &output_file)
{
    list<Node*> listN; // lista de nodes
    list<int> listTop; // lista topologica
    if (this->graphTemCiclo())// verifica se o grafo é aciclico ou não
    {
        output_file <<" Se o Grafo possui ciclos, logo, nao possui ordenação topologica"<<endl;
    }
    else{ // adaptando algoritimo kahn's
            Node *auxN;
            Edge *auxE;
            //procurando nos com enttrada =0
            for (auxN=this->first_node;auxN!=NULL;auxN = auxN->getNextNode())
            {   if (auxN->getInDegree()==0)// se entrada  = 0
                {
                    listN.push_back(auxN); //coloca os nos corretos na fila
                }
            }
            while (!listN.empty())// enquanto lista e vazia
            {
                Node *aux = listN.front();
                listN.pop_front(); //remove da lista
                listTop.push_back(aux->getId()); //coloca na lista auxiliar
                for(auxE =aux->getFirstEdge(); auxE!=NULL;auxE=auxE->getNextEdge())
                {
                    auxN = this->getNode(auxE->getTargetId()); //pega o no vizinho
                    auxN->decrementInDegree(); //decrementa o grau de entrada
                    if (auxN->getInDegree()==0) //se a entrada = 0
                    {
                        listN.push_back(auxN);
                    }

                }
            }
            //imprimindo ordenaçao a classificação topologica
            output_file << "Ordenação Topologica :" << endl;
            for(list<int>::iterator k = listTop.begin(); k != listTop.end(); k++)
            {
                    if(listTop.size() == this->getOrder())
                    output_file << (*k) << endl;
            }

        }
}

bool Graph::graphTemCiclo()
{
    list<int> auxCiclo;
    // Alocando os ints em uma lista
    for (int i = 0; i < this->order; i++)
    {
        auxCiclo.push_back(i);
    }
    auxCiclo.sort();

    for (list<int>::iterator i = auxCiclo.begin(); i !=  auxCiclo.end();){
     int anterior = *i;
        i++;
        // Se houver componentes iguais, o gráfo é cíclico,
         // entao o grafo tem um circuito
        if (anterior == *i)
            return true;
    }
        // Se  forem diferentes entre eles, o grafo nao tem circuito
    return false;
}

/// FIM DA ORDENAÇÃO TOPOLOGICA ///////////////////////////

void Graph::Guloso(ofstream &output_file, int p)
{
    bool *visitado = new bool[this->order];  // vetor para verificar os vértices já utilizados

    /*for (Node *node = this->first_node; node != nullptr; node = node->getNextNode())
    {
        output_file << "Node id: " << node->getId() << " entrada: " << node->getInDegree() << endl;
    }*/
    
    for(Node *i = this->first_node; i != nullptr; i = i->getNextNode())
    {
        output_file << " Id : " << i->getId() << " IdNode : " << i->getIdNode() << endl;

    }

    for(int i=0;i<this->order;i++)
    {
        visitado[i] = false; // marcando todos nodes como não visitados
    }

	if(this->weighted_node) // só pode grafo com node com peso
    {
        //vector<vector<Node>> vectorNode;
        vector<vector<Node>> *vectorNode = new vector<vector<Node>>; //Note space between "> >" // vetor de vetores de node
        vectorNode->reserve(this->order);
    
        for(int i=0;i<p;i++) {
            vectorNode->push_back(*criaVector()); // criando os vetores de node;
            vectorNode->at(i).reserve(this->order);
            //cout << "Tamanho i: " << i << " " << vectorNode->end()->capacity() << endl;
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
        /*getNode(0)->setCor(0);
        getNode(5)->setCor(1);*/


       /* output_file << " -> Primeira cor do 5 :  " << vectorNode->at(1).begin()->getCor() <<  " Id : " << vectorNode->at(1).begin()->getId() << endl;
        vectorNode->at(1).begin()->setCor(1);
        output_file << " -> Segunda cor do 5 :  " << vectorNode->at(1).begin()->getCor() <<  " Id : " << vectorNode->at(1).begin()->getId() << endl;*/


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

        /*for(int i=0;i<vectorWeightEdge->size();i++) {
            output_file << "Ids restante: " << vectorWeightEdge->at(i).getId() << endl;            
        }*/

        vector<vector<int>> *listMaiorMenorPeso = new vector<vector<int>>;
        listMaiorMenorPeso->reserve(p);
        //output_file << listMaiorMenorPeso.at(0).capacity() << endl;
        //output_file <<  listMaiorMenorPeso.capacity() << endl;

        for(int i=0;i<p;i++) {
            listMaiorMenorPeso->emplace_back(*criaVetorMaiorMenor());
            listMaiorMenorPeso->at(i).reserve(2);
        }
        //output_file <<  "Chegou" << endl;
        do {
            cout << " ENTROUU 11" << endl;
            for(int i=0;i<p;i++) {
                float menorVal;   
                int contPosicao = 0;
                float gap = 0;
                float maiorValor = vectorNode->at(i).at(0).getWeight();
                float menorValor = vectorNode->at(i).at(0).getWeight();
                //getMaiorMenorVal(&maiorValor, &menorValor, vectorNode->at(i), i, p);
                output_file << "Maior valor: " << maiorValor << " Menor valor: " << menorValor << endl;
                for(int j=0;j<vectorNode->at(i).size();j++) {
                    if(maiorValor < vectorNode->at(i).at(j).getWeight()) {
                        maiorValor = vectorNode->at(i).at(j).getWeight();
                    } else if(menorValor > vectorNode->at(i).at(j).getWeight()) {
                        menorValor = vectorNode->at(i).at(j).getWeight();
                    }
                } // possivelmente isso vai sair daqui
                listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).begin(), maiorValor);
                listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).end(), menorValor);

                gap = maiorValor - menorValor;
                //listMaiorMenorPeso->at(i).insert(listMaiorMenorPeso->at(i).begin(), 18);
                //listMaiorMenorPeso->at(i).at(1) = 17;
                for(int j=0;j<vectorWeightEdge->size();j++) {
                    
                    float gapNode;
                    float gapFinal;
                    output_file << " Vertice atual: " << vectorWeightEdge->at(j).getId() << endl;
                    output_file << "Peso atual: " << vectorWeightEdge->at(j).getWeight() << endl;
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
                    output_file << "Valor do gap atual: " << gap << endl;
                    output_file << "Valor do gapNode: " << gapNode << endl;
                    output_file << "Valos do gapFinal: " << gapFinal << endl;
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
                cout << " SAIU 1 " << endl;
                // output_file << "Tamanho da lista i = " << i << " e:" << listRank->at(i).size() << endl;
                for (int j = 0; j < listRank->at(i).size(); j++)
                {
                    output_file << "Gap do node " << vectorWeightEdge->at(j).getId() << " e: " << listRank->at(i).at(j) << endl;
                }
                output_file << "Valor do cont: " << contPosicao << endl;
               // cout << " ANTES " << endl;
               // cout << " contPosicao " <<  contPosicao << endl;
               // cout << " vectorWeightEdge->capacity() " << vectorWeightEdge->capacity() << endl;

                if((vectorWeightEdge->size() > 0) && !visitado[vectorWeightEdge->at(contPosicao).getIdNode()])
                {
                    vectorWeightEdge->at(contPosicao).setCor(i);
                    output_file << "SET COR !  Vertice : " << vectorWeightEdge->at(contPosicao).getId() << " Cor : " << i << endl;
                    visitado[vectorWeightEdge->at(contPosicao).getIdNode()] = true;
                // cout << " DEPOIS " << endl;
                // cout << " vectorWeightEdge->at(contPosicao).getWeight() " << vectorWeightEdge->at(contPosicao).getWeight() << endl;
                    if(vectorWeightEdge->at(contPosicao).getWeight() > listMaiorMenorPeso->at(i).at(0)) {
                        cout << " ENTROU NO 1 if " << endl;
                        listMaiorMenorPeso->at(i).at(0) = vectorWeightEdge->at(contPosicao).getWeight();
                    } else if(vectorWeightEdge->at(contPosicao).getWeight() < listMaiorMenorPeso->at(i).at(1)) {
                        cout << " ENTROU NO 2 if " << endl;
                        listMaiorMenorPeso->at(i).at(1) = vectorWeightEdge->at(contPosicao).getWeight();
                    }
                    cout << " saiu if " << endl;
                    vectorNode->at(i).emplace_back(vectorWeightEdge->at(contPosicao));  
                    cout << " COMECOU 2 " << endl;
                    for(Edge *edge = vectorWeightEdge->at(contPosicao).getFirstEdge();edge != nullptr;edge = edge->getNextEdge()) {
                        output_file << "Id da aresta: " << edge->getTargetId() << endl;
                        if((getNode(edge->getTargetId())->getInDegree() == 1) && !visitado[getNode(edge->getTargetId())->getIdNode()] ) {
                            output_file << "Chegou 1.5.5" << endl;
                            getNode(edge->getTargetId())->setCor(i);
                            output_file << "SET COR !! Vertice : " << getNode(edge->getTargetId())->getId() << " Cor : " << i << endl;
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
                    output_file << " POSICAONODE : " << posicaoNode << endl;
                    cout << " SAIU 2 " << endl;
                    output_file << " Visitado  " << boolalpha << visitado[posicaoNode] << endl;
                    output_file << " Vertice : " << vectorWeightEdge->at(contPosicao).getId()  << " .getInDegree()" << vectorWeightEdge->at(contPosicao).getInDegree() << endl;
                    output_file << " PRINTADO O VERTICE QUE GRAU 1 : " << getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getId() << endl;
                    if((vectorWeightEdge->at(contPosicao).getInDegree() == 1) && !visitado[getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getIdNode()]) {
                        getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->setCor(i);
                        output_file << " SET COR !!! Vertice : " << getNode(vectorWeightEdge->at(contPosicao).getFirstEdge()->getTargetId())->getId() << " Cor : " << i << endl;
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
                    output_file << "Começo do vectorWeightEdge: " << vectorWeightEdge->begin()->getId() << endl;
                    vector<Node>::iterator n;
                    
                    n = vectorWeightEdge->begin(); 

                    advance(n, contPosicao);
                    
                    // output_file << "Valor do cont: " << contPosicao << endl;
                    //output_file << "Tamanho antes de excluir: " << vectorWeightEdge->size() << endl;
                    vectorWeightEdge->erase(n);
                    //vectorWeightEdge->resize(vectorWeightEdge->size()-1);
                /* for(int i =0;i<vectorWeightEdge->size();i++) {
                        output_file << "Elementos apos o resize: " << vectorWeightEdge->at(i).getId() << endl;
                    }*/               
                    //vectorWeightEdge->erase(vectorWeightEdge->begin() + contPosicao);

                    /*for(int l=0;l<vectorWeightEdge->size();l++) {
                        output_file << "Testando após excluir: " << vectorWeightEdge->at(l).getId() << endl;
                    }*/

                    //output_file << "Verificando o espaço dela antes de apagar para ter certeza: " << listRank->at(i).capacity() << endl;
                    listRank->at(i).clear();
                    listRank->reserve(listRank->capacity()-1);
                    //output_file << "Verificando se ela ainda continua tendo o mesmo espaço: " << listRank->at(i).capacity() << endl;
                    //vectorWeightEdge->erase(vectorWeightEdge->begin() + j);
                    for(int k=0;k<vectorNode->at(i).size();k++) {
                        output_file <<"i :"<< i << "VectorNode: " << vectorNode->at(i).at(k).getId() << endl;
                    }
                    /*for(Node *node = this->first_node;node != nullptr; node = node->getNextNode()) {
                        output_file << node->getId() << endl;
                    }*/
                }
            }

        } while(!vectorWeightEdge->empty());
       // output_file << " vectorNode->size()  : " << vectorNode->size() << endl;

        for(int a=0;a<vectorNode->at(0).size();a++)
        {
            output_file << " vectorNode.at(0). at (" << a << ") = " << vectorNode->at(0).at(a).getId() << endl;
        }

        output_file << endl;

        for(int a=0;a<vectorNode->at(1).size();a++)
        {
            output_file << " vectorNode.at(1). at (" << a << ") = " << vectorNode->at(1).at(a).getId() << endl;
        }


        for(int x =0; x < vectorNode->at(1).size() ; x++)
        {
            if(vectorNode->at(1).at(x).getId() == 5)
            {
                output_file << "  ->>>>>>>>> vectorNode->at(i).at(x).getId() cor " << vectorNode->at(1).at(x).getCor() << endl;
            }
        }  

        for(int w =0;w < vectorNode->size();w++)
        {
            for(int z =0; z < vectorNode->at(w).size();z++)
            {
                output_file << " i  = " << w << " Id " << vectorNode->at(w).at(z).getId() << " Cor : " <<  vectorNode->at(w).at(z).getCor() << endl;
            }

        }

        for(int i =0;i<p;i++)
        {
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
            }
            output_file << "  i : " << i << endl;
            vetorClusterNodes->at(0).insert(vetorClusterNodes->at(0).begin(),vectorNode->at(i).at(0));      
            output_file << "vetorClusterNodes->at(0).at(0) :  " << vetorClusterNodes->at(0).at(0).getId() << endl;
            //output_file << "vetorClusterNodes->at(0).at(0) :" << vetorClusterNodes->at(0).at(0).getId() << endl;
            cout<< "vectorNode->at(i).begin() " << vectorNode->at(i).at(0).getId() << endl;
            cout << " CAPACIDADE 1: " << vetorClusterNodes->capacity() << endl;
            cout << " CAPACIDADE 2: " << vetorClusterNodes->at(i).capacity() << endl;
            vectorNode->at(i).erase(vectorNode->at(i).begin());
            int contSameCluster;
            int contSubCluster = 1;
            //while(!vectorNode->at(i).empty()) {

            output_file << " COR DO 5 ********* " << getNode(5)->getCor() << endl; 
           
            output_file << " COR DO 0 ********* " << getNode(0)->getCor() << endl;  
            for(int k=0;k<contClusterAux;k++) 
            {
                contSameCluster = 0;
                maiorMenorValSubCluster->at(k).front() = vetorClusterNodes->at(k).at(0).getWeight();
                maiorMenorValSubCluster->at(k).back() = vetorClusterNodes->at(k).at(0).getWeight();

                int maior = vetorClusterNodes->at(k).at(0).getWeight();
                int menor = vetorClusterNodes->at(k).at(0).getWeight();
                        
                for(int j=0;j<vetorClusterNodes->at(k).size();j++) 
                {
                    Node *node = &vetorClusterNodes->at(k).at(j);
                    output_file << "  vetorClusterNodes->at(k).at(j).getId() :  " << vetorClusterNodes->at(k).at(j).getId() << endl;
                    verificados[node->getIdNode()] = true;                     
                    int tam = node->getTotal_Edge();
                    int *vizinhos = new int[tam];
                    int contAuxVizinhos = 0;
                    bool inseriu = false;
                    for(Edge *edge = node->getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) {
                        if((getNode(edge->getTargetId())->getCor() == node->getCor()) && !verificados[edge->getTargetIdNode()]) {
                            output_file << " node :   " << node->getId() << " COR : " << node->getCor() << endl;
                            output_file << " vizinho  : " << getNode(edge->getTargetId())->getId() << "COR : " << getNode(edge->getTargetId())->getCor() << endl;
                                
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
                            cout << " ENTROU 111111111111111 " << endl;
                            cout << " TAMANHAO K "<< vetorClusterNodes->size() << endl;
                            vetorClusterNodes->at(k+1).emplace_back(*vectorNode->at(i).begin());
                            vectorNode->at(i).erase(vectorNode->at(i).begin());
                        }
                    }

                    /* if(vetorClusterNodes->at(k).at(j).getWeight() > listMaiorMenorPeso->at(k).front()) {
                        maiorMenorValSubCluster->at(i).front() = vetorClusterNodes->at(k).at(j).getWeight();
                    } else if(vetorClusterNodes->at(k).at(j).getWeight() < listMaiorMenorPeso->at(k).back()) {
                        maiorMenorValSubCluster->at(i).back() = vetorClusterNodes->at(k).at(j).getWeight();
                    }
                    */
                        if(inseriu)
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
                    
                maiorMenorValSubCluster->at(k).front() = maior;
                maiorMenorValSubCluster->at(k).back() = menor;



                for(int d=0; d < vetorClusterNodes->at(k).size();d++)
                {
                    output_file << " AAAA vetorClusterNodes->at(k).at(j).getId() : " << vetorClusterNodes->at(k).at(d).getId() << endl;
                }

                for(int e = 0; e < vectorNode->at(i).size() ; e++)
                {
                    output_file << " vectorNode->at(" << e << ") = " << vectorNode->at(i).at(e).getId() << endl; 
                }

                /*if(!vectorNode->at(i).empty()) {
                    contClusterAux++;
                }*/

                if(!vectorNode->at(i).empty()) {
                    contClusterAux++;
                }

                cout << "i "<< i << "vetorClusterNodes.size() " << vetorClusterNodes->size() << endl; 
            } 
            //} 
    
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
                        gapFinalSubCluster = maiorMenorValSubCluster->at(e).front() - maiorMenorValSubCluster->at(e).back();
                        posicaoMaiorSubCluster = e;
                    } else { // buscando o maior subcluster com o menor gap
                        if(gapFinalSubCluster > (maiorMenorValSubCluster->at(e).front() - maiorMenorValSubCluster->at(e).back())) {
                            gapFinalSubCluster = maiorMenorValSubCluster->at(e).front() - maiorMenorValSubCluster->at(e).back();
                            posicaoMaiorSubCluster = e;
                        }
                    }
                }
            }

            vector<vector<int>> *arestas = new vector<vector<int>>;
            arestas->reserve(vetorClusterNodes->size());
            for(int e=0;e<vetorClusterNodes->size();e++) { 
                arestas->push_back(*criaVetorMaiorMenor());
                arestas->at(e).reserve(vetorClusterNodes->at(e).size());
            }
            vector<bool> *corNode = new vector<bool>;
            corNode->reserve(p);
            for(int n = 0;n<p;n++) {
                corNode->at(n) = true;
            }
            corNode->at(i) = false; // marcando a cor atual como false, pois não queremos ligar um subcluster em outro de mesma cor dele

            vector<vector<int>> *coresPossiveis = new vector<vector<int>>;
            //vector<Node> *vectorWeightEdge = new vector<Node>();
            coresPossiveis->reserve(this->order);
            for(int x=0;i<x;x++) {
                coresPossiveis->push_back(*criaVetorMaiorMenor());
                coresPossiveis->at(x).reserve(2); // 2 posições, 1° com a cor e a 2° com o gap
            }
            vector<vector<int>> *valores = new vector<vector<int>>;

            for(int e=0;e<vetorClusterNodes->size();e++) { // e < que o total de subclusters no i(cor atual)
                if(e != posicaoMaiorSubCluster) { // e sendo diferente do subcluster que a gente quer manter(no caso a posição dele no vetorClusterNodes->at(e))
                    for(int z = 0;z < vetorClusterNodes->at(e).size();z++) { // z < que a quantidade de nodes presentes em cada subcluster
                        for(Edge *edge = vetorClusterNodes->at(e).at(z).getFirstEdge();edge!=nullptr;edge = edge->getNextEdge()) { // verificando as arestas de cada subvertice
                            if(corNode->at(getNode(edge->getTargetId())->getCor())) { // se a cor do node estiver como true, ou seja não foi verificada ainda e nem é a cor do i
                                corNode->at(getNode(edge->getTargetId())->getCor()) = false; // marca a cor como visitada
                                listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()); //ela salva o maior e menor peso de cada i ou seja, de cada cluster
                                if(maiorMenorValSubCluster->at(e).front() > listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).front() && maiorMenorValSubCluster->at(e).back() < listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).back()) {
                                    coresPossiveis->at(z).push_back(maiorMenorValSubCluster->at(e).front() - maiorMenorValSubCluster->at(e).back()); // salvando o gap pra cor atual
                                    coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor()); // salvando a cor do vizinho
                                } else if(maiorMenorValSubCluster->at(e).front() > listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).front()) {
                                    coresPossiveis->at(z).push_back(maiorMenorValSubCluster->at(e).front() - listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).front()); // salvando o gap pra cor atual
                                    coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor());  // salvando a cor do vizinho
                                } else if(maiorMenorValSubCluster->at(e).back() < listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).back()) {
                                    coresPossiveis->at(z).push_back(listMaiorMenorPeso->at(getNode(edge->getTargetId())->getCor()).front() - maiorMenorValSubCluster->at(e).back()); // salvando o gap pra cor atual
                                    coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor()); // salvando a cor do vizinho
                                } else {
                                    coresPossiveis->at(z).push_back(0); // nesse caso o maior e menor valor do subcluster não impactam no gap do cluster
                                    coresPossiveis->at(z).push_back(getNode(edge->getTargetId())->getCor()); // salvando a cor do node
                                }
                            }
                        }    
                    }

                    //int menor = coresPossiveis->at(z).front();
                    int menor = coresPossiveis->at(0).front(); // pegando o primeiro gap
                    int corSelecionado = coresPossiveis->at(0).back(); // pegando a primeira cor selecionada
                    int contPosicaoSubCluster = 0;
                    for(int z = 0;z < coresPossiveis->size();z++) { // z < que o número de vértices presentes em cada subcluster
                    //if(menor > coresPossiveis->at(z).size()) { // 
                        if(menor > coresPossiveis->at(z).front()) {// pegando o menor valor de gap presente
                            menor = coresPossiveis->at(z).front(); // salvando esse valor de gap como o menor
                            corSelecionado = coresPossiveis->at(z).back(); // salvando a cor desse gap
                            contPosicaoSubCluster = z; // salvando essa posição escolhida com o menor gap
                        }
                    }

                    for(int z=0;z<vetorClusterNodes->at(e).size();z++) {
                        getNode(vetorClusterNodes->at(e).at(z).getId())->setCor(corSelecionado);
                    }

                    for(int z=0;z<coresPossiveis->size();z++) {
                        coresPossiveis->at(z).clear();
                    }

                    //if(coresPossiveis->at(contPosicaoSubCluster).back() != 0) {
                    if(menor != 0) {
                        //listMaiorMenorPeso->at(corSelecionado) = menor;//coresPossiveis->at(contPosicaoSubCluster);
                        listMaiorMenorPeso->insert(listMaiorMenorPeso->begin() + corSelecionado, { menor });
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

            if(i == 0)
            {
                output_file << "vetorClusterNodes.size()  " << vetorClusterNodes->size()  << endl;
                output_file << "vetorClusterNodes->at(0).size()  " << vetorClusterNodes->at(0).size()  << endl;
                for(int n=0;n<vetorClusterNodes->at(0).size();n++)
                {      
                    output_file << " Kluster : " << i << " Valor : " << vetorClusterNodes->at(0).at(n).getId() << endl;
                }
            }
        }

        
       

    } else {
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
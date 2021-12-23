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
    /*for(int i=0;i<order;i++) {
        insertNode(i);
    }*/
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
void Graph::insertNode(int id, int weight)
{
    Node *node = new Node(id, order);
    ///node->setNumber(order + 1);
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
        /*output_file << node->getId();
        output_file << endl;*/
    }

}

void Graph::insertEdge(int id, int target_id, float weight)
{
    //cout << "peso: " << weight << endl;
    /*if (!searchNode(id)) 
    {
       insertNode(id); 
    }

    if (!searchNode(target_id)) 
    {
       insertNode(target_id); 
    }*/
 

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
               // getNode(target_id)->insertEdge(node->getId(), weight, node->getIdNode());
            }
        }
        else
        {
            Node *node = getNode(id);             // search the actual node that's being called;
            if (!node->hasEdgeBetween(target_id)) // return if theres is no edge between the node id and the node target_id
            {
                Node *aux = getNode(target_id);
                node->insertEdge(target_id, weight, aux->getIdNode()); // inserts the edge between the two nodes
                //Node *aux = getNode(target_id);
                aux->insertEdge(node->getId(), weight, node->getIdNode()); // inserts the edge between the two nodes;
                //aux->insertEdge(node->getId(), node->getWeight(), node->getIdNode()); // inserts the edge between the two nodes;
               


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
        //Aux-BuscaEmProfundida(G,v);
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
        //cout<< getNodeId(id)->getIdNode()<<endl;
        delete[] node;
        return true;
    }
    //cout<<"Aux  :"<< getNodeId(start)->getIdNode()<<endl;
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

                    /*int pa[this->order];
                    int dist[this->order];   
                    bool mature[this->order];*/
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

                        if(dist[auxDest] == INT_MAX )
                        output_file << " Nao existe caminho entre o vertice " << orig << " ao vertice " << dest << endl;
                        else  
                        output_file << " A distancia do vertice " << orig << " ao vertice " << dest << " sera: " << dist[auxDest] << endl;

                } else {
                    /*int pa[this->order];
                    int dist[this->order];   
                    bool mature[this->order];*/
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
                        else if (dist2 != dist[auxDest] && dist2 != -1)
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
    /*int pa[this->order];
        int dist[this->order];   
        bool mature[this->order];*/
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
        /*for(Edge *aux = node->getFirstEdge(); aux != nullptr; aux = aux->getNextEdge())
        {
            if(aux->getTargetId() == idTarget)
            {
                verifica = true;
            }

        }*/
        if(*verifica == true)
        {
            int ordem = this->order;               // recebe ordem do grafo
            //Node *auxNode = this->first_node;     // recebe primeiro no // (NAO ESTA SENDO USADO)
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


    // Não precisa imprimir a matriz
    
   /* Node *auxN = this->first_node;

    
    output_file << " matriz das Distâncias mais curtas entre cada par de vértices:" << endl;
    for (int i = 0; i < ordem + 1; i++)
    {
        if (i == 0)
            output_file << setw(7) << " ";
        else
        {
            output_file << setw(7) << auxN->getId();
            auxN = auxN->getNextNode();
        }
    }

    output_file << endl;
    auxN = this->first_node;

    for (int i = 0; i < ordem; i++)
    {
        for (int j = 0; j < ordem + 1; j++)
        {
            if (j == 0)
            {
                output_file << setw(7) << " | " << auxN->getId() << " | ";
                ;
                auxN= auxN->getNextNode();
            }
            else
            {
                if (dist[i][j - 1] == INT_MAX / 2)
                    output_file << setw(7) << "INF"
                                << " | ";
                else
                    output_file << setw(7) << dist[i][j - 1] << " | ";
            }
        }
        output_file << endl;
    }
    */
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
            //verifica = this->searchNode(aux->getTargetId());


            if (verifica && verifica2)
            {
                // incluir a aresta no noh do subgrafo;
                node->insertEdge(aux->getTargetId(), aux->getWeight(), aux->getTargetIdNode()); 
                 getNode(aux->getTargetId())->insertEdge(node->getId(), aux->getWeight(), node->getIdNode()); 
            } 

           }
        //verificar as arestas no grafo original. 
    }

    

    return subGrafo;
}



// INICIO PRIM ////////////////////////////////////////////////////////////


Graph *Graph::arvGMin_Prim(ofstream &output_file)
{
    this->printGraph(output_file);
     
    int num, aux;
    cout << "Digite o numero de vértices de 1 a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> num;
    int *nodes = new int[num];
    for (int i = 0; i < num; i++)
    {
        cout << "Digite o vértice de numero " << i + 1 << ": " << endl;
        cin >> aux;
        nodes[i] = aux;
        cout << "NODES : " << nodes[i] << " i: " << i << endl;
    }
    
   
  

   Graph *grafoA;

    
    grafoA = this->getVertexInduced(nodes, num);

     

    Graph *grafoB = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);
     Node *v;
    //para todo noh da lista faça
    for (v = grafoA->getFirstNode(); v != NULL; v = v->getNextNode())
    {
        grafoB->insertNode(v->getId(),v->getWeight());
    }

   

   // bool adicionados[this->order]; //marca quais vértices ja possuem um caminho
    bool *adds = new bool [num];
    for (int i = 0; i < num; i++)
    {
        adds[i] = false;
    }
    adds[getNode(aux)->getIdNode()] = true;

    std::list<int> vertices; 
    std::list<int>::iterator k;
    vertices.push_front(aux);

    bool todosNodeAdd = false;

    int custoT=0;

    
    while (todosNodeAdd == false) //até ter um caminho para todos os node
    {
        int nodeA; //node armazena o node de onde vai sair o edge
        int nodeB; //node armazena o node que o edge vai chegar
        int custoMenor = 999999999;
        for (k = vertices.begin(); k != vertices.end(); k++) //percorre todos vértices da lista
        {
            
            Node *verticeAnalisado = grafoA->getNode(*k);
            for (Edge *ed = verticeAnalisado->getFirstEdge(); ed != nullptr; ed = ed->getNextEdge()) //percorre todas arestas de grafoVI
            {
               
               //int verticeAdjacente = ed->getTargetIdNode(); //node target desse edge
                int verticeAdjacente = ed->getTargetId(); //node target desse edge
                int custo_aresta = ed->getWeight();       //custo desse edge
                cout <<"NODE: " << verticeAnalisado->getId() << " EDGE:  "<< ed->getTargetId() <<  "CUSTO DO EDGE:  " << custo_aresta << endl;

                
                // if (adds[verticeAdjacente -1 ] == false) //node alvo não foi adicionado
                if (adds[ed->getTargetIdNode()] == false) //node alvo não foi adicionado0
                {
                    
                    if (custoMenor > custo_aresta) //custo desse edge for menor de todas que ja forram analisados
                    {
                        
                        nodeA = verticeAnalisado->getId(); //node que esta saindo esse edge
                        nodeB = verticeAdjacente; //node onde esta chegando esse edge
                        custoMenor = custo_aresta; //custo desse edge
                    }
                }
            }
            
        }

         cout << " NODEA: " << nodeA << " --- " << " NODEB: " << nodeB << endl;
        grafoB->insertEdge(nodeA, nodeB, custoMenor);

        custoT=custoT+custoMenor;

        
          vertices.push_front(nodeB); //add nodeB no nodes
         //adds[nodeB - 1] = true; //marcar nodeB como adicionado
         // adds[nodeB] = true; //marcar nodeB como adicionado
         adds[getNode(nodeB)->getIdNode()] = true;
        

         for(int i=0;i<num;i++)
          cout<< "ADDS NA POSICAO: "<< i << " SERA : " <<  adds[i] << endl;

        int cont = 0;
        for (int i = 0; i < num; i++) //verifica se todos nodes ja foram adicionados se sim todosNodeAdd=true
        {
            if (adds[i] == true)
            {
                cout << "CHEGOU ATE AQUI" << endl;
                cont++;
            }
        }
        if (cont == (grafoB->order))
        {
            
            todosNodeAdd = true;
        }
        cout  << " CONT : " << cont << endl;
    }

    delete[] nodes;

     // Nao precisa dessa parte, ( se caso for apagar, apagar a variavel custoT)

   
    output_file<<"Peso da Arvore Geradora Minima: "<<custoT<<endl;

   return grafoB;
}


// FIM PRIM ////////////////////////////////////////////////////

// INICIO KRUSKAL ////////////////////////////////////////////////////////////

Graph *Graph::arvGMin_Kruskal(ofstream &output_file)
{
    int num, v;
    cout << "Digite o numero de vértices de '1' a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> num;
    int *nodes = new int[this->order];
    for (int i = 0; i < this->order; i++)
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
    int totalEdge;

    totalEdge = grafoA->getNumberEdges();

    list<pair<int, int>> listP;

    Node *sup;
    Node *p;
    Edge *aux;

    for (sup = grafoA->getFirstNode(); sup != NULL; sup = sup->getNextNode())
    {
        //grafoB->insertNode(sup->getId());
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
            cout << sup->getId() << " -- " << p->getId() << endl;
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
    for (sup = grafoB->getFirstNode(); sup != NULL; sup = sup->getNextNode())
    {
        //agMin->insertNode(sup->getId());
    }

    //F ¬ Æ
    //Cria lista vazia
    list<pair<int, int>> listaAux;

    //contador ¬ 0
    int cont = 0;
    int numMaxAresta = agMin->getOrder() - 1;
    //bool verificado[this->order] = ;
    bool *verificado = new bool[this->order];

    for (int i = 0; i < this->order; i++)
    {
        verificado[i] = false;
    }

    //Enquanto contador < |V|-1 e L 1 Æ faça
    while (cont < numMaxAresta && !listP.empty())
    {
        //Seja (u,v) a próxima aresta de L.
        pair<int, int> dist_no = listP.front(); //copia par (id do vertice e distancia) do topo
        int v1 = dist_no.first;
        int v2 = dist_no.second;

        listP.pop_front();
        //Se u e v não estão na mesma subárvore então
        if (!verificaSubarvore(v1, v2, agMin))
        {
             //F ¬ F È {(u,v)}
            //preenche a lista;
            listaAux.push_back(make_pair(v1, v2));
            //busca o peso da aresta
            int peso = getWeightFromEdgeNodeCombo(v1, v2, grafoB);

            //Unir as subárvores que contêm u e v.
            agMin->insertEdge(v1, v2, peso);
            //contador ¬ contador + 1
            cont++;
            //fim-se
        }
    }

    // NAO PRECISA DESSA PARTE

   /* int pesoT = 0;
    while (!listaAux.empty())
    {
        pair<int, int> dist_no = listaAux.front(); //copia par (id do vertice e distancia) do topo
        int v1 = dist_no.first;
        int v2 = dist_no.second;
        pesoT = pesoT + getWeightFromEdgeNodeCombo(v1, v2, agMin);
        listaAux.pop_front();
    }
    output_file << "Peso da Arvore Geradora Minima: " << pesoT << endl;
    */

    return agMin;

}

// FIM KRUSKAL ////////////////////////////////////////////////////


// INICIO BUSCA EM LARGURA ////////////////////////////////////////////////////////////

void Graph::arv_Buscalargura(ofstream &output_file, int id)
{

  int *num = new int[this->order];
  int *pa = new int[this->order];
  int cont = 0;


    for (int i = 0; i < this->order; i++)
        num[i] = pa[i] = -1;

        
    list<Node*> listN;  
    Node *node1 = getNode(id);
    num[node1->getIdNode()] = cont++; 
    pa[node1->getIdNode()] = id;
    listN.push_back(getNode(id));


    while (!listN.empty()) 
    {
        Node *aux = listN.front();
        listN.pop_front(); 

        for (Edge *auxE = aux->getFirstEdge(); auxE!=NULL; auxE=auxE->getNextEdge())
        {
             if (num[auxE->getTargetIdNode()] == -1) {
                num[auxE->getTargetIdNode()] = cont++; 
                pa[auxE->getTargetIdNode()] = aux->getId();
                listN.push_back(getNode(auxE->getTargetId()));
            }

        }
            
    }

      Graph *arvBL = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);
    
     for(int i=1;i<this->order+1;i++)
        arvBL->insertNode(i,0); 

    

     for(int i=0;i<this->order;i++)
     {
        
         if(i+1 != id )
         {         
           arvBL->insertEdge(pa[i],i+1,0);
         }

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

                output_file << p->getId() << " -- " << aux->getTargetId() <<"PESO : " << aux->getWeight() << endl;
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

                output_file << p->getId() << " -> " << aux->getTargetId() <<"PESO : " << aux->getWeight() << endl;
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
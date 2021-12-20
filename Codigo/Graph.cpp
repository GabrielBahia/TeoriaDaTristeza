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
                aux->insertEdge(node->getId(), node->getWeight(), node->getIdNode()); // inserts the edge between the two nodes;
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

int Graph::dijkstra(int orig, int dest)
{

    if (negative_edge == false)
    {

        //int dist[this->getOrder()];       // vetor de dist�ncias
        int *dist = new int[this->getOrder()]; // vetor de dist�ncias
        //bool visitados[this->getOrder()]; // vetor de visitados para n�o repetir vertices j� expandido
        bool *visitados = new bool [this->getOrder()]; // vetor de visitados para n�o repetir vertices j� expandido
        // fila de prioridades de pair (distancia, v�rtice)
        priority_queue<pair<int, int>,
                       vector<pair<int, int>>, greater<pair<int, int>>>
            pq;

        // inicia o vetor de dist�ncias e visitados
        for (int i = 0; i < this->getOrder(); i++)
        {
            dist[i] = INFINITO;
            visitados[i] = false;
        }

        // a dist�ncia de orig para orig � 0
        dist[orig] = 0;

        // insere na fila
        pq.push(make_pair(dist[orig], orig));

        // loop do algoritmo
        while (!pq.empty())
        {
            pair<int, int> p = pq.top(); // extrai o pair do topo
            //int u = p.second; // obt�m o v�rtice do pair
            Node *u = getNode(p.second);
            pq.pop(); // remove da fila

            // verifica se o v�rtice n�o foi expandido
            //if (visitados[u->getId()] == false)
            if(visitados[u->getIdNode()] == false)
            {
                // marca como visitado
                //visitados[u->getId()] = true;
                visitados[u->getIdNode()] = true;

                //list<pair<int, int> >::iterator it;

                // percorre os v�rtices "v" adjacentes de "u"
                //Edge *p = v->getFirstEdge(); p != NULL; p = p->getNextEdge()
                for (Edge *aux = u->getFirstEdge(); aux != NULL; aux = aux->getNextEdge())
                {
                    // obt�m o v�rtice adjacente e o custo da aresta

                    //int v = aux->getTargetId();
                    int v = aux->getTargetIdNode();
                    int custo_aresta = aux->getWeight();

                    // relaxamento (u, v)
                    //if (dist[v] > (dist[u->getId()] + custo_aresta))
                    if (dist[v] > (dist[u->getIdNode()] + custo_aresta))
                    {
                        // atualiza a dist�ncia de "v" e insere na fila
                        //dist[v] = dist[u->getId()] + custo_aresta;
                        dist[v] = dist[u->getIdNode()] + custo_aresta;
                        pq.push(make_pair(dist[v], v));
                    }
                }
            }
        }

        // retorna a dist�ncia m�nima at� o destino
        return dist[dest];
    }
    return 0;
}

/////////// FIM DIJKSTRA /////////////////////////////////////////////

// INICIO FLOYD ////////////////////////////////////////////////////////////

void Graph::floydMarshall(ofstream &output_file, int idSource, int idTarget)
{
    int tam = this->order;               // recebe ordem do grafo
    Node *auxNode = this->first_node;    // recebe primeiro no
    int ** dist = new int *[this->order]; // inicializando matriz que recebe vetor
    dist = floyd(tam, dist);             // dist recebe funcao floyd
    output_file << "Menor caminho entre o No[" << idSource << "] e o No[" << idTarget << "] e: [" << dist[idSource - 1][idTarget - 1] << "]" << endl;

    Node *auxNode2 = this->first_node;
    output_file << " matriz das Distâncias mais curtas entre cada par de vértices:" << endl;
    for (int i = 0; i < this->order + 1; i++)
    {
        if (i == 0)
            output_file << setw(7) << " ";
        else
        {
            output_file << setw(7) << auxNode2->getId();
            auxNode2 = auxNode2->getNextNode();
        }
    }
    output_file << endl;
    auxNode2 = this->first_node;
    for (int i = 0; i < this->order; i++)
    {
        for (int j = 0; j < this->order + 1; j++)
        {
            if (j == 0)
            {
                output_file << setw(7) << " | " << auxNode2->getId() << " | ";
                
                auxNode2 = auxNode2->getNextNode();
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
}

int **Graph::floyd(int tam, int **dist)
{
    dist = new int *[tam];
    for (int i = 0; i < tam; i++)
    {
        dist[i] = new int[this->order];
    }
    Node *auxNode1 = this->first_node; // ponteiro do primeiro nó recebe primeiro no do grafo
    int peso_edge = 1;                  // peso da aresta
    Node *auxNode2;                    // ponteiro auxiliar para um no
    

    for (int i = 0; auxNode1 != NULL; auxNode1 = auxNode1->getNextNode(), i++)
    {
        auxNode2 = this->first_node;

        for (int j = 0; auxNode2 != NULL; auxNode2 = auxNode2->getNextNode(), j++)
        {
            Edge *aux = auxNode1->hasEdgeBetween(auxNode2->getId());

            if (this->weighted_edge && aux != NULL)
                peso_edge = aux->getWeight();

            if (auxNode1->getId() == auxNode2->getId())
                dist[i][j] = 0;

            else if (aux != NULL)

                dist[i][j] = peso_edge;

            else
                dist[i][j] = INT_MAX / 2;
        }
    }
    for (int k = 0; k < tam; k++)
    {
        for (int i = 0; i < tam; i++)
        {
            if (i != k)
            { 
                for (int j = 0; j < tam; j++)
                {
                    if (dist[i][j] > dist[i][k] + dist[k][j] && dist[i][k] + dist[i][j] > 0)
                        dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }
    return dist;
}

// FIM FLOYD ////////////////////////////////////////////////////

Graph *Graph::agmPrim(ofstream &output_file)
{
    int tam, x;
    cout << "Digite o numero de vértices de 1 a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> tam;
    int *listaNode = new int[tam];
    for (int i = 0; i < tam; i++)
    {
        cout << "Digite o vértice de numero " << i + 1 << ": " << endl;
        cin >> x;
        listaNode[i] = x;
    }

    Graph *grafo1;
    grafo1 = this->getVertexInduced(listaNode, tam);

    Graph *grafo2 = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);
    Node *p;
    //para todo noh da lista faça
    for (p = grafo1->getFirstNode(); p != NULL; p = p->getNextNode())
    {
        grafo2->insertNode(p->getIdNode(),p->getWeight());
    }

   // bool adicionados[this->order]; //marca quais vértices ja possuem um caminho
    bool *adds = new bool [this->order];
    for (int i = 0; i < this->order; i++)
    {
        adds[i] = false;
    }
    adds[x] = true;

    std::list<int> vertices; 
    std::list<int>::iterator k;
    vertices.push_front(x);

    bool todosNodeAdd = false;

    int custoTotal=0;


    while (todosNodeAdd == false) //até ter um caminho para todos os node
    {
        int node1; //node armazena o node de onde vai sair o edge
        int node2; //node armazena o node que o edge vai chegar
        int menorCusto = 999999999;
        for (k = vertices.begin(); k != vertices.end(); k++) //percorre todos vértices da lista
        {
            Node *verticeAnalisado = grafo1->getNode(*k);
            for (Edge *it = verticeAnalisado->getFirstEdge(); it != NULL; it = it->getNextEdge()) //percorre todas arestas de grafoVI
            {
                int verticeAdjacente = it->getTargetIdNode(); //node target desse edge
                int custo_aresta = it->getWeight();       //custo desse edge

                if (adds[verticeAdjacente - 1] == false) //node alvo não foi adicionado
                {
                    if (menorCusto > custo_aresta) //custo desse edge for menor de todas que ja forram analisados
                    {
                        node1 = verticeAnalisado->getId(); //node que esta saindo esse edge
                        node2 = verticeAdjacente; //node onde esta chegando esse edge
                        menorCusto = custo_aresta; //custo desse edge
                    }
                }
            }
        }

        grafo2->insertEdge(node1, node2, menorCusto);

        custoTotal=custoTotal+menorCusto;

        vertices.push_front(node2); //add node2 na listaNode
        adds[node2 - 1] = true; //marcar node2 como adicionado
        int contador = 0;
        for (int i = 0; i < (this->order); i++) //verifica se todos nodes ja foram adicionados se sim todosNodeAdd=true
        {
            if (adds[i] == true)
            {
                contador++;
            }
        }
        if (contador == (grafo2->order))
        {
            todosNodeAdd = true;
        }
    }
    delete[] listaNode;

    output_file<<"Peso da Arvore Geradora Minima: "<<custoTotal<<endl;

    return grafo2;
}


Graph* Graph::getVertexInduced(int *listIdNodes, int tam)
{

    Graph *subGrafo = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);

    for (int i = 0; i < tam; i++)
    {
        if (this->searchNode(listIdNodes[i]))
        {
            subGrafo->insertNode(listIdNodes[i],0);
        }
    }
    Node *node;
    Node *inicio;
    Edge *aux;
    bool verifica = false;
    //para todo noh do subgrafo,
    for (node = subGrafo->getFirstNode(); node != NULL; node = node->getNextNode())
    {
        inicio = getNode(node->getId());

        //verificar as arestas no grafo original.
        for (aux = inicio->getFirstEdge(); aux != NULL; aux = aux->getNextEdge())
        {

            // se a aresta do vertice pra onde ela aponta existir

            verifica = subGrafo->searchNode(aux->getTargetIdNode());
            if (verifica)
            {
                // incluir a aresta no noh do subgrafo;
                node->insertEdge(aux->getTargetId(), aux->getWeight(), aux->getTargetIdNode());
            }
        }
    }
    return subGrafo;
}

Graph *Graph::agmKuskal(ofstream &output_file)
{
    int tam, v;
    cout << "Digite o numero de vértices de '1' a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> tam;
    int *listaNode = new int[this->order];
    for (int i = 0; i < this->order; i++)
    {
        listaNode[i] = -1;
    }
    for (int i = 0; i < tam; i++)
    {
        cout << "Digite o vértice numero " << i + 1 << ": " << endl;
        cin >> v;
        listaNode[i] = v;
    }
    //pre-requisitos pra fazer a ordenacao
    Graph *grafo1;
    grafo1 = this->getVertexInduced(listaNode, tam);

    Graph *grafoAux = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node); //vai vira o grafoVI

    int *EdgeNode = new int[3];
    int totalEdge;

    totalEdge = grafo1->getNumberEdges();

    list<pair<int, int>> lista;

    Node *sup;
    Node *p;
    Edge *aux;

    for (sup = grafo1->getFirstNode(); sup != NULL; sup = sup->getNextNode())
    {
        //grafoAux->insertNode(sup->getId());
    }

    //Criar uma lista L com as arestas ordenadas em
    //ordem crescente de pesos.
    for (int i = 0; i < totalEdge; i++)
    {
        // acha a aresta de menor peso
        grafo1->getWeithlessEdge(EdgeNode);
        //insere a aresta de menor peso
        lista.push_back(make_pair(EdgeNode[0], EdgeNode[1]));
        //retira a aresta do grafo pra evitar repetir a mesma aresta;
        sup = grafo1->getNode(EdgeNode[0]);
        p = grafo1->getNode(EdgeNode[1]);
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
        grafoAux->insertEdge(EdgeNode[0], EdgeNode[1], EdgeNode[2]);
    }

    //Organizar a lista;

    //Criar |V| subárvores contendo cada uma um nó
    //isolado.
    Graph *arvoreGerMin = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);
    for (sup = grafoAux->getFirstNode(); sup != NULL; sup = sup->getNextNode())
    {
        //arvoreGerMin->insertNode(sup->getId());
    }

    //F ¬ Æ
    //Cria lista vazia
    list<pair<int, int>> listaAux;

    //contador ¬ 0
    int conta = 0;
    int numMaxAresta = arvoreGerMin->getOrder() - 1;
    //bool verify[this->order] = ;
    bool *verify = new bool[this->order];

    for (int i = 0; i < this->order; i++)
    {
        verify[i] = false;
    }

    //Enquanto contador < |V|-1 e L 1 Æ faça
    while (conta < numMaxAresta && !lista.empty())
    {
        //Seja (u,v) a próxima aresta de L.
        pair<int, int> distancia_no = lista.front(); //copia par (id do vertice e distancia) do topo
        int v1 = distancia_no.first;
        int v2 = distancia_no.second;

        lista.pop_front();
        //Se u e v não estão na mesma subárvore então
        if (!verificaSubarvore(v1, v2, arvoreGerMin))
        {
             //F ¬ F È {(u,v)}
            //preenche a lista;
            listaAux.push_back(make_pair(v1, v2));
            //busca o peso da aresta
            int peso = getWeightFromEdgeNodeCombo(v1, v2, grafoAux);

            //Unir as subárvores que contêm u e v.
            arvoreGerMin->insertEdge(v1, v2, peso);
            //contador ¬ contador + 1
            conta++;
            //fim-se
        }
    }

    int pesoTotal = 0;
    while (!listaAux.empty())
    {
        pair<int, int> distancia_no = listaAux.front(); //copia par (id do vertice e distancia) do topo
        int v1 = distancia_no.first;
        int v2 = distancia_no.second;
        pesoTotal = pesoTotal + getWeightFromEdgeNodeCombo(v1, v2, arvoreGerMin);
        listaAux.pop_front();
    }
    output_file << "Peso da Arvore Geradora Minima: " << pesoTotal << endl;
    return arvoreGerMin;

}

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

                output_file << p->getId() << " -- " << aux->getTargetId() << endl;
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

                output_file << p->getId() << " -> " << aux->getTargetId() << endl;
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

void Graph::ordenacaoTopologica(ofstream &output_file)
{
    list<Node*> listaNode;
    list<int> TopologicList;
    if (this->graphTemCiclo())// verifica se o grafo é aciclico ou não
    {
        output_file <<" Se o Grafo possui ciclos, logo, nao possui ordenação topologica"<<endl;
    }
    else{ // adaptando algoritimo kahn's
            Node *auxNode;
            Edge *auxEdge;
            //procurando nos com enttrada =0
            for (auxNode=this->first_node;auxNode!=NULL;auxNode = auxNode->getNextNode())
            {   if (auxNode->getInDegree()==0)// se entrada  = 0
                {
                    listaNode.push_back(auxNode); //coloca os nos corretos na fila
                }
            }
            while (!listaNode.empty())// enquanto lista e vazia
            {
                Node *aux = listaNode.front();
                listaNode.pop_front(); //remove da lista
                TopologicList.push_back(aux->getId()); //coloca na lista auxiliar
                for(auxEdge =aux->getFirstEdge(); auxEdge!=NULL;auxEdge=auxEdge->getNextEdge())
                {
                    auxNode = this->getNode(auxEdge->getTargetId()); //pega o no vizinho
                    auxNode->decrementInDegree(); //decrementa o grau de entrada
                    if (auxNode->getInDegree()==0) //se a entrada = 0
                    {
                        listaNode.push_back(auxNode);
                    }

                }
            }
            //imprimindo ordenaçao a classificação topologica
            output_file << "Ordenação Topologica :" << endl;
            for(list<int>::iterator i = TopologicList.begin(); i != TopologicList.end(); i++)
            {
                    if(TopologicList.size() == this->getOrder())
                    output_file << (*i) << endl;
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
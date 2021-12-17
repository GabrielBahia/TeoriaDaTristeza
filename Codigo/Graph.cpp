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
void Graph::insertNode(int id)
{
    Node *node = new Node(id, order);
    ///node->setNumber(order + 1);
 
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
                aux->insertEdge(node->getId(), node->getWeight(), aux->getIdNode()); // inserts the edge between the two nodes;
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

/// ALL THIS FUNCTIONS HERE WE DONT KNOW ALREADY

//Function that prints a set of edges belongs breadth tree

/*void Graph::breadthFirstSearch(ofstream &output_file, int id_inicial)
{ /// No parametro dessa fun��o, n�o deveria ser o id?
}*/

/// ATUAL

void Graph::fechoTransitivoDireto(ofstream &output_file, int id)
{

    //com o id do v�rtice acha o vertice que deve ser analisado
    //int idParametro = getNode(id)->getIdNode(); // vai pegar a posi��o exata em um vetor;
    //output_file << idParametro;
    //cria um vetor que marca quais v�rtices ja foram analisados
    //bool visitados[this->order];
    bool *visitados = new bool[this->order];
    //cria o vetor fecho transitivo direto
    //bool vet_ftd[this->order];
    bool *vet_ftd = new bool[this->order];
    //cria uma fila que diz quais vertices ainda precisam ser analisados
    queue<int> fila;
    //adiciona o vertice inicial nele
    fila.push(id);
    //ordem.push(id); //nao sabemos praq
    for (int i = 0; i < this->order; i++)
    {
        visitados[i] = false;
        vet_ftd[i] = false;
    }

    //come�a itera��o (enquanto a fila n�o estiver vazia repita)
    while (!(fila.empty()))
    {
        //pega um v�rtice a ser analisado da fila
        ///int aux = fila.front();
        int IdVet = getNode(fila.front())->getIdNode(); // j� que os vetores come�am da posi��o 0, isso possivelmente equivale a passar a posi��o equivalente do id do vertice no vetor
        Node *V = getNode(fila.front());
        output_file << IdVet;
        ///V = getNode(fila.front());
        //exclui ele da fila
        fila.pop();
        //verifica se o v�rtice a ser analisado ja foi analisado. (se ele ja foi acaba essa itera��o)
        if (visitados[IdVet] == false)
        {
            //marca o v�rtice como visitado;
            visitados[IdVet] = true;
            //adiciona ele no vetor fecho transitivo direto
            vet_ftd[IdVet] = true;
            //adiciona todos os v�rtices adjacentes a ele na fila
            for (Edge *it = V->getFirstEdge(); it != NULL; it = it->getNextEdge())
            {
                int verticeAdjacente = it->getTargetId(); // aqui ele possivelmente t� passando o id do vertice com o qual it(ou seja V) est� ligado pela aresta e que tem como id o v�rtice alvo
                fila.push(verticeAdjacente);
            }
        }
    }
    
    //imprimir o FTD
    output_file << "O conjunto FTD do v�rtice " << id << " �: {";
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

void Graph::fechoTransitivoIndireto(ofstream &output_file, int id)
{
    //cout << this->order;
    bool *fti = new bool[this->order];    // vetor para verificar o fecho transitivo indireto
    bool *node = new bool[this->order];   // vetor para verificar os vizinhos
    int *vet = new int[this->order];
    int cont = 0;

    for (int i = 0; i < this->order; i++) // passando false para tudo antes de come�ar
    {
        fti[i] = false;
        node[i] = false;
    }

    int conta = 0; // auxilia a descobrir qual o ultimo vertice para quando a gente for printar n�o colocar uma "," depois do ultimo

    for (Node *p = this->first_node; p != nullptr; p = p->getNextNode()) /// percorre todos os vertices
    {
       /* cout << "Posição no vetor: " << p->getIdNode() << endl;
        cout << "Id do node: " << p->getId() << endl; */
        if (!node[p->getIdNode()]) // se a posi��o do vetor que equivale ao indice do vertice-1 j� que a posi��o do vetor come�a do 0, se ela for false o c�digo ocorre, pois ainda n�o passamos por esse vertice
        {
            node[p->getIdNode()] = true;                            // passa true para a posi��o atual
            fti[p->getIdNode()] = deephFirstSearch1(id, p->getId()); // chama a busca em profundidade passando o id que queremos e o id equivalente ao no que estamos no for
            if (fti[p->getIdNode()])                                  // se true, ou seja se � possivel desse vertice p chegar ao vertice "id" que � parametro da fun��o ent�o conta++ para auxiliar com a impress�o, assim como est� escrito ali em cima;
            {
                cout<<"vizinho: " << p->getIdNode()<<endl;
                conta++;
                vet[cont] = p->getIdNode(); 
                cont++;
            }
        }
    }
      cout<< "saiu do for" << endl;
    output_file << "O fecho transitivo indireto de " << id << "�: ";
    output_file << "{ ";
    output_file << this->order;
    int aux = 0;
    for (int i = 0; i < cont; i++)
    {
        if (fti[vet[i]])
        {
            if (aux == conta - 1)
            {
                output_file << (getNode(i)->getId());
                aux++;
            }
            else
            {
                output_file << (getNode(i)->getId()) << ",";
                aux++;
            }
        }
    }
}

bool Graph::deephFirstSearch1(int id, int start)
{
    //cout << start << endl;
    //Criando vetor para verificar e tamb�m vetor predecessor de profundidade
    bool *verify = new bool[this->order]; // vetor do tamanho do grafo
    int conta = 0;
    int idParametro; // equivale a posi��o do id do vertice no vetor;
    for (int i = 0; i < this->order; i++)
    {
        verify[i] = false; // passa false para todas as posi��es
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
        cout<< "Vizinhos: "<< p->getId()<<endl;
        auxDeepthFirstSearch1(verify, p); // passa o vetor e o node que estamos come�ando, pode ser 0,1,2 ... depende de onde o for
    }                                    // da fecho transitivo indireto est� chamando
    else
    {
        return true; // retorna true caso o v�rtice em que estamos � o vertice que queremos
    }

    //Se encontrou
    if (verify[getNode(id)->getIdNode()]) // dps de passar pela aux ele verifica se foi mudado por parametro a posi��o equivalente
    {                                     // ao id que queremos no vetor, caso a gente queira o vertice 3 e passamos o vertice 8 que est� ligado
                                          // aos v�rtices 9 e 10, somente as posi��es 7,8,9 receberiam true, ou seja 8 n�o chega ao v�rtice 3, ou seja n�o entre nesse if
        //cout<< getNodeId(id)->getIdNode()<<endl;
        cout<< "Vizinho(2) :"<< p->getId()<<endl; 
        delete[] verify;
        return true;
    }
    cout<<"Aux  :"<< getNodeId(id)->getIdNode()<<endl;
    delete[] verify;
    return false;
}

void Graph::auxDeepthFirstSearch1(bool verify[], Node *v)
{
    //Protocolo inicial
    int idParametro = v->getIdNode(); /// pega a posi��o equivalente do id desse v�rtice no vetor;

    Node *aux;

    //Marca v como visitado;
    verify[idParametro] = true; /// marca o v�rtice que estamos como visitado;

    //Para todo w em Adj(v)
    for (Edge *p = v->getFirstEdge(); p != NULL; p = p->getNextEdge())
    {
        idParametro = p->getTargetIdNode(); /// pega a posi��o no vetor dos vizinhos que a gente quer verificar
        cout << " idParametro: "<< idParametro << endl ;
        //Se w n�o visitado ent�o

        if (!verify[idParametro])
        {
            cout<<"p->getTargetIdNode() :"<< p->getTargetIdNode() <<endl;
            aux = getNode(p->getTargetId());
            //AuxBuscaEmProfundidade(G,w);
            auxDeepthFirstSearch1(verify, aux);
        }
    }
    /*
        Explicando essa fun��o acima: se por exemplo n�s chamamos atraves da deepthFirstSearch o v�rtice 8
        que � ligado aos v�rtices 9 e 10, por�m estamos buscando o v�rtice 3, ele retornara para a fecho
        transitivo indireto falso, pois na linha 355 n�s passamos para todos os v�rtices falso, na chamada
        da fun��o na qual o v�rtice 8 foi chamado, logo ap�s n�s chamamos essa fun��o aq passando o vetor com
        tudo falso e o v�rtice 8, como ele est� ligado ao 9 e 10, somente esses 3 ir�o receber true, oq acabara
        fazendo com que a posi��o 2 ou seja (3-1) que equivale a posi��o do vetor, continue false

    */
}

void Graph::floydMarshall(ofstream &output_file, int idSource, int idTarget)
{
    int tam = this->order;               // recebe ordem do grafo
    Node *auxNode = this->first_node;    // recebe primeiro no
    int ** dist = new int *[this->order]; // inicializando matriz que recebe vetor
    dist = floyd(tam, dist);             // dist recebe funcao floyd
    output_file << "O menor caminho entre o No[" << idSource << "] e o No[" << idTarget << "] e: [" << dist[idSource - 1][idTarget - 1] << "]" << endl;

    Node *aux_node2 = this->first_node;
    output_file << " matriz das Distâncias mais curtas entre cada par de vértices:" << endl;
    for (int i = 0; i < this->order + 1; i++)
    {
        if (i == 0)
            output_file << setw(7) << " ";
        else
        {
            output_file << setw(7) << aux_node2->getId();
            aux_node2 = aux_node2->getNextNode();
        }
    }
    output_file << endl;
    aux_node2 = this->first_node;
    for (int i = 0; i < this->order; i++)
    {
        for (int j = 0; j < this->order + 1; j++)
        {
            if (j == 0)
            {
                output_file << setw(7) << " | " << aux_node2->getId() << " | ";
                ;
                aux_node2 = aux_node2->getNextNode();
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
{ // fucnao para utilizar lista de adj e para usar o alg de floyd

    dist = new int *[tam];
    for (int i = 0; i < tam; i++)
    {
        dist[i] = new int[this->order];
    }
    Node *aux_node1 = this->first_node; // ponteiro do primeiro nó recebe primeiro no do grafo
    Node *aux_node2;                    // ponteiro auxiliar para um no
    int peso_edge = 1;                  // peso da aresta
    // matriz com os valores de cada aresta entre os nos
    for (int i = 0; aux_node1 != NULL; aux_node1 = aux_node1->getNextNode(), i++)
    {
        aux_node2 = this->first_node;

        for (int j = 0; aux_node2 != NULL; aux_node2 = aux_node2->getNextNode(), j++)
        {
            Edge *aux = aux_node1->hasEdgeBetween(aux_node2->getId());

            if (this->weighted_edge && aux != NULL)
                peso_edge = aux->getWeight();

            if (aux_node1->getId() == aux_node2->getId())
                dist[i][j] = 0;

            else if (aux != NULL)

                dist[i][j] = peso_edge;

            else
                dist[i][j] = INT_MAX / 2;
        }
    }
    for (int c = 0; c < tam; c++)
    {
        // Escolhendo todos os vértices como fonte, um por um
        for (int i = 0; i < tam; i++)
        {
            if (i != c)
            { // Escolhendo todos os vértices como destino
                for (int j = 0; j < tam; j++)
                {
                    //Se o vértice c estiver no caminho mais curto de i para j, em seguida, atualize o valor de dist [i] [j]
                    if (dist[i][j] > dist[i][c] + dist[c][j] && dist[i][c] + dist[i][j] > 0)
                        dist[i][j] = dist[i][c] + dist[c][j];
                }
            }
        }
    }
    return dist;
}

int Graph::dijkstra(int orig, int dest)
{

    if (negative_edge != true)
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
            if (visitados[u->getId()] == false)
            {
                // marca como visitado
                visitados[u->getId()] = true;

                //list<pair<int, int> >::iterator it;

                // percorre os v�rtices "v" adjacentes de "u"
                //Edge *p = v->getFirstEdge(); p != NULL; p = p->getNextEdge()
                for (Edge *aux = u->getFirstEdge(); aux != NULL; aux = aux->getNextEdge())
                {
                    // obt�m o v�rtice adjacente e o custo da aresta

                    int v = aux->getTargetId();
                    int custo_aresta = aux->getWeight();

                    // relaxamento (u, v)
                    if (dist[v] > (dist[u->getId()] + custo_aresta))
                    {
                        // atualiza a dist�ncia de "v" e insere na fila
                        dist[v] = dist[u->getId()] + custo_aresta;
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

//function that prints a topological sorting
void Graph::topologicalSorting(ofstream &output_file)
{
    list<Node*> listaNos;
    list<int> listaTopologica;
    if (this->graphCiclo())// verifica se o grafo e aciclico ou nao
    {
        output_file <<" Se Grafo possui ciclos, nao possui ordenação topologica"<<endl;
    }
    else{ // adaptando algoritimo kahn's
            Edge *auxAres;
            Node *auxNo;
            //procurando nos com enttrada =0
            for (auxNo=this->first_node;auxNo!=NULL;auxNo = auxNo->getNextNode())
            {   if (auxNo->getInDegree()==0)// se entrada  = 0
                {
                    listaNos.push_back(auxNo); //coloca os nos corretos na fila
                }
            }
            while (!listaNos.empty())// enquanto lista e vazia
            {
                Node *aux = listaNos.front();
                listaNos.pop_front(); //remove da lista
                listaTopologica.push_back(aux->getId()); //coloca na lista auxiliar
                for(auxAres =aux->getFirstEdge(); auxAres!=NULL;auxAres=auxAres->getNextEdge())
                {
                    auxNo = this->getNode(auxAres->getTargetId()); //pega o no vizinho
                    auxNo->decrementInDegree(); //decrementa o grau de entrada
                    if (auxNo->getInDegree()==0) //se a entrada = 0
                    {
                        listaNos.push_back(auxNo);
                    }

                }
            }
            //imprimindo ordenaçao a classificação topologica
            output_file << "Ordenação Topologica :" << endl;
            for(list<int>::iterator i = listaTopologica.begin(); i != listaTopologica.end(); i++)
            {
                    if(listaTopologica.size() == this->getOrder())
                    output_file << (*i) << endl;
            }

        }
}

bool Graph::graphCiclo()
{
    list<int> auxC;
    // coloca os components em uma lista
    for (int i = 0; i < this->order; i++)
    {
        auxC.push_back(i);
    }// Então a lista é classificada
    auxC.sort();

    for (list<int>::iterator i = auxC.begin(); i !=  auxC.end();){
     int prev = *i;
        i++;
        // Se houver componentes iguais, o gráfo é cíclico,
         // entao o grafo tem um circuito
        if (prev == *i)
            return true;
    }
        // Se  forem diferentes entre eles, o grafo nao tem circuito
    return false;
}

void breadthFirstSearch(ofstream &output_file)
{
}

Graph *Graph::agmPrim(ofstream &output_file)
{
    int tamanho, x;
    cout << "Digite o numero de vértices de 1 a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> tamanho;
    int *listaNos = new int[tamanho];
    for (int i = 0; i < tamanho; i++)
    {
        cout << "Digite o vértice numero " << i + 1 << ": " << endl;
        cin >> x;
        listaNos[i] = x;
    }

    Graph *grafoVI;
    grafoVI = this->getVertexInduced(listaNos, tamanho);

    Graph *grafoX = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);
    Node *p;
    //para todo noh da lista faça
    for (p = grafoVI->getFirstNode(); p != NULL; p = p->getNextNode())
    {
        grafoX->insertNode(p->getId());
    }

   // bool adicionados[this->order]; //marca quais vértices ja possuem um caminho
    bool *adicionados = new bool [this->order];
    for (int i = 0; i < this->order; i++)
    {
        adicionados[i] = false;
    }
    adicionados[x] = true;

    std::list<int> vertices; //marca quais vértices ja possuem um caminho
    std::list<int>::iterator k;
    vertices.push_front(x); //adiciona o primeiro vértice na lista

    bool todosVerticesAdicionados = false;

    int custoTotal=0;


    while (todosVerticesAdicionados == false) //repetir até ter um caminho para todos os vértices
    {
        int vertice1; //nó que vai armazenar o vértice de onde vai sair a aresta
        int vertice2; //nó que vai armazenar o vértice que a aresta vai chegar
        int menorCusto = 999999999;
        for (k = vertices.begin(); k != vertices.end(); k++) //percorre todos vértices da lista
        {
            Node *verticeAnalisado = grafoVI->getNode(*k);
            for (Edge *it = verticeAnalisado->getFirstEdge(); it != NULL; it = it->getNextEdge()) //percorre todas arestas de grafoVI
            {
                int verticeAdjacente = it->getTargetId(); //pega o vértice alvo dessa aresta
                int custo_aresta = it->getWeight();       //pega o custo dessa aresta

                if (adicionados[verticeAdjacente - 1] == false) //se o vértice alvo não foi adicionado
                {
                    if (menorCusto > custo_aresta) //se o custo dessa aresta for menor de todas que ja forram analisados
                    {
                        vertice1 = verticeAnalisado->getId();                   //lembra do nó que esta saindo essa aresta
                        vertice2 = verticeAdjacente; //lembra do nó onde esta chegando essa arresta
                        menorCusto = custo_aresta;                     //lembra do custo dessa aresta
                    }
                }
            }
        }

        //adiciona uma aresta entre o vértice 1 e 2 que possui custo = menorCusto
        grafoX->insertEdge(vertice1, vertice2, menorCusto);

        custoTotal=custoTotal+menorCusto;

        vertices.push_front(vertice2);    //adiciona o vertice 2 na lista vertices
        adicionados[vertice2 - 1] = true; //marcar o vertice 2 como adicionado
        int contador = 0;
        for (int i = 0; i < (this->order); i++) //verificar se todos vértices ja foram adicionados se sim todosVerticesAdicionados=true
        {
            if (adicionados[i] == true)
            {
                contador++;
            }
        }
        if (contador == (grafoX->order))
        {
            todosVerticesAdicionados = true;
        }
    }
    delete[] listaNos;

    output_file<<"Peso da Arvore Geradora Minima: "<<custoTotal<<endl;

    return grafoX;
}


Graph* Graph::getVertexInduced(int *listIdNodes, int tam)
{

    Graph *subGrafo = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);
    //para todo noh da lista faça

    for (int i = 0; i < tam; i++)
    {

        //incluir noh no subgrafo
        if (this->searchNode(listIdNodes[i]))
        {
            //subGrafo->insertNode(listIdNodes[i]);
        }
    }
    Node *p;
    Node *orig;
    Edge *aux;
    bool verificaSeTem = false;
    //para todo noh do subgrafo,
    for (p = subGrafo->getFirstNode(); p != NULL; p = p->getNextNode())
    {
        orig = getNode(p->getId());

        //verificar as arestas no grafo original.
        for (aux = orig->getFirstEdge(); aux != NULL; aux = aux->getNextEdge())
        {

            // se a aresta do vertice pra onde ela aponta existir

            verificaSeTem = subGrafo->searchNode(aux->getTargetId());
            if (verificaSeTem)
            {
                // incluir a aresta no noh do subgrafo;
                subGrafo->insertEdge(p->getId(), aux->getTargetId(), aux->getWeight());
            }
        }
    }
    // retorna subgrafo
    return subGrafo;
}

Graph *Graph::agmKuskal(ofstream &output_file)
{
    //transformar o abaixo em codigo e usar o vetor listaNos como parametro
    int tamanho, x;
    cout << "Digite o numero de vértices de '1' a " << this->order << " que serão adicionados no subgrafo vértice induzido" << endl;
    cin >> tamanho;
    int *listaNos = new int[this->order];
    for (int i = 0; i < this->order; i++)
    {
        listaNos[i] = -1;
    }
    for (int i = 0; i < tamanho; i++)
    {
        cout << "Digite o vértice numero " << i + 1 << ": " << endl;
        cin >> x;
        listaNos[i] = x;
    }
    //pre-requisitos pra fazer a ordenacao
    Graph *grafoVI;
    grafoVI = this->getVertexInduced(listaNos, tamanho);

    Graph *grafoAux = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node); //vai vira o grafoVI

    int *nohArestas = new int[3];
    int totalArestas;

    totalArestas = grafoVI->getNumberEdges();

    list<pair<int, int>> lista;

    Node *sup;
    Node *p;
    Edge *aux;

    for (sup = grafoVI->getFirstNode(); sup != NULL; sup = sup->getNextNode())
    {
        //grafoAux->insertNode(sup->getId());
    }

    //Criar uma lista L com as arestas ordenadas em
    //ordem crescente de pesos.
    for (int i = 0; i < totalArestas; i++)
    {
        // acha a aresta de menor peso
        grafoVI->getWeithlessEdge(nohArestas);
        //insere a aresta de menor peso
        lista.push_back(make_pair(nohArestas[0], nohArestas[1]));
        //retira a aresta do grafo pra evitar repetir a mesma aresta;
        sup = grafoVI->getNode(nohArestas[0]);
        p = grafoVI->getNode(nohArestas[1]);
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
        grafoAux->insertEdge(nohArestas[0], nohArestas[1], nohArestas[2]);
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

        fti[p->getId() - 1] = subGrafo->deephFirstSearch1(v2, p->getId());
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
/*
void Graph::printGraph(ofstream &output_file)
{
    output_file << endl;
    output_file << endl;
    output_file << endl;
    Node *p = this->first_node;
    if(p == NULL) {
        output_file << "C++ fdp";
    }
    //output_file << p->getId();
    while(p != NULL) {
        output_file << p->getId();
        output_file << endl;
        p = p->getNextNode(); 
    }
}*/

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


void Graph::printEdge(ofstream &output_file)
{
    output_file << endl;
    output_file << endl;
    output_file << endl;
    Node *p = this->first_node;
    if(p == NULL) {
        output_file << "C++ fdp";
    }
    //output_file << p->getId();
    while(p != NULL) {
        Edge *edge = p->getFirstEdge();
        while(edge != nullptr) {
            output_file << "Arestas do: " << p->getId() << ": " << edge->getTargetId() << endl;
            edge = edge->getNextEdge();
            //stdout pra imprime na tela;
        }
        output_file << p->getId();
        output_file << endl;
        p = p->getNextNode(); 
    }
}
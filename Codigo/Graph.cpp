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
// Other methods
/*
    The outdegree attribute of nodes is used as a counter for the number of edges in the graph.
    This allows the correct updating of the numbers of edges in the graph being directed or not.
*/
void Graph::insertNode(int id)
{
    Node *node = new Node(id,order);
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
        if(weight < 0)
            this->negative_edge = true; // verifica se o peso da aresta � negativa

        if(this->directed)
        {
            Node *node = getNode(id); // search the actual node that's being called;
            if(!node->hasEdgeBetween(target_id)) // return if theres is no edge between the node id and the node target_id
            {
                node->insertEdge(target_id,weight,getNode(target_id())->getIdNode());  // inserts the edge between the node we are to the node targeted
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
                while(previous != nullptr) // enquanto previous n�o chegar no ultimo vertice
                {
                    previous->removeEdge(id,this->directed,previous); // vai chamando todos os vertices e verificando se eles tem aresta com o id(vertice que a gente quer excluir)
                    previous = previous->getNextNode(); // passa o previous para o prox vertice
                }
                node->removeAllEdges(); // remove todas as arestas do vertice que a gente vai deletar, n�o sei se � necessario j� que a gente j� vai deletar ele mesmo, mas fica ai
                delete node; // deleta o node que a gente quer
                delete previous; // deleta o previous que usamos na fun��o
                order--; // diminui a ordem do grafo ou seja o n�mero de v�rtices presente nele, j� que excluimos um v�rtice;
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

Node *Graph::getNodeId(int id)
{
    Node *node = this->first_node; // first node to the actual node
    for( node; node!= nullptr; node = node->getNextNode() )  // searching for the node
    {
        if(node->getIdNode() == id) // if it finds it returns the node
        {
            return node;
        }
    }
    return nullptr; // else it returns nullptr;
}



/// ALL THIS FUNCTIONS HERE WE DONT KNOW ALREADY

//Function that prints a set of edges belongs breadth tree

void Graph::breadthFirstSearch(ofstream &output_file, int id_inicial){ /// No parametro dessa fun��o, n�o deveria ser o id?


}

/// ATUAL

void Graph::fechoTransitivoDireto(ofstream &output_file, int id)
{

    //com o id do v�rtice acha o vertice que deve ser analisado
    int idParametro = getNode(id)->getIdNode(); // vai pegar a posi��o exata em um vetor;
    //cria um vetor que marca quais v�rtices ja foram analisados
    bool visitados[this->order];
    //cria o vetor fecho transitivo direto
    bool vet_ftd[this->order];
    //cria uma fila que diz quais vertices ainda precisam ser analisados
    queue<int> fila;
    //adiciona o vertice inicial nele
    fila.push_front(id);
    ordem.push_front(id);

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
        ///V = getNode(fila.front());
        //exclui ele da fila
        fila.pop_front();
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
                fila.push_front(verticeAdjacente);

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
    bool *fti = new bool[this->order]; // vetor para verificar o fecho transitivo indireto
    bool *node = new bool[this->order]; // vetor para verificar os vizinhos
    for(int i =0;i<this->order;i++) // passando false para tudo antes de come�ar
    {
        fti[i] = false;
        verify[i] = false;
    }

    int conta = 0; // auxilia a descobrir qual o ultimo vertice para quando a gente for printar n�o colocar uma "," depois do ultimo

    for(Node *p = this->first_node; p!= nullptr; p = p->getNextNode()) /// percorre todos os vertices
    {
        if(!verify[p->getIdNode()]) // se a posi��o do vetor que equivale ao indice do vertice-1 j� que a posi��o do vetor come�a do 0, se ela for false o c�digo ocorre, pois ainda n�o passamos por esse vertice
        {
            verify[p->getIdNode()] = true; // passa true para a posi��o atual
            fti[p->getIdNode()] = deepthFirstSearch1(id, p->getId()); // chama a busca em profundidade passando o id que queremos e o id equivalente ao no que estamos no for
            if(fti[p->getIdNode()]) // se true, ou seja se � possivel desse vertice p chegar ao vertice "id" que � parametro da fun��o ent�o conta++ para auxiliar com a impress�o, assim como est� escrito ali em cima;
            {
                conta++;
            }
        }
    }

    output_file << "O fecho transitivo indireto de " << id << "�: ";
    output_file << "{";

    int aux = 0;
    for(int i = 0;i < this->order;i++)
    {
        if(fti[i])
        {
            if(aux == conta - 1)
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

    //Criando vetor para verificar e tamb�m vetor predecessor de profundidade
    bool *verify = new bool[this->order]; // vetor do tamanho do grafo
    int conta = 0;
    int idParametro; // equivale a posi��o do id do vertice no vetor;
    for(int i = 0;i < this->order; i++)
    {
        verify[i] = false; // passa false para todas as posi��es
    }
    // cria vetor para auxiliar
    Node *p;

    //Para todo v em G;
    p = getNode(start); // passa o primeiro v�rtice do grafo para o Node p;
    idParametro = p->getIdNode(); /// passa a posi��o equivalente do id de p em rela��o ao vetor
    //Se v n�o visitado ent�o

    if(id != p->getId()) // se o v�rtice que foi passado como parametro nessa fun��o que � chamada pela fecho transitivo indireta
    {                    // n�o for igual ao id, ele faz isso, caso contrario obviamente � pois j� estamos no v�rtice que queremos buscar
        //Aux-BuscaEmProfundida(G,v);
        auxDeephFirstSearch1(verify, p); // passa o vetor e o node que estamos come�ando, pode ser 0,1,2 ... depende de onde o for
    }                                    // da fecho transitivo indireto est� chamando
    else
    {
        return true; // retorna true caso o v�rtice em que estamos � o vertice que queremos
    }

    //Se encontrou
    if(verify[getNode(id)->getIdNode()]) // dps de passar pela aux ele verifica se foi mudado por parametro a posi��o equivalente
    {                  // ao id que queremos no vetor, caso a gente queira o vertice 3 e passamos o vertice 8 que est� ligado
                        // aos v�rtices 9 e 10, somente as posi��es 7,8,9 receberiam true, ou seja 8 n�o chega ao v�rtice 3, ou seja n�o entre nesse if
        delete[] verify;
        return true;
    }
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
    for(Edge *p = v->getFirstEdge(); p != NULL; p = p->getNextEdge())
    {
        id Parametro = p->getTargetIdNode(); /// pega a posi��o no vetor dos vizinhos que a gente quer verificar
        //Se w n�o visitado ent�o

        if(!verify[idParametro])
        {

            aux = getNode(p->getTargetId());
            //AuxBuscaEmProfundidade(G,w);
            auxDeephFirstSearch1(verify, aux);
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

float Graph::floydMarshall(int idSource, int idTarget){

}



float Graph::dijkstra(int orig, int dest){


       if(testNegativos == true)
       {

		int dist[this->getOrder()];  // vetor de dist�ncias
		bool visitados[this->getOrder()]; // vetor de visitados para n�o repetir vertices j� expandido
		// fila de prioridades de pair (distancia, v�rtice)
		priority_queue < pair<int, int>,
                    vector<pair<int, int> >, greater<pair<int, int> > > pq;

		// inicia o vetor de dist�ncias e visitados
		for(int i = 0; i < this->getOrder(); i++)
		{
			dist[i] = INFINITO;
			visitados[i] = false;
		}

		// a dist�ncia de orig para orig � 0
		dist[orig] = 0;

		// insere na fila
		pq.push(make_pair(dist[orig], orig));

		// loop do algoritmo
		while(!pq.empty())
		{
			pair<int, int> p = pq.top(); // extrai o pair do topo
			//int u = p.second; // obt�m o v�rtice do pair
			Node *u = getNode(p.second);
			pq.pop(); // remove da fila

			// verifica se o v�rtice n�o foi expandido
			if(visitados[u->getId()] == false)
			{
				// marca como visitado
				visitados[u->getId()] = true;

				//list<pair<int, int> >::iterator it;

				// percorre os v�rtices "v" adjacentes de "u"
				//Edge *p = v->getFirstEdge(); p != NULL; p = p->getNextEdge()
				for(Edge *aux = u->getFirstEdge(); aux != NULL; aux = aux->getNextEdge())
				{
					// obt�m o v�rtice adjacente e o custo da aresta
					Node *v = getNode(p->getTargetId());
					int custo_aresta = aux->getWeight();

					// relaxamento (u, v)
					if(dist[v] > (dist[u] + custo_aresta))
					{
					    idInteiro = v->getId();
						// atualiza a dist�ncia de "v" e insere na fila
						dist[v] = dist[u] + custo_aresta;
						pq.push(make_pair(dist[v], idInteiro));
					}
				}
			}
		}

		// retorna a dist�ncia m�nima at� o destino
		return dist[dest];

       }
    }

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

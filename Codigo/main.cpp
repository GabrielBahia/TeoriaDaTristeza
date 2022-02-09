#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <utility>
#include <tuple>
#include <iomanip>
#include <stdlib.h>
#include <chrono>
#include "Graph.h"
#include "Node.h"
#include <thread>
//#include <TIME.H>
#include <time.h>
//#include <dos.h>
#include <Windows.h>
using namespace std;
//using this_thread::sleep_for;
//using namespace chrono_literals;

Graph* leitura(ifstream& input_file, int directed, int weightedEdge, int weightedNode){

    //Vari�veis para auxiliar na cria��o dos n�s no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;

    //Pegando a ordem do grafo
    input_file >> order;

    //Criando objeto grafo
    Graph* graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo

    if(!graph->getWeightedEdge() && !graph->getWeightedNode()){

        while(input_file >> idNodeSource >> idNodeTarget) {

            graph->insertEdge(idNodeSource, idNodeTarget, 0);

        }

    }else if(graph->getWeightedEdge() && !graph->getWeightedNode() ){

        float edgeWeight;

        while(input_file >> idNodeSource >> idNodeTarget >> edgeWeight) {

            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);

        }

    }else if(graph->getWeightedNode() && !graph->getWeightedEdge()){

        float nodeSourceWeight, nodeTargetWeight;
        while(input_file >> idNodeSource >> idNodeTarget >> nodeSourceWeight >> nodeTargetWeight) {
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);
            graph->insertEdge(idNodeSource, idNodeTarget, 0);

        }

    }else if(graph->getWeightedNode() && graph->getWeightedEdge()){

        float nodeSourceWeight, nodeTargetWeight, edgeWeight;

        while(input_file >> idNodeSource >> idNodeTarget >> nodeSourceWeight >> nodeTargetWeight) {

            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);

        }

    }

    return graph;
}





Graph* leituraInstancia(ifstream& input_file, int directed, int weightedEdge, int weightedNode, ofstream &output_file){

    //Vari�veis para auxiliar na cria��o dos n�s no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;
    int numEdges;
    float weightEdge;
    Graph* graph;
    int weightNode;
    int weightNodeTarget; //fazendo isso pois não estou usando as instancias do stenio
    //Pegando a ordem do grafo

    //getline(input_file) // Pular linha 

  /*  int kluster = 0;
    int ordem = 0;
    string s;

    getline(input_file,s);
    getline(input_file,s);
    getline(input_file,s);

    input_file >> s;
    input_file >> s;
    input_file >> s;
    input_file >> kluster;

    getline(input_file,s);
    input_file >> s;
    input_file >> ordem;

    cout << " kluster : " << kluster << endl; 
    cout << " Ordem do grafo : " << ordem << endl; */

    
    input_file >> order;

    if(weightedEdge && !weightedNode) {
        //Criando objeto grafo
        graph = new Graph(order, directed, weightedEdge, weightedNode);
        //Leitura de arquivo
        //input_file >> "param: " >> variavel
        // getline(input_file) // Pular linha 
        while(input_file >> idNodeSource >> idNodeTarget >> weightEdge) {
            // graph->insertNode(idNodeSource);
            //cout << "IdNodeSource: " << idNodeSource << endl;
            //cout << "IdNodeTarget: " << idNodeTarget << endl;
            graph->insertNode(idNodeSource,0);
            graph->insertNode(idNodeTarget,0);
            graph->insertEdge(idNodeSource, idNodeTarget, weightEdge);
            //output_file << endl;
        }
    } else if(!weightedEdge && weightedNode) {
        //Criando objeto grafo
        graph = new Graph(order, directed, weightedEdge, weightedNode);
        //Leitura de arquivo
        //while(input_file >> idNodeSource >> idNodeTarget >> weightNode) { // comentei essa e fiz o while de baixo pois não estou utilizando as instancias do stenio
        while(input_file >> idNodeSource >> idNodeTarget >> weightNode >> weightNodeTarget) {
            // graph->insertNode(idNodeSource);
            //cout << "IdNodeSource: " << idNodeSource << endl;
            //cout << "IdNodeTarget: " << idNodeTarget << endl;
            graph->insertNode(idNodeSource,weightNode);
            //graph->insertNode(idNodeTarget,weightNode);
            graph->insertNode(idNodeTarget,weightNodeTarget);
            graph->insertEdge(idNodeSource, idNodeTarget, 0);
            //output_file << endl;
        }
    }
    else if(!weightedEdge && !weightedNode) {
        //Criando objeto grafo
        graph = new Graph(order, directed, weightedEdge, weightedNode);
        //Leitura de arquivo
        while(input_file >> idNodeSource >> idNodeTarget) {
            // graph->insertNode(idNodeSource);
            //cout << "IdNodeSource: " << idNodeSource << endl;
            //cout << "IdNodeTarget: " << idNodeTarget << endl;
            graph->insertNode(idNodeSource,0);
            graph->insertNode(idNodeTarget,0);
            graph->insertEdge(idNodeSource, idNodeTarget, 0);
            //output_file << endl;
        }
    } else {
        //Criando objeto grafo
        graph = new Graph(order, directed, weightedEdge, weightedNode);
        //Leitura de arquivo
        while(input_file >> idNodeSource >> idNodeTarget >> weightEdge >> weightNode) {
            // graph->insertNode(idNodeSource);
            //cout << "IdNodeSource: " << idNodeSource << endl;
            //cout << "IdNodeTarget: " << idNodeTarget << endl;
            graph->insertNode(idNodeSource,weightNode);
            graph->insertNode(idNodeTarget,weightNode);
            graph->insertEdge(idNodeSource, idNodeTarget, weightEdge);
            //output_file << endl;
        }
    }
    
    return graph;
}

Graph* leituraInstancia2(ifstream& input_file, int directed, int weightedEdge, int weightedNode, ofstream &output_file){

    Graph *graph;
    int j=0;
    int klusters = 0;
    int ordem;
    string s;
    string s2;
    string s3;
    int node;
    float peso;
    int n1;
    int n2;

    input_file >> s;

    do
    {
        input_file  >> s;
    }while(s != "#");

    input_file >> klusters;
    //output_file << " k : " << klusters << endl;

    do
    {
        input_file  >> s;
    }while(s != "#");

    input_file >> ordem;
    //output_file << " Ordem : " << ordem << endl;
    graph = new Graph(ordem, 0 , 0 , 1);
    do
    {
        input_file  >> s;
    }while(s != ":=");

    do
    {
        input_file  >> s;
    }while(s != ":=");

   for(int i=1;i != ordem+1;i++)
    { 
        input_file >> node;
        input_file >> peso;
        //output_file << " Node " << node << " peso " << peso << endl;
        graph->insertNode(node, peso);        
    }

    do
    {
        input_file >> s;
    } while (s != ":=");
   
    do
    {
        input_file >> s;
        if(s != ";")
        {
            s2 = " ";
            s3 = " ";
            j = 1;
            while(s[j] != ',')
            {
                s2 = s2 + s[j];
                j++;
            }
            j++;
            while(s[j] != ')')
            {
                s3 = s3 + s[j];
                j++;         
            } 
            //cout << " Arestas  " << s << endl;
            n1 = stoi(s2);
            n2 = stoi(s3);
            graph->insertEdge(n1,n2,0);
            //output_file << " Node 1 : " << n1 << " Node 2 : " << n2 << endl;
        }

    } while (s !=  ";");

    //graph = new Graph(ordem, directed, weightedEdge, weightedNode);
    return graph;
}



int menu(){

    int selecao;

    cout << "MENU" << endl;
    cout << "----" << endl;
    /*cout << "[1] Subgrafo induzido pelo fecho transitivo direto "<< endl;
    cout << "[2] Subgrafo induzido pelo fecho transitivo indireto "<< endl;
    cout << "[3] Caminho Minimo entre dois vertices - Dijkstra" << endl;
    cout << "[4] Caminho Minimo entre dois vertices - Floyd" << endl;
    cout << "[5] Arvore Geradora Minima de Prim" << endl;
    cout << "[6] Arvore Geradora Minima de Kruskal" << endl;
    cout << "[7] Arvore dada pela ordem de caminhamento em largura" << endl;
    cout << "[8] Imprimir ordenacao topologica" << endl;*/
    cout << "[1] Algoritmo guloso" << endl;
    cout << "[2] Algoritmo guloso randomizado" << endl;
    cout << "[3] Algoritmo guloso randomizado reativo" << endl;
    cout << "[0] Sair" << endl;

    cin >> selecao;
    cout << "Selecao: " << selecao << endl;
    return selecao;

}

void selecionar(int selecao, Graph* graph, ofstream& output_file){

    switch (selecao) {

           //Sair
        case 0:{

            break;
        }

        /*
           // Subgrafo induzido pelo fecho transitivo direto //
        case 1:{
           //graph->printGraph(output_file);
            int x;
            cout << "Digite o id o noh a ser pesquisado: ";
            cin >> x;
            graph->fechoTransitivoDireto(output_file, x);
           // graph->printGraph(output_file); 
            //graph->printEdge(output_file);//
            break;
        }


            // Subgrafo induzido pelo fecho transitivo indireto //
        case 2:{

            int x;
            cout << "Digite o id o noh a ser pesquisado: ";
            cin >> x;
            graph->fechoTransitivoIndireto(output_file,x);
            break;
        }

            // Caminho Minimo entre dois vertices - Dijkstra //
        case 3:{

            cout<<"Digite o vertice de origem:"<< endl;
            int origem;
            cin>>origem;
            cout<<"Digite o vertice de destino:"<<endl;
            int destino;
            cin>> destino;
            graph->caminhoMin_djkstra(output_file,origem ,destino);

            break;
        }

            // Caminho Minimo entre dois vertices - Floyd //
        case 4:{

            cout<<"Digite o vertice de origem:"<< endl;
            int origem;
            cin>>origem;
            cout<<"Digite o vertice de destino:"<<endl;
            int destino;
            cin>> destino;
            graph->caminhoMin_floyd(output_file,origem,destino);
            break;
        }

            // Arvore Geradora Minima de Prim //
        case 5:{
            Graph *grafoAux = graph->arvGMin_Prim(output_file);
            if(grafoAux == nullptr) {
                cout << "Não é possivel realizar o agm_Prim para grafos direcionados";
            } else {
                grafoAux->printGraph(output_file);
            }
            break;
        }

            // Arvore Geradora Minima de Kruskal //
        case 6:{
            Graph *grafoAux2 = graph->arvGMin_Kruskal(output_file);
            if(grafoAux2 == nullptr) {
                cout << "Não é possivel realizar o agm_Prim para grafos direcionados";
            } else {
                grafoAux2->printGraph(output_file);
            }

            break;
        }
            //Arvore dada pela ordem de caminhamento em largura
        case 7:{ 

            cout<<"Digite o id do vertice:"<< endl;
            int x;
            cin >>x;
            graph->arv_Buscalargura(output_file,x);
            break;
        }
           // Imprimir ordenacao topologica 
        case 8:{

            graph->ord_Topologica(output_file);
         
            break;
        }*/

        case 1:{
            //for(int i=0;i<10;i++) {
                graph->Guloso(output_file, 5);
                //graph->teste(output_file);
                //sleep_for(1000ms);
            //}
            output_file << "saiu" << endl;
            break;
        }

        case 2:{
            graph->GulosoRandomizado(output_file, 5, 0.1, 1000);
            //graph->teste(output_file);
            output_file << "saiu" << endl;
            break;
        }

        case 3:{
            //cout << "Digite a quantidade de alfas: ";
            int qtdAlfas = 3;
            //cin >> qtdAlfas;
            //cout << endl;
            float *vetAlfas = new float[qtdAlfas];
            vetAlfas[0] = 0.1;
            vetAlfas[1] = 0.3;
            vetAlfas[2] = 0.5;
            /*for(int i=0;i<qtdAlfas;i++)
            {
                cout << "Digite o valor do " << i << "o alfa: ";
                cin >> vetAlfas[i];
                cout << endl;
            }*/
            int total, itBloco;
            /*
            cout << "Digite o número de interacoes total: ";
            cin >> total;
            cout << endl;
            cout << "Digite o numero de iteracoes de cada bloco: ";
            cin >> itBloco;
            */
            total = 200;
            itBloco = 10;
            graph->GulosoRandomizadoReativo(output_file, 6, vetAlfas, total, itBloco, qtdAlfas);
            //graph->teste(output_file);
            output_file << "saiu" << endl;
            //delete vetAlfas;
            break;
        }

        default:
        {
            cout << " Error!!! invalid option!!" << endl;
        }

    }
}

int mainMenu(ofstream& output_file, Graph* graph){

    int selecao = 1;

    while(selecao != 0){
        system("clear");
        selecao = menu();

        if(output_file.is_open())
            selecionar(selecao, graph, output_file);

        else
            cout << "Unable to open the output_file" << endl;

        output_file << endl;

    }
    output_file << "Saiu pq?: " << selecao << endl;
    return 0;
}



int main(int argc, char const *argv[]) {
    //Verifica��o se todos os par�metros do programa foram entrados
    if (argc != 6) {

        cout << "ERROR: Expecting: ./<program_name> <input_file> <output_file> <directed> <weighted_edge> <weighted_node> " << endl;
        return 1;

    }

    unsigned seed = time(NULL);
    srand(seed);

    string program_name(argv[0]);
    string input_file_name(argv[1]);

    string instance;
    if(input_file_name.find("v") <= input_file_name.size()){
        string instance = input_file_name.substr(input_file_name.find("v"));
        cout << "Running " << program_name << " with instance " << instance << " ... " << endl;
    }

    //Abrindo arquivo de entrada
    ifstream input_file;
    ofstream output_file;
    input_file.open(argv[1], ios::in);
    output_file.open(argv[2], ios::out | ios::trunc);



    Graph* graph;

    if(input_file.is_open()){
        graph = leituraInstancia2(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), output_file);

    }else
        cout << "Unable to open " << argv[1];


    mainMenu(output_file, graph);



    //Fechando arquivo de entrada
    input_file.close();

    //Fechando arquivo de sa�da
    output_file.close();

    return 0;
}

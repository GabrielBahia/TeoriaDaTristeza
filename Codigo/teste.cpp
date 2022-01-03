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
    cout <<  " node1->getIdNode() : " << node1->getIdNode() << " id : " << id << endl;
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
                cout <<  " auxE->getTargetId()  " << auxE->getTargetId() << endl;
                cout <<  " auxE->getTargetIdNode()  " << (auxE->getTargetIdNode()) << endl;
                cout <<  " aux->getId()  " << aux->getId() << endl;
                listN.push_back(getNode(auxE->getTargetId()));
                cont2++;
                vetAd[cont2] = auxE->getTargetId();
            }

        }
        cout << "SAIU DO FOR " << endl;
     
    }
     cout << "CHEGOU " << endl;
     cout << "CONT2 " << cont2 << endl;

      Graph *arvBL = new Graph(this->order, this->directed, this->weighted_edge, this->weighted_node);

     for(int i=0;i<=cont2;i++)
     {
        cout << " VETAD : " << vetAd[i] << endl;
        arvBL->insertNode(vetAd[i],0); 
     }

    int auxP;
    int auxId;

    for(int i=0;i<=cont2;i++)
    cout << " PA i: " << i << " = " << posicoes[i]<< endl;


     for(int i=1;i<=cont2;i++)
     {
          cout << " ENTROU " << endl;
          
            Node *e = getNode(posicoes[i]);
            for(Edge *x = e->getFirstEdge(); x != nullptr; x = x->getNextEdge())
            {
                if(x->getTargetId() == vetAd[i])
                {
                   auxP = x->getWeight();
                }
            }

            cout << "VERTICE A : " << pa[i] << " VERTICE B : " << i+1 << endl;
            arvBL->insertEdge(posicoes[i],vetAd[i],auxP);

     }

    output_file << "Arvore dada pelo caminhamento em lagura: ";

    arvBL->printGraph(output_file);
 
}
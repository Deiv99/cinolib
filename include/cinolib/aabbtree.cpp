#include "aabbtree.h"
#include <cinolib/how_many_seconds.h>
#include <cinolib/parallel_for.h>
#include <cinolib/geometry/point.h>
#include <cinolib/geometry/sphere.h>
#include <cinolib/geometry/segment.h>
#include <cinolib/geometry/triangle.h>
#include <cinolib/geometry/tetrahedron.h>
#include <stack>
#include <list>

namespace cinolib
{

CINO_INLINE
AABBtreeNode::~AABBtreeNode()
{
    // Distruttore

    for(int i=0; i<2; ++i)
    {
        if(children[i]!=nullptr)
        {
            delete children[i];
        }
    }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
AABBtree::AABBtree()
{}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
AABBtree::~AABBtree()
{
    // delete AABBtree

    if(root!=nullptr) delete root;

    // delete item list
    while(!items.empty())
    {
        delete items.back();
        items.pop_back();
    }
}


//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void AABBtree::build()
{
    typedef std::chrono::high_resolution_clock Time;
    Time::time_point t0 = Time::now();

    // Dobbiamo mettere semplicemnte un solo nodo e vedere se riusciamo ad inglobare l'oggetto

    // Creazione della root, nullptr perchè li ci andrebbe il puntatore al padre, ma essendo la radice non ne abbiamo
    root = new AABBtreeNode(nullptr, AABB());

    // Ridimensioniamo la dimensione del vector di supporto
    root->item_indices.resize(items.size());

    // Riempie il vettore item_indices con dei valori che vanno da 0 incrementandosi
    std::iota(root->item_indices.begin(),root->item_indices.end(),0);

    for(auto it : items) root->bbox.push(it->aabb);

    tree_depth = 1;

    // Dopo aver creato la radice, dobbiamo dividerla
    // Visto che dobbiamo calcolare la mediana del lato più lungo
    // Si prende l'aabb e si vede quale lato è il più lungo
    // Si prende poi l'oggetto e si cercano le coordinate di quel lato e si fa la mediana, dalla mediana si creano i figli

    int flag = findLongestEdge(root->bbox);

    subdivide(root,findMedian(root));


    // Debug Info
    Time::time_point t1 = Time::now();
    double t = how_many_seconds(t0,t1);
    std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
    std::cout << "AABBTree created (" << t << "s)                      " << std::endl;
    std::cout << "#Items                   : " << items.size()         << std::endl;
    std::cout << "#Leaves                  : " << leaves.size()        << std::endl;
    std::cout << "Depth                    : " << tree_depth           << std::endl;
    std::cout << "Flag                     : " << flag                 << std::endl;
    std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;

}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

// Funzione che trova il lato più lungo dell'aabb
CINO_INLINE
int AABBtree::findLongestEdge(AABB box) {

    vec3d min = box.min;
    vec3d max = box.max;

    if((max.x() - min.x()) >= (max.y() - min.y()) && (max.x() - min.x()) >= (max.z() - min.z()))
        return 0; // Asse delle x più lungo
    else if ((max.x() - min.x()) <= (max.y() - min.y()) && (max.y() - min.y()) >= (max.z() - min.z()))
        return 1; // Asse delle y più lungo
    else
        return 2; // Asse delle z più lungo

}

// Funzione che calcola il centroide di un triangolo (non è mai ottimizzato)
CINO_INLINE
vec3d AABBtree::findCentroid (Triangle t) {

    vec3d bar;
    bar.x() = ((t.v[0].x() + t.v[1].x() + t.v[2].x())/3);
    bar.y() = ((t.v[0].y() + t.v[1].y() + t.v[2].y())/3);
    bar.z() = ((t.v[0].z() + t.v[1].z() + t.v[2].z())/3);

    /*
    std::cout << "Baricentro X: " <<  bar.x()     << std::endl;
    std::cout << "Baricentro Y: " <<  bar.y()     << std::endl;
    std::cout << "Baricentro Z: " <<  bar.z()     << std::endl;
    */
    return bar;
}

// Funzione che trova la mediana degli oggetti
CINO_INLINE
double AABBtree::findMedian (AABBtreeNode *node) {

    // So che quando si fa la buildfrommesh, nel vettore item viene inserito il triangolo
    // Quindi in items ho i vertici dei triangoli dalla quale prendere il valore
    // In items indices invece ho gli indici che mi servono di items e del vettore dei triangoli

    // In questo vector salveremo tutte le coordinate dei triangoli rispettive al lato più lungo
    std::vector <double> list;
    int flag = findLongestEdge(node->bbox);
    vec3d baricentro;


    // MEDIANA SUI CENTROIDI
    for (uint it : node->item_indices) {

        baricentro = findCentroid(vecTriangles[it]);

        switch(flag) {
            case 0: // Asse delle x
                list.push_back(baricentro.x());
                break;
            case 1: // Asse delle yù
                list.push_back(baricentro.y());
                break;
            case 2: // Asse delle z
                list.push_back(baricentro.z());
                break;
        }

    }

    std::sort(list.begin(),list.end());

    // Si ritorna il valore in mezzo alla lista ( la precisione in caso di numeri dispari non è essenziale )
    return list[list.size()/2];

    /*
     *
     *  Pezzo di codice basato sui vertici, lo lascio per non perderlo
     *
     *
        for (uint it : node->item_indices) {

            switch(flag) {
                case 0: // Asse delle x
                    list.push_back(vecTriangles[it].v[0].x());
                    list.push_back(vecTriangles[it].v[1].x());
                    list.push_back(vecTriangles[it].v[2].x());
                    break;
                case 1: // Asse delle y
                    list.push_back(vecTriangles[it].v[0].y());
                    list.push_back(vecTriangles[it].v[1].y());
                    list.push_back(vecTriangles[it].v[2].y());
                    break;
                case 2: // Asse delle z
                    list.push_back(vecTriangles[it].v[0].z());
                    list.push_back(vecTriangles[it].v[1].z());
                    list.push_back(vecTriangles[it].v[2].z());
                    break;
            }

        }

        for (uint i = 0; i < list.size(); i++ ) {

            std::cout << "#Valore Numero lista nuova " << i << ":   " <<  list[i]      << std::endl;

        }

        std::cout << "#Mediana nuova: " << list[list.size()/2]      << std::endl;

        */

}

// Funzione che aggiorna la dimensione della bbox in base agli oggetti al suo interno
// Quello che facciamo qui può essere riscritto nel for centrale per evitare cicli inutili
CINO_INLINE
void AABBtree::newBbox (AABBtreeNode *node) {

    vec3d min;
    vec3d max;

    vec3d auxMax;
    vec3d auxMin;

    // Vengono inizializzati a valori impossibili i due vettori importanti
    max.x() = -10;
    max.y() = -10;
    max.z() = -10;

    min.x() = 10;
    min.y() = 10;
    min.z() = 10;

    /*
     * Stampa di aiuto
     *
    std::cout << "#Prima Max x: " << max.x() <<  std::endl;
    std::cout << "#Prima Max y: " << max.y() <<  std::endl;
    std::cout << "#Prima Max z: " << max.z() <<  std::endl;

    std::cout << "#Prima Min x: " << min.x() <<  std::endl;
    std::cout << "#Prima Min y: " << min.y() <<  std::endl;
    std::cout << "#Prima Min z: " << min.z() <<  std::endl;
    */

    for (uint it : node->item_indices) {

        // Molto grezzo bisogna pensare a come farlo meglio
        auxMin = findMinCoordinates(vecTriangles[it]);
        auxMax = findMaxCoordinates(vecTriangles[it]);

        // Si fanno i vari controlli e si aggiornano i dati

        // MIN
        if (min.x() > auxMin.x())
            min.x() = auxMin.x();
        if (min.y() > auxMin.y())
            min.y() = auxMin.y();
        if (min.z() > auxMin.z())
            min.z() = auxMin.z();

        // MAX
        if (max.x() < auxMax.x())
            max.x() = auxMax.x();
        if (max.y() < auxMax.y())
            max.y() = auxMax.y();
        if (max.z() < auxMax.z())
            max.z() = auxMax.z();

        /*
        std::cout << "#Fin Max x: " << max.x() <<  std::endl;
        std::cout << "#Fin Max y: " << max.y() <<  std::endl;
        std::cout << "#Fin Max z: " << max.z() <<  std::endl;

        std::cout << "#Fin Min x: " << min.x() <<  std::endl;
        std::cout << "#Fin Min y: " << min.y() <<  std::endl;
        std::cout << "#Fin Min z: " << min.z() <<  std::endl;
        */

        // Si aggiorna il nuovo bounding box del nodo
        node->bbox = AABB(min,max);

    }

}


CINO_INLINE
void AABBtree::subdivide(AABBtreeNode *node, double median) {

    // Creiamo i due figli
    vec3d min = node->bbox.min;
    vec3d max = node->bbox.max;
    int flag = findLongestEdge(node->bbox);

    if (node->item_indices.size() > 1) {

        // Vengono creati i due figli e le bbox all'inizio sono molto grandi per essere poi ristrette più tardi
        // In base al lato si creano i due figli
        switch(flag) {
            case 0: // Asse delle x
                node->children[0] = new AABBtreeNode(node, AABB(vec3d(min[0], min[1], min[2]), vec3d(median, max[1], max[2])));
                node->children[1] = new AABBtreeNode(node, AABB(vec3d(median, min[1], min[2]), vec3d(max[0], max[1], max[2])));
                break;
            case 1: // Asse delle y
                node->children[0] = new AABBtreeNode(node, AABB(vec3d(min[0], min[1], min[2]), vec3d(max[0], median, max[2])));
                node->children[1] = new AABBtreeNode(node, AABB(vec3d(min[0], median, min[2]), vec3d(max[0], max[1], max[2])));
                break;
            case 2: // Asse delle z
                node->children[0] = new AABBtreeNode(node, AABB(vec3d(min[0], min[1], min[2]), vec3d(max[0], max[1], median)));
                node->children[1] = new AABBtreeNode(node, AABB(vec3d(min[0], min[1], median), vec3d(max[0], max[1], max[2])));
                break;
        }

        for(uint it : node->item_indices)
        {

            for(int i=0; i<2; ++i)
            {
                assert(node->children[i]!=nullptr);

                /*
                    Controllo che il triangolo possa stare sia nel primo figlio che nel secondo
                    Nel caso in cui questa cosa sia possibile, controllo le dimensioni dell'item indices dei figlio e lo metto in quello più piccolo
                    Sennò li inserisco tranquillamente dove stanno
                */

                if (bothChildren(node,it)) {

                    i++; // Saltiamo il for quando entra la prima volta

                    if (differenceSize(node) < 0) {
                        node->children[0]->item_indices.push_back(it);
                    } else {
                        node->children[1]->item_indices.push_back(it);
                    }

                } else {
                    if(node->children[i]->bbox.contains(findCentroid(vecTriangles[it]))) {
                        node->children[i]->item_indices.push_back(it);
                    }
                }
/*
                // Se la bbox contiene il centroide allora si inserisce nella push back
                if(node->children[i]->bbox.contains(findCentroid(vecTriangles[it])))
                {
                    // Bisogna fare un controllo sul fatto che un triangolo sia già da una parte o dall'altra
                    // Per esempio...
                    // Se un baricentro si trova al centro di una divisione di una bounding box
                    // vuol dire che si può trovare dia nel figlio destro che nel figlio sinistro
                    // Quindi se si trova già nel primo figlio non ha senso metterlo nel secondo
                    // Sarebbe anche figo controllare che le size siano simili e quindi mettere un coso da una parte o dall'altra

                    //node->children[i]->item_indices.push_back(it);


                    if (i == 0) {
                        // Primo figlio
                        node->children[i]->item_indices.push_back(it);
                        std::cout << "#Item 0: " << node->children[0]->item_indices.back()         << std::endl;

                    } else {
                        // Secondo figlio
                        // Se l'ultimo elemento del primo figlio è diverso da it, questo vuol dire che it non è nel primo figlio
                        if (node->children[0]->item_indices.size() > 0) {

                            // Se la dimensione di questa differenza è maggiore di uno vuol dire che l'albero non è bilanciato
                            // Potrebbe provocare qualche problema
                            if (!(differenceSize(node) < -1 && differenceSize(node) > 1))

                            if (node->children[0]->item_indices.back() != it) {
                                node->children[1]->item_indices.push_back(it);
                                std::cout << "#Item 1: " << node->children[1]->item_indices.back()         << std::endl;
                            }

                        } else {
                            node->children[i]->item_indices.push_back(it);
                        }

                    }

                }
*/

            } // Fine for dei figli
        } // Fine for di item_indices

        newBbox(node->children[0]);
        newBbox(node->children[1]);


        // A catena tramite ricorsione
        if (node->children[0]->item_indices.size() > 1) {
            tree_depth++;
            subdivide(node->children[0], findMedian(node->children[0]));
        }
        if (node->children[1]->item_indices.size() > 1)
            subdivide(node->children[1], findMedian(node->children[1]));


        // Nell'octree avviene questo... non so se sia utile per risparmiare spazio anche nell'aabbtree
        // Quando dovremo fare le collisioni vedremo...
        //node->item_indices.clear();
    }

}

// Funzione ausiliaria per sapere se un triangolo si trovi in mezzo a due triangoli
CINO_INLINE
bool AABBtree::bothChildren (AABBtreeNode *node, int it) {
    return (node->children[0]->bbox.contains(findCentroid(vecTriangles[it])) && node->children[1]->bbox.contains(findCentroid(vecTriangles[it])));
}

// Funzione ausiliaria per sapere la differenze tra le dimensioni di item indices di due nodi
CINO_INLINE
int AABBtree::differenceSize (AABBtreeNode *node) {
    return node->children[0]->item_indices.size() - node->children[1]->item_indices.size();
}

// Trova le coordinate massime x y z dei tre vertici di un triangolo
CINO_INLINE
vec3d AABBtree::findMaxCoordinates (Triangle t) {

    vec3d max;

    max.x() = -10;
    max.y() = -10;
    max.z() = -10;

    for (int i = 0; i < 3; i++) {

    // MAX
    if (max.x() < t.v[i].x())
        max.x() = t.v[i].x();
    if (max.y() < t.v[i].y())
        max.y() = t.v[i].y();
    if (max.z() < t.v[i].z())
        max.z() = t.v[i].z();

    }
/*
    std::cout << "#Aux Max x: " << max.x() <<  std::endl;
    std::cout << "#Aux Max y: " << max.y() <<  std::endl;
    std::cout << "#Aux Max z: " << max.z() <<  std::endl;
*/
    return max;
}

// Trova le coordinate minime x y z dei tre vertici di un triangolo
CINO_INLINE
vec3d AABBtree::findMinCoordinates (Triangle t) {

    vec3d min;

    min.x() = 10;
    min.y() = 10;
    min.z() = 10;

    for (int i = 0; i < 3; i++) {

    // MIN
    if (min.x() > t.v[i].x())
        min.x() = t.v[i].x();
    if (min.y() > t.v[i].y())
        min.y() = t.v[i].y();
    if (min.z() > t.v[i].z())
        min.z() = t.v[i].z();

    }
/*
    std::cout << "#Aux Min x: " << min.x() <<  std::endl;
    std::cout << "#Aux Min y: " << min.y() <<  std::endl;
    std::cout << "#Aux Min z: " << min.z() <<  std::endl;
*/
    return min;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void AABBtree::push_point(const uint id, const vec3d & v)
{
    items.push_back(new Point(id,v));
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void AABBtree::push_sphere(const uint id, const vec3d & c, const double r)
{
    items.push_back(new Sphere(id,c,r));
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void AABBtree::push_segment(const uint id, const std::vector<vec3d> & v)
{
    items.push_back(new Segment(id,v.data()));
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void AABBtree::push_triangle(const uint id, const std::vector<vec3d> & v)
{
    items.push_back(new Triangle(id,v.data()));
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void AABBtree::push_tetrahedron(const uint id, const std::vector<vec3d> & v)
{
    items.push_back(new Tetrahedron(id,v.data()));
}

} // Fine namespace cinolib

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
Split::Split(AABBtreeNode *node) {

    edge = findLongestEdge(node->bbox);
    // Median non viene inizializzato
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

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

    subdivide(root);

    tree_depth = returnTreeDepth(root);

    // Debug Info
    if(print_debug_info)
    {
        Time::time_point t1 = Time::now();
        double t = how_many_seconds(t0,t1);
        std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
        std::cout << "AABBTree created (" << t << "s)                    " << std::endl;
        std::cout << "#Items                   : " << items.size()         << std::endl;
        std::cout << "#Leaves                  : " << leaves.size()        << std::endl;
        std::cout << "Depth                    : " << tree_depth           << std::endl;
        std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
    }
}

// Nuova versione della subdivide
CINO_INLINE
void AABBtree::subdivide(AABBtreeNode *node) {

    if (node->item_indices.size() > 1) {

        Split *split = new Split(node);
        split->median = findMedian(node,split->edge);

        // Queste due ci serviranno per andare a creare i figli alla fine
        AABB left, right; // AABB del figlio sinistro e destro
        std::vector <uint> vleft, vright; // Item indices del figlio sinistro e figlio destro

        for (uint it : node->item_indices) {

            if (bothChildren(*split, it)) {

                if (differenceSize(vleft,vright) < 0) {
                    left.push(items.at(it)->aabb);
                    vleft.push_back(it);
                } else {
                    right.push(items.at(it)->aabb);
                    vright.push_back(it);
                }

            } else {

                switch (split->edge) {
                case 0: // Asse delle x
                    if (items.at(it)->aabb.center().x() < split->median) {
                        left.push(items.at(it)->aabb);
                        vleft.push_back(it);
                    } else {
                        right.push(items.at(it)->aabb);
                        vright.push_back(it);
                    }
                    break;
                case 1: // Asse delle y
                    if (items.at(it)->aabb.center().y() < split->median) {
                        left.push(items.at(it)->aabb);
                        vleft.push_back(it);
                    } else {
                        right.push(items.at(it)->aabb);
                        vright.push_back(it);
                    }
                    break;
                case 2: // Asse delle z
                    if (items.at(it)->aabb.center().z() < split->median) {
                        left.push(items.at(it)->aabb);
                        vleft.push_back(it);
                    } else {
                        right.push(items.at(it)->aabb);
                        vright.push_back(it);
                    }
                    break;
                }
            }

        }

        assert(!vleft.empty() && !vright.empty());

        node->children[0] = new AABBtreeNode(node, left);
        node->children[1] = new AABBtreeNode(node, right);

        node->children[0]->item_indices = vleft;
        node->children[1]->item_indices = vright;

        // A catena tramite ricorsione
        if (node->children[0]->item_indices.size() > 1) {
            subdivide(node->children[0]);
        }
        if (node->children[1]->item_indices.size() > 1)
            subdivide(node->children[1]);


    } // Fine if iniziale


}

int AABBtree::returnTreeDepth(AABBtreeNode *node) {

    if (node == nullptr) return 0;
    int left = returnTreeDepth(node->children[0]);
    int right = returnTreeDepth(node->children[1]);

    if (left > right)
        return 1 + left;
    else
        return 1 + right;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// FUNZIONI AUSILIARIE AABBTREE

double AABBtree::findMedian(AABBtreeNode *node, int edge) {

    std::vector <double> list;

    // MEDIANA SUI CENTROIDI
    for (uint it : node->item_indices) {

        switch(edge) {
            case 0: // Asse delle x
                list.push_back(items.at(it)->aabb.center().x());
                break;
            case 1: // Asse delle y
                list.push_back(items.at(it)->aabb.center().y());
                break;
            case 2: // Asse delle z
                list.push_back(items.at(it)->aabb.center().z());
                break;
        }

    }

    std::sort(list.begin(),list.end());

    // Si ritorna il valore in mezzo alla lista ( la precisione in caso di numeri dispari non è essenziale )
    return list[list.size()/2];

}

// Funzione ausiliaria per sapere se un triangolo si trovi in mezzo a due triangoli
CINO_INLINE
bool AABBtree::bothChildren (Split split, int it) {

    switch(split.edge) {
        case 0: // Asse delle x
            return split.median == items.at(it)->aabb.center().x();
        case 1: // Asse delle y
            return split.median == items.at(it)->aabb.center().y();
        case 2: // Asse delle z
            return split.median == items.at(it)->aabb.center().z();
    }

    return false;
}

CINO_INLINE
int AABBtree::differenceSize (std::vector<uint> left, std::vector<uint> right) {
    return left.size() - right.size();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// FUNZIONI SPLIT

// Funzione che trova il lato più lungo dell'aabb
CINO_INLINE
int Split::findLongestEdge(AABB box) {

    vec3d min = box.min;
    vec3d max = box.max;

    if((max.x() - min.x()) >= (max.y() - min.y()) && (max.x() - min.x()) >= (max.z() - min.z()))
        return 0; // Asse delle x più lungo
    else if ((max.x() - min.x()) <= (max.y() - min.y()) && (max.y() - min.y()) >= (max.z() - min.z()))
        return 1; // Asse delle y più lungo
    else
        return 2; // Asse delle z più lungo

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

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void AABBtree::debug_mode(const bool b)
{
    print_debug_info = b;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// QUERY

CINO_INLINE
vec3d AABBtree::closest_point(const vec3d & p) const
{
    uint   id;
    vec3d  pos;
    double dist;
    closest_point(p, id, pos, dist);
    return pos;
}

CINO_INLINE
void AABBtree::closest_point(const vec3d  & p,          // query point
                                 uint   & id,         // id of the item T closest to p
                                 vec3d  & pos,        // point in T closest to p
                                 double & dist) const // distance between pos and p
{

    assert(root != nullptr);

    typedef std::chrono::high_resolution_clock Time;
    Time::time_point t0 = Time::now();

    PrioQueue q;
    // Se la root è inner
    if(root->children[0] != nullptr)
    {
        // Si crea un Obj
        Obj obj;
        obj.node = root; // il nodo diventa la root
        obj.dist = root->bbox.dist_sqrd(p); // e si calcola la distanza (?)
        q.push(obj); // e viene pusghata nella coda prioritaria

    }
    else // in case the root is alrady a leaf...
    {

        // Se è una foglia vuol dire che ha un solo item_indices
        assert(root->item_indices.size() == 1);

        Obj obj;
        obj.node  = root;
        obj.index = root->item_indices[0];
        obj.pos   = items.at(root->item_indices[0])->point_closest_to(p);
        obj.dist  = obj.pos.dist_squared(p);
        q.push(obj);

    }

    // Fino a quando il nodo nella coda prioritaria è inner si va (quindi non + una foglia)
    while(q.top().node->children[0] != nullptr)
    {
        // Si prende il top
        Obj obj = q.top();
        q.pop();

        // Si fanno tutti e due i figli
        for(int i=0; i<2; ++i)
        {
            AABBtreeNode *child = obj.node->children[i];
            // Se è inner
            if(child->children[0] != nullptr)
            {
                Obj obj;
                obj.node = child;
                obj.dist = child->bbox.dist_sqrd(p);
                q.push(obj); // SI PUSHA L'obj nella coda
            }
            // Se è una foglia
            else
            {
                // Se è una foglia vuol dire che ha un solo item_indices
                assert(child->item_indices.size() == 1);

                Obj obj;
                obj.node  = child;
                obj.index = child->item_indices[0];
                obj.pos   = items.at(child->item_indices[0])->point_closest_to(p);
                obj.dist  = obj.pos.dist_squared(p);
                q.push(obj);

            }
        }
    }

    if(print_debug_info)
    {
        Time::time_point t1 = Time::now();
        std::cout << "Closest point\t" << how_many_seconds(t0,t1) << " seconds" << std::endl;
    }

    // Si arriva che al top della coda si ha una foglia e vengono salvate le sue informazioni, l'item rimasto alla fine è il punto più vicino
    assert(q.top().index>=0);
    id   = items.at(q.top().index)->id;
    pos  = q.top().pos;
    dist = q.top().dist;

}

// Qui stampiamo ogni volta l'id con relativa distanza per vedere se il risultato ufficiale combacia con uno di quelli ottenuti
// perchè in molti casi ci potrebbe essere più di un punto distante lo stesso tanto da un altro punto e quinndi ci possono essere più risultati giusti
CINO_INLINE
void AABBtree::provaClosest(const vec3d & p,  uint & id, vec3d  & pos, double & dist) {

    assert(root != nullptr);

    std::cout << "############## PROVA CLOSEST POINT ###############"<< std::endl;

    typedef std::chrono::high_resolution_clock Time;
    Time::time_point t0 = Time::now();

    dist = max_double;
    id = -10;

    // Variabili ausiliarie
    double aux;
    vec3d auxPos;
    int cont = 0;

    std::stack<AABBtreeNode*> lifo;

    if(root->children[0] != nullptr) {
        lifo.push(root);

    } else {

        // Se è una foglia vuol dire che ha un solo item_indices
        assert(root->item_indices.size() == 1);

        // Calcolare la distanza
        pos = items.at(root->item_indices[0])->point_closest_to(p);
        dist = pos.dist_squared(p);
        id = root->item_indices[0];

        // Stampa per vedere
        std::cout << "Closest Point trovato nella Radice: Dist: " << dist << " Id: " << id << std::endl;
        return;
    }

    while(!lifo.empty()) {

        AABBtreeNode *node = lifo.top();
        lifo.pop();

        if(node->children[0] != nullptr) {
            for(int i=0; i<2; ++i)
            {
                lifo.push(node->children[i]);
            }
        } else {
            // Se siamo una foglia
            assert(node->item_indices.size() == 1);
            cont++;
            auxPos = items.at(node->item_indices[0])->point_closest_to(p);
            aux = auxPos.dist_squared(p);

            // Se la distanza appena ottenuta è minore o uguale si aggiornano e si stampano
            if (aux <= dist) {
                pos = auxPos;
                dist = aux;
                id = node->item_indices[0];

                // Qui si stampa la distanza che si è appena aggiunta e l'id dell'item che possiede
                std::cout << "Closest Point trovato nell'albero: Dist: " << dist << " Id: " << id << std::endl;
            }

        }
    }

    Time::time_point t1 = Time::now();
    std::cout << "Closest point Tempo Prova\t" << how_many_seconds(t0,t1) << " seconds" << std::endl;

    std::cout << "CONTATORE: " << cont <<  std::endl;
}

// this query becomes exact if CINOLIB_USES_EXACT_PREDICATES is defined
CINO_INLINE
bool AABBtree::contains(const vec3d & p, const bool strict, uint & id) const
{

    // Dichiarazione per il tempo
    typedef std::chrono::high_resolution_clock Time;
    Time::time_point t0 = Time::now();

    // Viene creato uno stack di nodi
    std::stack<AABBtreeNode*> lifo;
    if(root && root->bbox.contains(p,strict))
    {
        // Viene inserita la radice
        lifo.push(root);
    }

    while(!lifo.empty())
    {
        // Il nodo prende la testa
        AABBtreeNode *node = lifo.top();
        lifo.pop();
        assert(node->bbox.contains(p, strict));

        // Se il nodo è interno
        // non dovrebbe esserci bisogno di controllare anche il secondo figlio, perchè il controllo viene fatto nella costruzione dell'albero
        if(node->children[0] != nullptr)
        {
            for(int i=0; i<2; ++i)
            {
                // Si fa un controllo sui 2 figli, nel vedere se il figlio contiene il vec3d dato
                // Nel caso in cui questo succeda, viene pushato quel figlio nello stack
                if(node->children[i]->bbox.contains(p,strict)) lifo.push(node->children[i]);
            }
        }
        else // Questo else dovrebbe ssere se il nodo è una foglia
        {

            // Si controlla per ogni item_indices che contiene quel nodo (nel nostro caso, un nodo foglia contiene un solo item_indices)
            for(uint i : node->item_indices)
            {
                // Se quell'item contiene il vec3d p
                if(items.at(i)->contains(p,strict))
                {
                    // L'id prende il valore di quel item
                    id = items.at(i)->id;
                    // Viene stampato il tempo e si ritorna true
                    if(print_debug_info)
                    {
                        Time::time_point t1 = Time::now();
                        std::cout << "Contains query (first item)\t" << how_many_seconds(t0,t1) << " seconds" << std::endl;
                    }
                    return true;
                }
            }
        }
    }

    return false;

}

// this query becomes exact if CINOLIB_USES_EXACT_PREDICATES is defined
CINO_INLINE
bool AABBtree::contains(const vec3d & p, const bool strict, std::unordered_set<uint> & ids) const
{

    typedef std::chrono::high_resolution_clock Time;
    Time::time_point t0 = Time::now();

    // ids viene pulito
    ids.clear();

    std::stack<AABBtreeNode*> lifo;
    if(root && root->bbox.contains(p,strict))
    {
        lifo.push(root);
    }

    while(!lifo.empty())
    {
        AABBtreeNode *node = lifo.top();
        lifo.pop();
        assert(node->bbox.contains(p,strict));

        if(node->children[0] != nullptr)
        {
            for(int i=0; i<2; ++i)
            {
                if(node->children[i]->bbox.contains(p,strict)) lifo.push(node->children[i]);
            }
        }
        else
        {
            for(uint i : node->item_indices)
            {
                // Se la foglia contiene il vec3d allora in ids viene inserito l'id
                if(items.at(i)->contains(p,strict))
                {
                    ids.insert(items.at(i)->id);
                }
            }
        }
    }

    if(print_debug_info)
    {
        Time::time_point t1 = Time::now();
        std::cout << "Contains query (all items)\t" << how_many_seconds(t0,t1) << " seconds" << std::endl;
    }

    // Si ritorna se l'ids è vuoto o no, se lo è vuol dire che il vec3d non è contenuto
    return !ids.empty();

}

} // Fine namespace cinolib

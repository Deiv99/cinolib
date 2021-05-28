#ifndef AABBTREE_H
#define AABBTREE_H

#include <cinolib/geometry/spatial_data_structure_item.h>
#include <cinolib/meshes/meshes.h>
#include <queue> // Probabilmente a noi non servirà

namespace cinolib {

class AABBtreeNode {

    public:
        AABBtreeNode(const AABBtreeNode *father, const AABB & bbox) : father(father), bbox(bbox) {};

        ~AABBtreeNode(); // Distruttore

        const AABBtreeNode  *father; // Puntatore al padre
        AABBtreeNode        *children[2] = {nullptr, nullptr}; // Figli
        AABB                bbox;
        std::vector<uint>   item_indices; // Struttura di supporto a cosa serve esattamente?

        // Commento di item_indices originale:
        // index Octree::items, avoiding to store a copy of the same object multiple times in each node it appears
};

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

/* Appunto del professore da controllare ovviamente su come creare un octree
 *
 *  Usage:
 *  i)   Create an empty octree
 *  ii)  Use the push_segment/triangle/tetrahedron facilities to populate it
 *  iii) Call build to make the tree
*/

class AABBtree
{
    public:

    explicit AABBtree();

    virtual ~AABBtree(); // Distruttore

    // DA QUA BISOGNA CONTROLLARE COSA CI INTERESSA E COSA NO

    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    void push_point      (const uint id, const vec3d & v);
    void push_sphere     (const uint id, const vec3d & c, const double r);
    void push_segment    (const uint id, const std::vector<vec3d> & v);
    void push_triangle   (const uint id, const std::vector<vec3d> & v);
    void push_tetrahedron(const uint id, const std::vector<vec3d> & v);

    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    void build();

    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    int findLongestEdge(AABB box);
    void returnListCoordinates(AABBtreeNode *node, double * list);
    double findMedian (AABBtreeNode *node);
    void subdivide(AABBtreeNode *node, double median);
    vec3d findCentroid (Triangle t);

    void newBbox (AABBtreeNode *node);

    // Funzioni ausiliarie
    int differenceSize (AABBtreeNode *node);
    bool bothChildren (AABBtreeNode *node, int it);
    vec3d findMaxCoordinates (Triangle t);
    vec3d findMinCoordinates (Triangle t);

    // BUILD ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


    template<class M, class V, class E, class P>
    void build_from_mesh_polys(const AbstractPolygonMesh<M,V,E,P> & m)
    {
        assert(items.empty());
        items.reserve(m.num_polys());
        vecTriangles.reserve(m.num_polys());
        for(uint pid=0; pid<m.num_polys(); ++pid)
        {
            for(uint i=0; i<m.poly_tessellation(pid).size()/3; ++i)
            {
                vec3d v0 = m.vert(m.poly_tessellation(pid).at(3*i+0));
                vec3d v1 = m.vert(m.poly_tessellation(pid).at(3*i+1));
                vec3d v2 = m.vert(m.poly_tessellation(pid).at(3*i+2));

                // Salvo i triangoli nel mio vector
                vecTriangles[pid].id = pid;
                vecTriangles[pid].v[0] = v0;
                vecTriangles[pid].v[1] = v1;
                vecTriangles[pid].v[2] = v2;

                push_triangle(pid, {v0,v1,v2});
            }
        }

        build();
    }
/*
    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    template<class M, class V, class E, class P>
    void build_from_mesh_polys(const AbstractPolyhedralMesh<M,V,E,P> & m)
    {
            assert(items.empty());
            items.reserve(m.num_polys());
            for(uint pid=0; pid<m.num_polys(); ++pid)
            {
                switch(m.mesh_type())
                {
                    case TETMESH : push_tetrahedron(pid, m.poly_verts(pid)); break;
                    default: assert(false && "Unsupported element");
                }
            }
            build();
    }

    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    void build_from_vectors(const std::vector<vec3d> & verts,
                            const std::vector<uint>  & tris)
    {
        assert(items.empty());
        items.reserve(tris.size()/3);
        for(uint i=0; i<tris.size(); i+=3)
        {
            push_triangle(i/3, { verts.at(tris.at(i  )),
                                verts.at(tris.at(i+1)),
                                verts.at(tris.at(i+2))});
        }
        build();
    }

    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    template<class M, class V, class E, class P>
    void build_from_mesh_edges(const AbstractMesh<M,V,E,P> & m)
    {
        assert(items.empty());
        items.reserve(m.num_edges());
        for(uint eid=0; eid<m.num_edges(); ++eid)
        {
            push_segment(eid, m.edge_verts(eid));
        }
        build();
    }


*/
    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    void debug_mode(const bool b);

    // QUERIES :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    /*
    // returns pos, id and distance of the item that is closest to query point p
    void  closest_point(const vec3d & p, uint & id, vec3d & pos, double & dist) const;
    vec3d closest_point(const vec3d & p) const;

    // returns respectively the first item and the full list of items containing query point p
    // note: this query becomes exact if CINOLIB_USES_EXACT_PREDICATES is defined
    bool contains(const vec3d & p, const bool strict, uint & id) const;
    bool contains(const vec3d & p, const bool strict, std::unordered_set<uint> & ids) const;

    // returns respectively the first and the full list of intersections
    // between items in the octree and a ray R(t) := p + t * dir
    bool intersects_ray(const vec3d & p, const vec3d & dir, double & min_t, uint & id) const; // first hit
    bool intersects_ray(const vec3d & p, const vec3d & dir, std::set<std::pair<double,uint>> & all_hits) const;

    // note: the first two queries become exact if CINOLIB_USES_EXACT_PREDICATES is defined
    // (intersect_box DOES NOT BECOME exact)
    bool intersects_segment (const vec3d s[], const bool ignore_if_valid_complex, std::unordered_set<uint> & ids) const;
    bool intersects_triangle(const vec3d t[], const bool ignore_if_valid_complex, std::unordered_set<uint> & ids) const;
    bool intersects_box     (const AABB  & b, std::unordered_set<uint> & ids) const;

    */

    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    // Questi dati bisogna capire quando servono e per cosa possono tornarci utili

    // all items live here, and leaf nodes only store indices to items
    std::vector<Triangle> vecTriangles;
    std::vector<SpatialDataStructureItem*>  items;
    AABBtreeNode                            *root = nullptr;
    std::vector<const AABBtreeNode*>        leaves;

    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    protected:

    uint tree_depth = 0; // actual depth of the tree
    bool print_debug_info = false;

    // SUPPORT STRUCTURES ::::::::::::::::::::::::::::::::::::::::::::::::::::

    // Queste strutture di supporto verranno utilizzate ?

    /* La struct Obj viene utilizzata in closest point
     * e intersect ray che sono cose che vedremo tra un bel po'
     * Greater e la typedef invece si basano sempre su Obj quindi non
     * ha senso tenerle attive anche per noi */

    /*
    struct Obj
    {
        double      dist  = inf_double;
        AABBtreeNode *node  = nullptr;
        int         index = -1; // note: this is the item ID, NOT necessarily the index of vector items!!
        vec3d       pos;        // closest point
    };

    struct Greater
    {
        bool operator()(const Obj & obj1, const Obj & obj2)
        {
            return obj1.dist > obj2.dist;
        }
    };

    typedef std::priority_queue<Obj,std::vector<Obj>,Greater> PrioQueue;
    */
};

}

#ifndef  CINO_STATIC_LIB
#include "aabbtree.cpp"
#endif

#endif // AABBTREE_H

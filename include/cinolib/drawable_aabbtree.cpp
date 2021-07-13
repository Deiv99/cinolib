#include "drawable_aabbtree.h"

#ifdef __APPLE__
#include <gl.h>
#include <glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

namespace cinolib {

CINO_INLINE
DrawableAABBtree::DrawableAABBtree() : AABBtree()
{
    updateGL();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void DrawableAABBtree::draw(const float) const
{
    for(auto obj : render_list) obj.draw();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
vec3d DrawableAABBtree::scene_center() const
{
    if(this->root==nullptr) return vec3d(0,0,0);
    return this->root->bbox.center();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
float DrawableAABBtree::scene_radius() const
{
    if(this->root==nullptr) return 0.0;
    return this->root->bbox.diag();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void DrawableAABBtree::updateGL()
{
    render_list.clear();
    if(this->root==nullptr) return;
    updateGL(this->root);
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void DrawableAABBtree::updateGL(const AABBtreeNode *node)
{

    // In questo modo solo i nodi foglie vengono rappresentati nell'aabbtrre
/*
    if(node->children[0] != nullptr && node->children[1] != nullptr)
    {
        for(int i=0; i<2; ++i) updateGL(node->children[i]);
    } else {
        //Se non ha figli ed è quindi una foglia viene stampata
        if (node->item_indices.size() == 1) {
            render_list.push_back(DrawableAABB(node->bbox.min, node->bbox.max));
        }
    }
*/
    // Se volessi stampare tutto l'albero

    render_list.push_back(DrawableAABB(node->bbox.min, node->bbox.max));
    if(node->children[0] != nullptr && node->children[1] != nullptr)
    {
        for(int i=0; i<2; ++i) updateGL(node->children[i]);
    }

}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void DrawableAABBtree::set_color(const Color & c)
{
    for(auto & obj : render_list) obj.set_color(c);
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CINO_INLINE
void DrawableAABBtree::set_thickness(float t)
{
    for(auto & obj : render_list) obj.set_thickness(t);
}



} // Fine cinolib

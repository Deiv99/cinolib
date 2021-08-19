#ifndef DRAWABLE_AABBTREE_H
#define DRAWABLE_AABBTREE_H

#include <cinolib/drawable_object.h>
#include <cinolib/geometry/drawable_aabb.h>
#include <cinolib/aabbtree.h>

namespace cinolib {


class DrawableAABBtree : public AABBtree, public DrawableObject
{
    public:

        explicit DrawableAABBtree();

        ~DrawableAABBtree() {}

        //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

        void  draw(const float scene_size=1) const;

        vec3d scene_center() const;
        float scene_radius() const;

        //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

        ObjectType object_type() const { return DRAWABLE_CURVE; }

        //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

        void updateGL();

        void updateGL(const AABBtreeNode * node);

        //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

        void set_color(const Color & c);
        void set_thickness(float t);

    private:

        std::vector<DrawableAABB> render_list;

};

}

#ifndef  CINO_STATIC_LIB
#include "drawable_aabbtree.cpp"
#endif

#endif // DRAWABLE_AABBTREE_H

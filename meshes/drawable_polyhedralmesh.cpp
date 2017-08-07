/*********************************************************************************
*  Copyright(C) 2016: Marco Livesu                                               *
*  All rights reserved.                                                          *
*                                                                                *
*  This file is part of CinoLib                                                  *
*                                                                                *
*  CinoLib is dual-licensed:                                                     *
*                                                                                *
*   - For non-commercial use you can redistribute it and/or modify it under the  *
*     terms of the GNU General Public License as published by the Free Software  *
*     Foundation; either version 3 of the License, or (at your option) any later *
*     version.                                                                   *
*                                                                                *
*   - If you wish to use it as part of a commercial software, a proper agreement *
*     with the Author(s) must be reached, based on a proper licensing contract.  *
*                                                                                *
*  This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE       *
*  WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.     *
*                                                                                *
*  Author(s):                                                                    *
*                                                                                *
*     Marco Livesu (marco.livesu@gmail.com)                                      *
*     http://pers.ge.imati.cnr.it/livesu/                                        *
*                                                                                *
*     Italian National Research Council (CNR)                                    *
*     Institute for Applied Mathematics and Information Technologies (IMATI)     *
*     Via de Marini, 6                                                           *
*     16149 Genoa,                                                               *
*     Italy                                                                      *
**********************************************************************************/
#include <cinolib/meshes/drawable_polyhedralmesh.h>
#include <cinolib/textures/textures.h>
#include <cinolib/color.h>


namespace cinolib
{

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class C>
CINO_INLINE
DrawablePolyhedralmesh<M,V,E,F,C>::DrawablePolyhedralmesh() : Polyhedralmesh<M,V,E,F,C>()
{
    init_drawable_stuff();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class C>
CINO_INLINE
DrawablePolyhedralmesh<M,V,E,F,C>::DrawablePolyhedralmesh(const char * filename) : Polyhedralmesh<M,V,E,F,C>(filename)
{
    init_drawable_stuff();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class C>
CINO_INLINE
DrawablePolyhedralmesh<M,V,E,F,C>::DrawablePolyhedralmesh(const std::vector<vec3d>             & verts,
                                                          const std::vector<std::vector<uint>> & faces,
                                                          const std::vector<std::vector<uint>> & polys,
                                                          const std::vector<std::vector<bool>> & polys_face_winding)
    : Polyhedralmesh<M,V,E,F,C>(verts, faces, polys, polys_face_winding)
{
    init_drawable_stuff();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::init_drawable_stuff()
{
    slicer = MeshSlicer<DrawablePolyhedralmesh<M,V,E,F,P>>(*this);

    drawlist_in.draw_mode  = DRAW_TRIS | DRAW_TRI_FLAT | DRAW_TRI_FACECOLOR | DRAW_SEGS | DRAW_MARKED_SEGS;
    drawlist_out.draw_mode = DRAW_TRIS | DRAW_TRI_FLAT | DRAW_TRI_FACECOLOR | DRAW_SEGS | DRAW_MARKED_SEGS;

    updateGL();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::draw(const float) const
{
    render(drawlist_in );
    render(drawlist_out);
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::updateGL()
{
    updateGL_out();
    updateGL_in();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::updateGL_out()
{
    drawlist_out.tris.clear();
    drawlist_out.tri_coords.clear();
    drawlist_out.tri_v_norms.clear();
    drawlist_out.tri_v_colors.clear();
    drawlist_out.tri_text.clear();
    drawlist_out.segs.clear();
    drawlist_out.seg_coords.clear();
    drawlist_out.seg_colors.clear();
    drawlist_out.marked_segs.clear();
    drawlist_out.marked_seg_coords.clear();

    for(uint fid=0; fid<this->num_faces(); ++fid)
    {
        if (!this->face_is_on_srf(fid)) continue;

        assert(this->adj_f2p(fid).size()==1);
        uint pid = this->adj_f2p(fid).front();

        if (!(this->poly_data(pid).visible)) continue;

        for(uint i=0; i<this->face_tessellation(fid).size()/3; ++i)
        {
            uint vid0 = this->face_tessellation(fid).at(3*i+0);
            uint vid1 = this->face_tessellation(fid).at(3*i+1);
            uint vid2 = this->face_tessellation(fid).at(3*i+2);

            int base_addr = drawlist_out.tri_coords.size()/3;

            drawlist_out.tris.push_back(base_addr    );
            drawlist_out.tris.push_back(base_addr + 1);
            drawlist_out.tris.push_back(base_addr + 2);

            drawlist_out.tri_coords.push_back(this->vert(vid0).x());
            drawlist_out.tri_coords.push_back(this->vert(vid0).y());
            drawlist_out.tri_coords.push_back(this->vert(vid0).z());
            drawlist_out.tri_coords.push_back(this->vert(vid1).x());
            drawlist_out.tri_coords.push_back(this->vert(vid1).y());
            drawlist_out.tri_coords.push_back(this->vert(vid1).z());
            drawlist_out.tri_coords.push_back(this->vert(vid2).x());
            drawlist_out.tri_coords.push_back(this->vert(vid2).y());
            drawlist_out.tri_coords.push_back(this->vert(vid2).z());

            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.x());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.y());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.z());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.x());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.y());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.z());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.x());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.y());
            drawlist_out.tri_v_norms.push_back(this->face_data(fid).normal.z());

            if (drawlist_out.draw_mode & DRAW_TRI_TEXTURE1D)
            {
                drawlist_out.tri_text.push_back(this->vert_data(vid0).uvw[0]);
                drawlist_out.tri_text.push_back(this->vert_data(vid1).uvw[0]);
                drawlist_out.tri_text.push_back(this->vert_data(vid2).uvw[0]);
            }
            else if (drawlist_out.draw_mode & DRAW_TRI_TEXTURE2D)
            {
                drawlist_out.tri_text.push_back(this->vert_data(vid0).uvw[0]*drawlist_out.tri_text_unit_scalar);
                drawlist_out.tri_text.push_back(this->vert_data(vid0).uvw[1]*drawlist_out.tri_text_unit_scalar);
                drawlist_out.tri_text.push_back(this->vert_data(vid1).uvw[0]*drawlist_out.tri_text_unit_scalar);
                drawlist_out.tri_text.push_back(this->vert_data(vid1).uvw[1]*drawlist_out.tri_text_unit_scalar);
                drawlist_out.tri_text.push_back(this->vert_data(vid2).uvw[0]*drawlist_out.tri_text_unit_scalar);
                drawlist_out.tri_text.push_back(this->vert_data(vid2).uvw[1]*drawlist_out.tri_text_unit_scalar);
            }

            if (drawlist_out.draw_mode & DRAW_TRI_FACECOLOR) // replicate f color on each vertex
            {
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.r);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.g);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.b);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.a);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.r);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.g);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.b);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.a);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.r);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.g);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.b);
                drawlist_out.tri_v_colors.push_back(this->poly_data(pid).color.a);
            }
            else if (drawlist_out.draw_mode & DRAW_TRI_VERTCOLOR)
            {
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid0).color.r);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid0).color.g);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid0).color.b);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid0).color.a);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid1).color.r);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid1).color.g);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid1).color.b);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid1).color.a);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid2).color.r);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid2).color.g);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid2).color.b);
                drawlist_out.tri_v_colors.push_back(this->vert_data(vid2).color.a);
            }
            else if (drawlist_out.draw_mode & DRAW_TRI_QUALITY)
            {
                float q = this->poly_data(pid).quality;
                Color c = Color::quality2rgb(q);
                drawlist_out.tri_v_colors.push_back(c.r);
                drawlist_out.tri_v_colors.push_back(c.g);
                drawlist_out.tri_v_colors.push_back(c.b);
                drawlist_out.tri_v_colors.push_back(c.a);
                drawlist_out.tri_v_colors.push_back(c.r);
                drawlist_out.tri_v_colors.push_back(c.g);
                drawlist_out.tri_v_colors.push_back(c.b);
                drawlist_out.tri_v_colors.push_back(c.a);
                drawlist_out.tri_v_colors.push_back(c.r);
                drawlist_out.tri_v_colors.push_back(c.g);
                drawlist_out.tri_v_colors.push_back(c.b);
                drawlist_out.tri_v_colors.push_back(c.a);
            }
        }
    }

    for(uint eid=0; eid<this->num_edges(); ++eid)
    {
        if (!this->edge_is_on_srf(eid)) continue;

        bool invisible = true;
        for(uint cid : this->adj_e2p(eid))
        {
            if (this->poly_data(cid).visible) invisible = false;
        }
        if (invisible) continue;

        int base_addr = drawlist_out.seg_coords.size()/3;
        drawlist_out.segs.push_back(base_addr    );
        drawlist_out.segs.push_back(base_addr + 1);

        vec3d vid0 = this->edge_vert(eid,0);
        vec3d vid1 = this->edge_vert(eid,1);

        drawlist_out.seg_coords.push_back(vid0.x());
        drawlist_out.seg_coords.push_back(vid0.y());
        drawlist_out.seg_coords.push_back(vid0.z());
        drawlist_out.seg_coords.push_back(vid1.x());
        drawlist_out.seg_coords.push_back(vid1.y());
        drawlist_out.seg_coords.push_back(vid1.z());

        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.r);
        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.g);
        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.b);
        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.a);
        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.r);
        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.g);
        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.b);
        drawlist_out.seg_colors.push_back(this->edge_data(eid).color.a);

        if (this->edge_data(eid).marked && drawlist_out.draw_mode & DRAW_MARKED_SEGS)
        {
            int base_addr = drawlist_out.marked_seg_coords.size()/3;
            drawlist_out.marked_segs.push_back(base_addr    );
            drawlist_out.marked_segs.push_back(base_addr + 1);

            drawlist_out.marked_seg_coords.push_back(vid0.x());
            drawlist_out.marked_seg_coords.push_back(vid0.y());
            drawlist_out.marked_seg_coords.push_back(vid0.z());
            drawlist_out.marked_seg_coords.push_back(vid1.x());
            drawlist_out.marked_seg_coords.push_back(vid1.y());
            drawlist_out.marked_seg_coords.push_back(vid1.z());
        }
    }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::updateGL_in()
{
    drawlist_in.tris.clear();
    drawlist_in.tri_coords.clear();
    drawlist_in.tri_v_norms.clear();
    drawlist_in.tri_v_colors.clear();
    drawlist_in.tri_text.clear();
    drawlist_in.segs.clear();
    drawlist_in.seg_coords.clear();
    drawlist_in.seg_colors.clear();
    drawlist_in.marked_segs.clear();
    drawlist_in.marked_seg_coords.clear();

    std::set<uint> edges_to_render;

    for(uint fid=0; fid<this->num_faces(); ++fid)
    {
        if (this->face_is_on_srf(fid)) continue;

        assert(this->adj_f2p(fid).size()==2);
        std::vector<uint> visible_polys;
        for(uint pid : this->adj_f2p(fid))
        {
            if (this->poly_data(pid).visible) visible_polys.push_back(pid);
        }
        if (visible_polys.size()!=1) continue;

        uint pid   = visible_polys.front();
        bool is_CW = this->poly_face_is_CW(pid,this->poly_face_offset(pid,fid));

        for(uint eid : this->adj_f2e(fid))
        {
            if (this->edge_is_on_srf(eid)) continue; // updateGL_out() will consider it
            edges_to_render.insert(eid);
        }

        for(uint i=0; i<this->face_tessellation(fid).size()/3; ++i)
        {
            uint vid0 = this->face_tessellation(fid).at(3*i+0);
            uint vid1 = this->face_tessellation(fid).at(3*i+1);
            uint vid2 = this->face_tessellation(fid).at(3*i+2);
            if (is_CW) std::swap(vid1,vid2); // flip triangle orientation

            int base_addr = drawlist_in.tri_coords.size()/3;

            drawlist_in.tris.push_back(base_addr    );
            drawlist_in.tris.push_back(base_addr + 1);
            drawlist_in.tris.push_back(base_addr + 2);

            drawlist_in.tri_coords.push_back(this->vert(vid0).x());
            drawlist_in.tri_coords.push_back(this->vert(vid0).y());
            drawlist_in.tri_coords.push_back(this->vert(vid0).z());
            drawlist_in.tri_coords.push_back(this->vert(vid1).x());
            drawlist_in.tri_coords.push_back(this->vert(vid1).y());
            drawlist_in.tri_coords.push_back(this->vert(vid1).z());
            drawlist_in.tri_coords.push_back(this->vert(vid2).x());
            drawlist_in.tri_coords.push_back(this->vert(vid2).y());
            drawlist_in.tri_coords.push_back(this->vert(vid2).z());

            vec3d n = (is_CW) ? -this->face_data(fid).normal : this->face_data(fid).normal;
            drawlist_in.tri_v_norms.push_back(n.x());
            drawlist_in.tri_v_norms.push_back(n.y());
            drawlist_in.tri_v_norms.push_back(n.z());
            drawlist_in.tri_v_norms.push_back(n.x());
            drawlist_in.tri_v_norms.push_back(n.y());
            drawlist_in.tri_v_norms.push_back(n.z());
            drawlist_in.tri_v_norms.push_back(n.x());
            drawlist_in.tri_v_norms.push_back(n.y());
            drawlist_in.tri_v_norms.push_back(n.z());

            if (drawlist_in.draw_mode & DRAW_TRI_TEXTURE1D)
            {
                drawlist_in.tri_text.push_back(this->vert_data(vid0).uvw[0]);
                drawlist_in.tri_text.push_back(this->vert_data(vid1).uvw[0]);
                drawlist_in.tri_text.push_back(this->vert_data(vid2).uvw[0]);
            }
            else if (drawlist_in.draw_mode & DRAW_TRI_TEXTURE2D)
            {
                drawlist_in.tri_text.push_back(this->vert_data(vid0).uvw[0]*drawlist_in.tri_text_unit_scalar);
                drawlist_in.tri_text.push_back(this->vert_data(vid0).uvw[1]*drawlist_in.tri_text_unit_scalar);
                drawlist_in.tri_text.push_back(this->vert_data(vid1).uvw[0]*drawlist_in.tri_text_unit_scalar);
                drawlist_in.tri_text.push_back(this->vert_data(vid1).uvw[1]*drawlist_in.tri_text_unit_scalar);
                drawlist_in.tri_text.push_back(this->vert_data(vid2).uvw[0]*drawlist_in.tri_text_unit_scalar);
                drawlist_in.tri_text.push_back(this->vert_data(vid2).uvw[1]*drawlist_in.tri_text_unit_scalar);
            }

            if (drawlist_in.draw_mode & DRAW_TRI_FACECOLOR) // replicate f color on each vertex
            {
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.r);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.g);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.b);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.a);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.r);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.g);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.b);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.a);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.r);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.g);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.b);
                drawlist_in.tri_v_colors.push_back(this->poly_data(pid).color.a);
            }
            else if (drawlist_in.draw_mode & DRAW_TRI_VERTCOLOR)
            {
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid0).color.r);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid0).color.g);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid0).color.b);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid0).color.a);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid1).color.r);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid1).color.g);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid1).color.b);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid1).color.a);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid2).color.r);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid2).color.g);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid2).color.b);
                drawlist_in.tri_v_colors.push_back(this->vert_data(vid2).color.a);
            }
            else if (drawlist_in.draw_mode & DRAW_TRI_QUALITY)
            {
                float q = this->poly_data(pid).quality;
                Color c = Color::quality2rgb(q);
                drawlist_in.tri_v_colors.push_back(c.r);
                drawlist_in.tri_v_colors.push_back(c.g);
                drawlist_in.tri_v_colors.push_back(c.b);
                drawlist_in.tri_v_colors.push_back(c.a);
                drawlist_in.tri_v_colors.push_back(c.r);
                drawlist_in.tri_v_colors.push_back(c.g);
                drawlist_in.tri_v_colors.push_back(c.b);
                drawlist_in.tri_v_colors.push_back(c.a);
                drawlist_in.tri_v_colors.push_back(c.r);
                drawlist_in.tri_v_colors.push_back(c.g);
                drawlist_in.tri_v_colors.push_back(c.b);
                drawlist_in.tri_v_colors.push_back(c.a);
            }
        }
    }

    for(uint eid : edges_to_render)
    {
        uint base_addr = drawlist_in.seg_coords.size()/3;
        drawlist_in.segs.push_back(base_addr    );
        drawlist_in.segs.push_back(base_addr + 1);

        vec3d vid0 = this->edge_vert(eid,0);
        vec3d vid1 = this->edge_vert(eid,1);

        drawlist_in.seg_coords.push_back(vid0.x());
        drawlist_in.seg_coords.push_back(vid0.y());
        drawlist_in.seg_coords.push_back(vid0.z());
        drawlist_in.seg_coords.push_back(vid1.x());
        drawlist_in.seg_coords.push_back(vid1.y());
        drawlist_in.seg_coords.push_back(vid1.z());

        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.r);
        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.g);
        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.b);
        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.a);
        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.r);
        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.g);
        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.b);
        drawlist_in.seg_colors.push_back(this->edge_data(eid).color.a);

        if (this->edge_data(eid).marked && drawlist_in.draw_mode & DRAW_MARKED_SEGS)
        {
            int base_addr = drawlist_in.marked_seg_coords.size()/3;
            drawlist_in.marked_segs.push_back(base_addr    );
            drawlist_in.marked_segs.push_back(base_addr + 1);

            drawlist_in.marked_seg_coords.push_back(vid0.x());
            drawlist_in.marked_seg_coords.push_back(vid0.y());
            drawlist_in.marked_seg_coords.push_back(vid0.z());
            drawlist_in.marked_seg_coords.push_back(vid1.x());
            drawlist_in.marked_seg_coords.push_back(vid1.y());
            drawlist_in.marked_seg_coords.push_back(vid1.z());
        }
    }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::slice(const float thresh, // thresh on centroids or quality
                                       const int   item,   // X, Y, Z, L, Q
                                       const int   sign,   // either LEQ or GEQ
                                       const int   mode)   // either AND or OR
{
    slicer.update(*this, thresh, item, sign, mode); // update per element visibility flags
    updateGL();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_mesh(const bool b)
{
    if (b)
    {
        drawlist_in.draw_mode  |= DRAW_TRIS;
        drawlist_out.draw_mode |= DRAW_TRIS;
    }
    else
    {
        drawlist_in.draw_mode  &= ~DRAW_TRIS;
        drawlist_out.draw_mode &= ~DRAW_TRIS;
    }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_mesh_flat()
{
    drawlist_in.draw_mode  |=  DRAW_TRI_FLAT;
    drawlist_in.draw_mode  &= ~DRAW_TRI_SMOOTH;
    drawlist_in.draw_mode  &= ~DRAW_TRI_POINTS;
    drawlist_out.draw_mode |=  DRAW_TRI_FLAT;
    drawlist_out.draw_mode &= ~DRAW_TRI_SMOOTH;
    drawlist_out.draw_mode &= ~DRAW_TRI_POINTS;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_mesh_smooth()
{
    drawlist_in.draw_mode  |=  DRAW_TRI_SMOOTH;
    drawlist_in.draw_mode  &= ~DRAW_TRI_FLAT;
    drawlist_in.draw_mode  &= ~DRAW_TRI_POINTS;
    drawlist_out.draw_mode |=  DRAW_TRI_SMOOTH;
    drawlist_out.draw_mode &= ~DRAW_TRI_FLAT;
    drawlist_out.draw_mode &= ~DRAW_TRI_POINTS;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_mesh_points()
{
    drawlist_in.draw_mode  |=  DRAW_TRI_POINTS;
    drawlist_in.draw_mode  &= ~DRAW_TRI_FLAT;
    drawlist_in.draw_mode  &= ~DRAW_TRI_SMOOTH;
    drawlist_out.draw_mode |=  DRAW_TRI_POINTS;
    drawlist_out.draw_mode &= ~DRAW_TRI_FLAT;
    drawlist_out.draw_mode &= ~DRAW_TRI_SMOOTH;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_color()
{
    drawlist_out.draw_mode |=  DRAW_TRI_FACECOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_QUALITY;
    drawlist_out.draw_mode &= ~DRAW_TRI_TEXTURE1D;
    drawlist_out.draw_mode &= ~DRAW_TRI_TEXTURE2D;
    updateGL_out();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_quality()
{
    drawlist_out.draw_mode |=  DRAW_TRI_QUALITY;
    drawlist_out.draw_mode &= ~DRAW_TRI_FACECOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_TEXTURE1D;
    drawlist_out.draw_mode &= ~DRAW_TRI_TEXTURE2D;
    updateGL_out();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_texture1D(const GLint texture)
{
    drawlist_out.draw_mode |=  DRAW_TRI_TEXTURE1D;
    drawlist_out.draw_mode &= ~DRAW_TRI_TEXTURE2D;
    drawlist_out.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_FACECOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_QUALITY;

    load_texture(drawlist_out.tri_text_id, texture);
    updateGL_out();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_texture2D(const GLint texture, const double tex_unit_scalar)
{
    drawlist_out.draw_mode |=  DRAW_TRI_TEXTURE2D;
    drawlist_out.draw_mode &= ~DRAW_TRI_TEXTURE1D;
    drawlist_out.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_FACECOLOR;
    drawlist_out.draw_mode &= ~DRAW_TRI_QUALITY;

    drawlist_out.tri_text_unit_scalar = tex_unit_scalar;
    load_texture(drawlist_out.tri_text_id, texture);
    updateGL_out();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_wireframe(const bool b)
{
    if (b) drawlist_out.draw_mode |=  DRAW_SEGS;
    else   drawlist_out.draw_mode &= ~DRAW_SEGS;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_wireframe_color(const Color & c)
{
    this->edge_set_color(c); // NOTE: this will change alpha for ANY adge (both interior and boundary)
    updateGL();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_wireframe_width(const float width)
{
    drawlist_out.seg_width = width;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_face_wireframe_transparency(const float alpha)
{
    this->edge_set_alpha(alpha); // NOTE: this will change alpha for ANY adge (both interior and boundary)
    updateGL();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_color()
{
    drawlist_in.draw_mode |=  DRAW_TRI_FACECOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_QUALITY;
    drawlist_in.draw_mode &= ~DRAW_TRI_TEXTURE1D;
    drawlist_in.draw_mode &= ~DRAW_TRI_TEXTURE2D;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_quality()
{
    drawlist_in.draw_mode |=  DRAW_TRI_QUALITY;
    drawlist_in.draw_mode &= ~DRAW_TRI_FACECOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_TEXTURE1D;
    drawlist_in.draw_mode &= ~DRAW_TRI_TEXTURE2D;
    updateGL_in();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_texture1D(const GLint texture)
{
    drawlist_in.draw_mode |=  DRAW_TRI_TEXTURE1D;
    drawlist_in.draw_mode &= ~DRAW_TRI_TEXTURE2D;
    drawlist_in.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_FACECOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_QUALITY;

    load_texture(drawlist_in.tri_text_id, texture);
    updateGL_in();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_texture2D(const GLint texture, const double tex_unit_scalar)
{
    drawlist_in.draw_mode |=  DRAW_TRI_TEXTURE2D;
    drawlist_in.draw_mode &= ~DRAW_TRI_TEXTURE1D;
    drawlist_in.draw_mode &= ~DRAW_TRI_VERTCOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_FACECOLOR;
    drawlist_in.draw_mode &= ~DRAW_TRI_QUALITY;

    drawlist_in.tri_text_unit_scalar = tex_unit_scalar;
    load_texture(drawlist_in.tri_text_id, texture);
    updateGL_in();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_wireframe(const bool b)
{
    if (b) drawlist_in.draw_mode |=  DRAW_SEGS;
    else   drawlist_in.draw_mode &= ~DRAW_SEGS;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_wireframe_color(const Color & c)
{
    this->edge_set_color(c); // NOTE: this will change color for ANY adge (both interior and boundary)
    updateGL();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_wireframe_width(const float width)
{
    drawlist_in.seg_width = width;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

template<class M, class V, class E, class F, class P>
CINO_INLINE
void DrawablePolyhedralmesh<M,V,E,F,P>::show_cell_wireframe_transparency(const float alpha)
{
    this->edge_set_alpha(alpha); // NOTE: this will change alpha for ANY adge (both interior and boundary)
    updateGL();
}

}

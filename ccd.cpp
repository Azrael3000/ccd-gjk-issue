#include <stdio.h>
#include <stdlib.h>

#include <ccd/ccd.h>
#include <ccd/quat.h>

struct box
{
    ccd_quat_t quat;
    ccd_vec3_t pos;
    ccd_vec3_t size;
};

void support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    // assume that obj_t is user-defined structure that holds info about
    // object (in this case box: x, y, z, pos, quat - dimensions of box,
    // position and rotation)
    box *obj = (box *)_obj;
    ccd_vec3_t dir;
    ccd_quat_t qinv;

    // apply rotation on direction vector
    ccdVec3Copy(&dir, _dir);
    ccdQuatInvert2(&qinv, &obj->quat);
    ccdQuatRotVec(&dir, &qinv);

    // compute support point in specified direction
    ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * ccdVec3X(&(obj->size)) * CCD_REAL(0.5),
                  ccdSign(ccdVec3Y(&dir)) * ccdVec3Y(&(obj->size)) * CCD_REAL(0.5),
                  ccdSign(ccdVec3Z(&dir)) * ccdVec3Z(&(obj->size)) * CCD_REAL(0.5));

    // transform support point according to position and rotation of object
    ccdQuatRotVec(v, &obj->quat);
    ccdVec3Add(v, &obj->pos);
}

void center(const void *_obj, ccd_vec3_t *center)
{
    box *obj = (box *)_obj;
    ccdVec3Copy(center, &obj->pos);
}

int main()
{
    box a;
    box b;
    ccdVec3Set(&(a.pos),0.0, 0.0, 0.0);
    ccdVec3Set(&(b.pos),0.0, 0.0, 1.9);
    ccdQuatSet(&(a.quat),0.0, 0.0, 0.0, 1.0);
    ccdQuatSet(&(b.quat),0.0, 0.0, 0.0, 1.0);
    ccdVec3Set(&(a.size),2.0, 2.0, 2.0);
    ccdVec3Set(&(b.size),2.0, 2.0, 2.0);

    //ccd_vec3_t v, dir;
    //ccdVec3Set(&dir,1.0, 1.0, 0.0);
    //support((void*) &a, &dir, &v);
    //printf("dir %e %e %e\n", ccdVec3X(&dir), ccdVec3Y(&dir), ccdVec3Z(&dir));
    //printf("v %e %e %e\n", ccdVec3X(&v), ccdVec3Y(&v), ccdVec3Z(&v));

    ccd_t ccd;
    CCD_INIT(&ccd);

    ccd.support1 = support;
    ccd.support2 = support;
    // gjk specific
    ccd.max_iterations = 100;
    // mpr specific
    ccd.center1 = center;
    ccd.center2 = center;
    ccd.mpr_tolerance = 1e-7;

    int intersect;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;

    intersect = ccdGJKPenetration((void*)&a,(void*)&b,&ccd,&depth,&dir,&pos);
    printf("GJK: intersect: %d depth: %e\n", intersect, depth);
    printf("GJK: dir: %e %e %e\n", ccdVec3X(&dir), ccdVec3Y(&dir), ccdVec3Z(&dir));
    printf("GJK: pos: %e %e %e\n", ccdVec3X(&pos), ccdVec3Y(&pos), ccdVec3Z(&pos));

    intersect = ccdMPRPenetration((void*)&a,(void*)&b,&ccd,&depth,&dir,&pos);
    printf("MPR: intersect: %d depth: %e\n", intersect, depth);
    printf("MPR: dir: %e %e %e\n", ccdVec3X(&dir), ccdVec3Y(&dir), ccdVec3Z(&dir));
    printf("MPR: pos: %e %e %e\n", ccdVec3X(&pos), ccdVec3Y(&pos), ccdVec3Z(&pos));

    return 0;
}

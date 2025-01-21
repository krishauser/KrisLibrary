#include "SpiralIterator.h"
#include <KrisLibrary/math/math.h>
#include <KrisLibrary/errors.h>
#include <limits.h>
#include <stdio.h>

using namespace Math;

SpiralIterator::SpiralIterator(const IntPair& _center)
: center(_center), imin(INT_MIN,INT_MIN), imax(INT_MAX,INT_MAX), dmax(INT_MAX),stage(-1)
{
    //initialize from start stage
    operator ++();
}

SpiralIterator::SpiralIterator(const IntPair& _center, const IntPair& _imin, const IntPair& _imax)
: center(_center), imin(_imin), imax(_imax), n(1), stage(-1)
{
    Assert(imin.a <= imax.a);
    Assert(imin.b <= imax.b);
    dmax = Max(Max(abs(imin.a - center.a), abs(imin[1] - center[1])),Max(abs(imax[0] - center[0]), abs(imax[1] - center[1])));
    
    //advance until current is inside the box
    operator ++();
}

void SpiralIterator::operator ++()
{
    while(!isDone()) {
        if(inc()) return;
    }
}

bool SpiralIterator::inc()
{
    switch(stage) {
    case -1:
        current = center;
        stage = 0;
        n = 0;
        return false;
    case 0:  //start state
        stage = 1;
        n = 1;
        if(current.a >= imin.a && current.a <= imax.a && current.b >= imin.b && current.b <= imax.b) {
            //output
            return true;
        }
        return false; //don't output
    case 1:
        current_corner = center;
        current.a = Max(current.a+1,imin.a);
        range_end = Min(current_corner.a + n,imax.a);
        //printf("1: Corner %d %d. Going from x=%d to %d\n",current_corner.a,current_corner.b,current.a,range_end);
        if(current.b < imin.b || current.b > imax.b) { //skip 
            stage = 3;
            return false;
        }
        else {
            stage = 2;
            return true;
        }
    case 2:
        if(current.a >= range_end) {
            stage = 3;
            return false;
        }
        current.a++;
        return true;
    case 3:
        current_corner.a += n;
        current = current_corner;
        current.b = Max(current.b+1,imin.b);
        range_end = Min(current_corner.b + n,imax.b);
        //printf("3: Corner %d %d. Going from y=%d to %d\n",current_corner.a,current_corner.b,current.b,range_end);
        if(current.a < imin.a || current.a > imax.a) { //skip 
            stage = 5;
            return false;
        }
        stage = 4;
        return true;
    case 4:
        if(current.b >= range_end) {
            stage = 5;
            return false;
        }
        current.b++;
        return true;
    case 5:
        current_corner.b += n;
        current = current_corner;
        n += 1;
        current.a = Min(current.a-1,imax.a);
        range_end = Max(current_corner.a - n,imin.a);
        //printf("5: Corner %d %d. Going from x=%d to %d\n",current_corner.a,current_corner.b,current.a,range_end);
        if(current.b < imin.b || current.b > imax.b) { //skip 
            stage = 7;
            return false;
        }
        stage = 6;
        return true;
    case 6:
        if(current.a <= range_end) {
            stage = 7;
            return false;
        }
        current.a--;
        return true;
    case 7:
        current_corner.a -= n;
        current = current_corner;
        current.b = Min(current.b-1,imax.b);
        range_end = Max(current_corner.b - n,imin.b);
        //printf("7: Corner %d %d. Going from y=%d to %d\n",current_corner.a,current_corner.b,current.b,range_end);
        if(current.a < imin.a || current.a > imax.a) { //skip 
            stage = 9;
            return false;
        }
        else {
            stage = 8;
            return true;
        }
    case 8:
        if(current.b <= range_end) {
            stage = 9;
            return false;
        }
        current.b--;
        return true;
    case 9:
        current_corner.b -= n;
        current = current_corner;
        n += 1;
        current.a = Max(current.a+1,imin.a);
        range_end = Min(current_corner.a + n,imax.a);
        //printf("9: Corner %d %d. Going from x=%d to %d\n",current_corner.a,current_corner.b,current.a,range_end);
        if(current.b < imin.b || current.b > imax.b) { //skip 
            stage = 3;
            return false;
        }
        else {
            stage = 2;
            return true;
        }
    default:
        printf("Invalid stage %d?\n",stage);
        abort();
        return false;
    }
}

bool SpiralIterator::isDone() const
{
    return dmax < (n-1)/2 || (dmax == (n-1)/2 && stage >= 3);  
}

/*
def iterate_spiral(center : Tuple[int,int]) -> Iterator[Tuple[int,int]]:
    x, y = center
    yield x, y
    n = 1
    while True:
        for _ in range(n):
            x += 1
            yield x, y
        for _ in range(n):
            y -= 1
            yield x, y
        n += 1
        for _ in range(n):
            x -= 1
            yield x, y
        for _ in range(n):
            y += 1
            yield x, y
        n += 1
*/


/*
def iterate_spiral_bounded(center : Tuple[int,int], imin : Tuple[int,int], imax : Tuple[int,int]) -> Iterator[Tuple[int,int]]:
    x, y = center
    dmax = max(abs(imin[0] - center[0]), abs(imin[1] - center[1]), abs(imax[0] - center[0]), abs(imax[1] - center[1]))
    if imin[0] <= x <= imax[0] and imin[1] <= y <= imax[1]:
        yield x, y
    if dmax == 0:
        return
    n = 1
    while True:
        if y >= imin[1] and y <= imax[1]: #iterate down the row
            for xtmp in range(max(x+1,imin[0]),min(x+n+1,imax[0]+1)):
                yield xtmp, y
        x += n
        if n >= dmax * 2 + 1:
            break
        if x >= imin[0] and x >= imax[0]: #iterate down the column
            for ytmp in range(max(y+1,imin[1]),min(y+n+1,imax[1]+1)):
                yield x, ytmp
        y += n
        n += 1
        if y >= imin[1] and y <= imax[1]: #iterate up the row
            for xtmp in range(min(x-1,imax[0]),max(x-n-1,imin[0]-1),-1):
                yield xtmp, y
        x -= n
        if x >= imin[0] and x >= imax[0]: #iterate up the column
            for ytmp in range(min(y-1,imax[1]),max(y-n-1,imin[1]-1),-1):
                yield x, ytmp
        y -= n
        n += 1
*/



SpiralIterator3D::SpiralIterator3D(const IntTriple& _center)
: center(_center), imin(INT_MIN,INT_MIN,INT_MIN), imax(INT_MAX,INT_MAX,INT_MAX), current(_center), distance(0), dmax(INT_MAX), stage(-1), faceIterator(IntPair(0,0))
{
    operator ++();
}

SpiralIterator3D::SpiralIterator3D(const IntTriple& center, const IntTriple& imin, const IntTriple& imax)
: center(center), imin(imin), imax(imax), current(center), distance(0), stage(-1), faceIterator(IntPair(0,0))
{
    Assert(imin.a <= imax.a);
    Assert(imin.b <= imax.b);
    Assert(imin.c <= imax.c);

    dmax = Max(Max(abs(imin.a - center.a), abs(imin.b - center.b), abs(imin.c - center.c)),
                    abs(imax.a - center.a), abs(imax.b - center.b), abs(imax.c - center.c));
    operator ++();
}

void SpiralIterator3D::operator ++()
{
    while(!isDone()) {
        if(inc()) return;
    }
}

bool SpiralIterator3D::isDone() const
{
    return distance > dmax;
}

bool SpiralIterator3D::inc()
{
    switch(stage) {
    case -1: //start state
        distance = 1;
        stage = 0;
        if(imin.a <= center.a && center.a <= imax.a && imin.b <= center.b && center.b <= imax.b && imin.c <= center.c && center.c <= imax.c) {
            return true;
        }
        return false;
    case 0: //start of shell at given distance
        {
            bool contains_x = imin.a <= center.a - distance || center.a + distance <= imax.a;
            bool contains_y = imin.b <= center.b - distance || center.b + distance <= imax.b;
            bool contains_z = imin.c <= center.c - distance || center.c + distance <= imax.c;
            bool intersect_x = center.a - distance <= imax.a && imin.a <= center.a + distance;
            bool intersect_y = center.b - distance <= imax.b && imin.b <= center.b + distance;
            bool intersect_z = center.c - distance <= imax.c && imin.c <= center.c + distance;
            bool consider_x = contains_x && (intersect_y && intersect_z);
            bool consider_y = contains_y && (intersect_x && intersect_z);
            bool consider_z = contains_z && (intersect_x && intersect_y);
            if(!consider_x && !consider_y && !consider_z) {
                // printf("Skipping distance level %d\n",distance);
                // printf("Contains %d %d %d\n",contains_x,contains_y,contains_z);
                // printf("Intersects %d %d %d\n",intersect_x,intersect_y,intersect_z);
                distance += 1;
                return false;
            }
            //set up face iterator
            IntPair umin(INT_MAX,INT_MAX), umax(INT_MIN,INT_MIN);
            if(consider_x) {
                umin.a = imin.b - center.b;
                umax.a = imax.b - center.b;
                umin.b = imin.c - center.c;
                umax.b = imax.c - center.c;
            }
            if(consider_y) {
                umin.a = Min(umin.a,imax.a - center.a);
                umax.a = Max(umax.a,imax.a - center.a);
                umin.b = Min(umin.b,imax.c - center.c);
                umax.b = Max(umax.b,imax.c - center.c);
            }
            if(consider_z) {
                umin.a = Min(umin.a,imax.a - center.a);
                umax.a = Max(umax.a,imax.a - center.a);
                umin.b = Min(umin.b,imax.b - center.b);
                umax.b = Max(umax.b,imax.b - center.b);
            }
            // printf("Face iterator range %d %d %d %d\n",umin.a,umin.b,umax.a,umax.b);
            faceIterator = SpiralIterator(IntPair(0,0),umin,umax);
            // printf("Face iterator start %d %d\n",faceIterator->a,faceIterator->b);
            stage = 1;
            return false;
        }
    case 1: //+x face
        current.set(center.a+distance,center.b+faceIterator->a,center.c+faceIterator->b);
        if(imin.b <= current.b && current.b <= imax.b && imin.c <= current.c && current.c <= imax.c) {
            stage = 2;
            return (imin.a <= current.a && current.a <= imax.a);
        }
        else { //skip to +y face
            stage = 3;
            return false;
        }
    case 2: //-x face
        current.a = center.a - distance;
        stage = 3;
        return (imin.a <= current.a && current.a <= imax.a);
    case 3: //+y face
        if(abs(faceIterator->a) >= distance) { //skip to advance
            stage = 7;
            return false;
        }
        current.set(center.a+faceIterator->a,center.b+distance,center.c+faceIterator->b);
        if(imin.a <= current.a && current.a <= imax.a && imin.c <= current.c && current.c <= imax.c) {
            stage = 4;
            return (imin.b <= current.b && current.b <= imax.b);
        }
        else { //skip to +z face
            stage = 5;
            return false;
        }
    case 4: //-y face
        current.b = center.b - distance;
        stage = 5;
        return (imin.b <= current.b && current.b <= imax.b);
    case 5: //+z face
        if(abs(faceIterator->b) >= distance) { //skip to advance
            stage = 7;
            return false;
        }
        current.set(center.a+faceIterator->a,center.b+faceIterator->b,center.c+distance);
        if(imin.a <= current.a && current.a <= imax.a && imin.b <= current.b && current.b <= imax.b) {
            stage = 6;
            return (imin.c <= current.c && current.c <= imax.c);
        }
        else { //skip to advance and decide
            stage = 7;
            return false;
        }
    case 6: //-z face
        current.c = center.c - distance;
        stage = 7;
        return (imin.c <= current.c && current.c <= imax.c);
    case 7: //advance and decide
        ++faceIterator;
        if(faceIterator.isDone() || (abs(faceIterator->a) > distance || abs(faceIterator->b) > distance)) {
            distance += 1;
            stage = 0;
            return false;
        }
        else {
            stage = 1;
            return false;
        }
    default:
        printf("Invalid stage %d?\n",stage);
        abort();
        return false;
    }
}

/*
def iterate_spiral_3d(center : Tuple[int,int,int], imin : Tuple[int,int,int], imax : Tuple[int,int,int]) -> Iterator[Tuple[int,int,int]]:
    dmax = max(abs(imin[0] - center[0]), abs(imin[1] - center[1]), abs(imin[2] - center[2]), abs(imax[0] - center[0]), abs(imax[1] - center[1]), abs(imax[2] - center[2]))
    if imin[0] <= center[0] <= imax[0] and imin[1] <= center[1] <= imax[1] and imin[2] <= center[2] <= imax[2]:
        yield center
    distance = 0
    while distance < dmax:
        distance += 1
        contains_x = imin[0] <= center[0] - distance <= imax[0] or imin[0] <= center[0] + distance <= imax[0]
        contains_y = imin[1] <= center[1] - distance <= imax[1] or imin[1] <= center[1] + distance <= imax[1]
        contains_z = imin[2] <= center[2] - distance <= imax[2] or imin[2] <= center[2] + distance <= imax[2]
        intersect_x =  center[0] - distance <= imax[0] and imin[0] <= center[0] + distance
        intersect_y =  center[1] - distance <= imax[1] and imin[1] <= center[1] + distance
        intersect_z =  center[2] - distance <= imax[2] and imin[2] <= center[2] + distance
        consider_x = contains_x and (intersect_y and intersect_z)
        consider_y = contains_y and (intersect_x and intersect_z)
        consider_z = contains_z and (intersect_x and intersect_y)
        if not consider_x and not consider_y and not consider_z:
            # print("Skipping distance",distance)
            continue
        umin, umax, vmin, vmax = None, None, None, None
        if consider_x:
            umin = imin[1]-center[1]
            umax = imax[1]-center[1]
            vmin = imin[2]-center[2]
            vmax = imax[2]-center[2]
        if consider_y:
            if umin is not None:
                umin = min(umin,imax[0]-center[0])
                umax = max(umax,imax[0]-center[0])
            else:
                umin = imax[0]-center[0]
                umax = imax[0]-center[0]
            if vmin is None:
                vmin = imax[2]-center[2]
                vmax = imax[2]-center[2]
        if consider_z:
            if umin is None:
                umin = imin[0]-center[0]
                umax = imax[0]-center[0]
            if vmin is None:
                vmin = imin[1]-center[1]
                vmax = imax[1]-center[1]
            else:
                vmin = min(vmin,imin[1]-center[1])
                vmax = max(vmax,imax[1]-center[1])
        # print("Axes considered at distance",distance,":",consider_x,consider_y,consider_z)
        # print("Spiral dims",(umin,vmin),(umax,vmax))
        assert umin is not None
        face_iter = iterate_spiral_bounded((0,0),(umin,vmin),(umax,vmax))
        #print("Spiral max size",(umax-umin+1)*(vmax-vmin+1))
        nskipped = 0
        for pt in face_iter:
            #x faces
            out = (center[0]+distance,center[1]+pt[0],center[2]+pt[1])
            if imin[1] <= out[1] and out[1] <= imax[1] and imin[2] <= out[2] and out[2] <= imax[2]:
                if imin[0] <= out[0] <= imax[0]:
                    yield out
                else:
                    nskipped += 1
                out = (center[0]-distance,out[1],out[2])
                if imin[0] <= out[0] <= imax[0]:
                    yield out
                else:
                    nskipped += 1
            else:
                nskipped += 2
            #y faces
            if abs(pt[0]) < distance:
                out = (center[0]+pt[0],center[1]+distance,center[2]+pt[1])
                if imin[0] <= out[0] <= imax[0]:
                    if imin[2] <= out[2] <= imax[2]:
                        if imin[1] <= out[1] <= imax[1]:
                            yield out
                        else:
                            nskipped += 1
                        out = (out[0],center[1]-distance,out[2])
                        if imin[1] <= out[1] <= imax[1]:
                            yield out
                        else:
                            nskipped += 1
                    else:
                        nskipped += 2
                    #z faces
                    if abs(pt[1]) < distance:
                        out = (center[0]+pt[0],center[1]+pt[1],center[2]+distance)
                        if imin[1] <= out[1] <= imax[1]:
                            if imin[2] <= out[2] <= imax[2]:
                                yield out
                            else:
                                nskipped += 1
                            out = (center[0]+pt[0],center[1]+pt[1],center[2]-distance)
                            if imin[2] <= out[2] <= imax[2]:
                                yield out
                            else:
                                nskipped += 1
                        else:
                            nskipped += 2
                else:
                    nskipped += 4
            if pt == (distance,distance) or abs(pt[0]) > distance or abs(pt[1]) > distance:
                break
        #print("Spiral skipped",nskipped)
*/
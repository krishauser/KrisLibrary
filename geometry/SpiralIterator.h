#ifndef SPIRAL_ITERATOR_H
#define SPIRAL_ITERATOR_H

#include <KrisLibrary/utils/IntPair.h>
#include <KrisLibrary/utils/IntTriple.h>

/** @brief A 2D grid iterator that starts in a center and then
 * spirals outward.
 * 
 * If imin and imax are provided, the iterator stays within the box
 * [imin.a,imax.a] x [imin.b,imax.b]. (inclusive)  The iterator is
 * efficient and will take O(1) skips if the spiral passes outside
 * the box.
 * 
 * Tests indicate this method is 5-10x slower than iterating over
 * the box in standard fashion.
 */
class SpiralIterator
{
public:
    SpiralIterator(const IntPair& center);
    SpiralIterator(const IntPair& center, const IntPair& imin, const IntPair& imax);
    SpiralIterator(const SpiralIterator& rhs) = default;
    const IntPair& operator *() const { return current; }
    const IntPair* operator ->() const { return &current; }
    SpiralIterator& operator = (const SpiralIterator& rhs) = default;
    void operator ++();
    bool isDone() const;

private:
    bool inc();
    
    IntPair center;
    IntPair imin;
    IntPair imax;
    IntPair current;
    int dmax;
    int n;
    int stage;
    IntPair current_corner;
    int range_end;
};

/** @brief An iterator that loops over a 3d volume by spiraling out
 * faces of a cube centered at the center.
 *  
 * Tests indicate this method is 5-10x slower than iterating over
 * the box in standard fashion.
 */
class SpiralIterator3D
{
public:
    SpiralIterator3D(const IntTriple& center);
    SpiralIterator3D(const IntTriple& center, const IntTriple& imin, const IntTriple& imax);
    SpiralIterator3D(const SpiralIterator3D& rhs) = default;
    const IntTriple& operator *() const { return current; }
    const IntTriple* operator ->() const { return &current; }
    SpiralIterator3D& operator = (const SpiralIterator3D& rhs) = default;
    void operator ++();
    bool isDone() const;

private:
    bool inc();

    IntTriple center;
    IntTriple imin,imax;
    IntTriple current;
    int dmax;
    int distance;
    int stage;
    SpiralIterator faceIterator;
};

#endif // SPIRAL_ITERATOR_H
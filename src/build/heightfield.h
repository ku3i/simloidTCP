#ifndef HEIGHTFIELD_H
#define HEIGHTFIELD_H

#include <string>
#include <vector>
#include <math.h>

#include <ode/ode.h>
#include <draw/drawstuff.h>

#include <basic/vector3.h>
#include <basic/constants.h>
#include <basic/color.h>

class Heightfield
{
private:
    dGeomID geometry;
    dHeightfieldDataID heightid;
    const std::string name;
    const Color4 color;

public:
    Heightfield(const dSpaceID &space, const std::string name, const Vector3 pos, const Color4 color);
    ~Heightfield();
    void draw(void);
};

class HeightfieldVector
{
public:
    HeightfieldVector( const dSpaceID&  space
                     , const std::size_t max_number_of_heightfields)
    : space(space)
    , heightfields()
    , max_number_of_heightfields(max_number_of_heightfields)
    {
        dsPrint("Creating height field vector...");
        heightfields.reserve(max_number_of_heightfields);
        dsPrint("done.\n");
    }

    ~HeightfieldVector() { dsPrint("Destroying height fields.\n"); }

    void create(std::string name, const Vector3 pos, const Color4 color) {
        if (get_size() < max_number_of_heightfields)
        {
            if (name == "")
                name = "Landscape_" + std::to_string(get_size());
            heightfields.emplace_back(space, name, pos, color);
        }
        else {
            dsError("Maximum number of height fields is %u.", max_number_of_heightfields);
        }
    }

          Heightfield& operator[](std::size_t idx)       { return heightfields[idx]; }
    const Heightfield& operator[](std::size_t idx) const { return heightfields[idx]; }

    std::size_t get_size() const { return heightfields.size(); }

    void destroy(void) { heightfields.clear(); }

private:
    const dSpaceID&          space;
    std::vector<Heightfield> heightfields;
    const std::size_t        max_number_of_heightfields;
};


class Landscape
{
public:
    Landscape(const dSpaceID &space)
    : space(space)
    , heightfields(space, constants::max_heightfields)
    { dsPrint("Creating landscape.\n"); }

    const dSpaceID&   space;
    HeightfieldVector heightfields;

    std::size_t number_of_heightfields() const { return heightfields.get_size(); }

    void create_heightfield(const std::string name, const Vector3 pos, const Color4 color) { heightfields.create(name, pos, color); }

    void print_statistics(void) const;

    void destroy(void) {
        dsPrint("Destroying Landscape.\n");
        heightfields.destroy();
    }

    ~Landscape() { dsPrint("Destroying landscape.\n"); }

};

#endif /* HEIGHTFIELD_H */

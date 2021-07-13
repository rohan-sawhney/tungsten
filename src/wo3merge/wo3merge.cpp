#include <iostream>

#include "Version.hpp"

#include "io/DirectoryChange.hpp"
#include "io/CliParser.hpp"
#include "io/ObjLoader.hpp"
#include "io/FileUtils.hpp"
#include "io/MeshIO.hpp"
#include "io/Scene.hpp"

using namespace Tungsten;

static const int OPT_VERSION = 0;
static const int OPT_HELP    = 1;

int main(int argc, const char *argv[])
{
    CliParser parser("wo3merge", "[options] inputfiles outputfile");
    parser.addOption('h', "help", "Prints this help text", false, OPT_HELP);
    parser.addOption('v', "version", "Prints version information", false, OPT_VERSION);

    parser.parse(argc, argv);

    if (parser.isPresent(OPT_VERSION)) {
        std::cout << "wo3merge, version " << VERSION_STRING << std::endl;
        return 0;
    }
    if (parser.operands().size() < 2 || parser.isPresent(OPT_HELP)) {
        parser.printHelpText();
        return 0;
    }

    std::vector<Vertex> verts;
    std::vector<TriangleI> tris;
    for (size_t i = 0; i < parser.operands().size() - 1; ++i) {
        std::vector<Vertex> vertsI;
        std::vector<TriangleI> trisI;
        MeshIO::load(Path(parser.operands()[i]), vertsI, trisI);
        for (auto &t : trisI) {
            t.v0 += verts.size();
            t.v1 += verts.size();
            t.v2 += verts.size();
        }
        verts.insert(verts.end(), vertsI.begin(), vertsI.end());
        tris .insert( tris.end(),  trisI.begin(),  trisI.end());
    }

    MeshIO::save(Path(parser.operands().back()), verts, tris);

    return 0;
}

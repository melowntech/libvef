#ifndef vadstena_libs_ve_hpp_included_
#define vadstena_libs_ve_hpp_included_

#include <vector>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"

namespace vadstena { namespace vef {

typedef boost::optional<std::string> OptionalString;

struct Texture {
    typedef std::vector<Texture> list;

    boost::filesystem::path path;
    math::Size2 size;

    Texture() {}

    Texture(const boost::filesystem::path &path) : path(path) {}
};

struct Mesh {
    boost::filesystem::path path;

    boost::filesystem::path mtlPath() const;
};

struct Window {
    typedef std::vector<Window> list;

    boost::filesystem::path path;
    Mesh mesh;
    Texture::list atlas;

    Window(const boost::filesystem::path &path) : path(path) {}
};

struct LoddedWindow {
    typedef std::vector<LoddedWindow> list;

    boost::filesystem::path path;
    Window::list lods;

    LoddedWindow(const boost::filesystem::path &path) : path(path) {}
};

struct Manifest {
    boost::optional<geo::SrsDefinition> srs;

    LoddedWindow::list windows;
};

/** Vadstena export format archive reader
 */
class VadstenaArchive {
public:
    VadstenaArchive(const boost::filesystem::path &root);

    const Manifest manifest() const { return manifest_; }

private:
    boost::filesystem::path root_;

    Manifest manifest_;
};

typedef std::size_t Id;

/** Vadstena export format archive writer
 */
class VadstenaArchiveWriter {
public:
    /** Creates new vadstena export archive. Creation fails if root directory
     *  already exists and overwrite is false.
     */
    VadstenaArchiveWriter(const boost::filesystem::path &root, bool overwrite);

    ~VadstenaArchiveWriter();

    /** Adds new lodded window collection.
     */
    Id addWindow(const OptionalString &path = boost::none);

    /** Adds new lod to window.
     */
    Id addLod(Id windowId, const OptionalString &path = boost::none);

    /** Gets window mesh.
     *  NB: Reference is invalidated on next call to (addLod, addWindow)!
     */
    Mesh& mesh(Id windowId, Id lod);

    /** Allocates new texture for window at given lod.
     *  Returns full copy of a texture.
     */
    Texture addTexture(Id windowId, Id lod, const Texture &texture);

    void setSrs(const geo::SrsDefinition &srs);

    /** Flush pending changes to storage.
     */
    void flush();

private:
    boost::filesystem::path root_;

    bool changed_;

    Manifest manifest_;
};

} } // namespace vadstena::vef

#endif // vadstena_libs_ve_hpp_included_

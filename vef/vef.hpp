/**
 * Copyright (c) 2017-20 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef vef_vef_hpp_included_
#define vef_vef_hpp_included_

#include <vector>
#include <map>
#include <memory>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/enum-io.hpp"

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"

namespace vef {

const std::string MainFile("manifest.json");

typedef boost::optional<std::string> OptionalString;
typedef boost::optional<math::Matrix4> OptionalMatrix;

struct Texture {
    typedef std::vector<Texture> list;

    enum class Format { jpg, png, jpeg2000 };

    boost::filesystem::path path;
    math::Size2 size;
    Format format;

    Texture() {}

    Texture(const boost::filesystem::path &path) : path(path) {}
};

struct Mesh {
    enum class Format { obj, gzippedObj };

    boost::filesystem::path path;
    Format format;

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
    OptionalMatrix trafo;
    Window::list lods;

    LoddedWindow(const boost::filesystem::path &path) : path(path) {}
};

struct Manifest {
    boost::optional<geo::SrsDefinition> srs;
    OptionalMatrix trafo;

    LoddedWindow::list windows;
};

OptionalMatrix windowMatrix(const Manifest &manifest
                            , const LoddedWindow &window);

typedef std::size_t Id;

/** VEF archive writer
 */
class ArchiveWriter {
public:
    /** Creates new VEF archive. Creation fails if root directory
     *  already exists and overwrite is false.
     */
    ArchiveWriter(const boost::filesystem::path &root, bool overwrite);

    ~ArchiveWriter();

    /** Adds new lodded window collection.
     */
    Id addWindow(const OptionalString &path = boost::none
                 , const OptionalMatrix &trafo = boost::none);

    /** Adds new lod to window.
     *
     * \param meshFormat on-disk mesh format
     */
    Id addLod(Id windowId, const OptionalString &path = boost::none
              , Mesh::Format meshFormat = Mesh::Format::obj);

    /** Gets window mesh.
     *  NB: Reference is invalidated on next call to (addLod, addWindow)!
     *
     * \param windowId window identifier (generated by addWindow)
     * \param lod lod (generated by addLod)
     */
    Mesh& mesh(Id windowId, Id lod);

    /** Allocates new texture for window at given lod.
     *  Returns full copy of a texture.
     *
     * \param windowId window identifier (generated by addWindow)
     * \param lod lod (generated by addLod)
     * \param texture texture to add
     * \param format on-disk texture format
     */
    Texture addTexture(Id windowId, Id lod, const Texture &texture
                       , Texture::Format format = Texture::Format::jpeg2000);

    /** Set archive SRS.
     */
    void setSrs(const geo::SrsDefinition &srs);

    /** Set archive transformation matrix.
     */
    void setTrafo(const OptionalMatrix &trafo);

    /** Flush pending changes to storage.
     */
    void flush();

private:
    boost::filesystem::path root_;

    bool changed_;

    Manifest manifest_;
};

UTILITY_GENERATE_ENUM_IO(Texture::Format,
    ((jpg))
    ((png))
    ((jpeg2000))
)

UTILITY_GENERATE_ENUM_IO(Mesh::Format,
    ((obj))
    ((gzippedObj)("obj.gz"))
)

} // namespace vef

#endif // vef_vef_hpp_included_

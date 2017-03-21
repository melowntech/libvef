#include <map>
#include <fstream>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/magic.hpp"
#include "utility/substream.hpp"
#include "utility/tar.hpp"
#include "utility/path.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "./vef.hpp"

namespace fs = boost::filesystem;

namespace vef {

namespace {

namespace constants {
std::string ManifestName("manifest.json");
std::string MeshNameFormat("mesh.%s");
std::string MtlFileName("mesh.mtl");
std::string TextureNameFormat("texture-%d.%s");
}

std::string asExtension(Mesh::Format format)
{
    switch (format) {
    case Mesh::Format::obj: return "obj";
    case Mesh::Format::gzippedObj: return "obj.gz";
    }
    throw;
}

std::string asExtension(Texture::Format format)
{
    switch (format) {
    case Texture::Format::jpg: return "jpg";
    case Texture::Format::png: return "png";
    case Texture::Format::jpeg2000: return "jp2";
    }
    throw;
}

namespace detail {

Manifest parse1(const Json::Value &value, const fs::path &basePath)
{
    Manifest mf;

    if (value.isMember("srs")) {
        mf.srs = geo::SrsDefinition::fromString(value["srs"].asString());
    }

    std::string path;
    for (const auto &jwindow : Json::check(value["windows"], Json::arrayValue
                                           , "manifest.windows"))
    {
        Json::get(path, jwindow, "path");

        mf.windows.emplace_back(basePath / path);
        auto &window(mf.windows.back());

        for (const auto &jlod : Json::check(jwindow["lods"], Json::arrayValue
                                            , "manifest.windows.lods"))
        {
            Json::check(jlod, Json::objectValue, "manifest.windows.lods[]");
            Json::get(path, jlod, "path");

            window.lods.emplace_back(window.path / path);
            auto &lod(window.lods.back());

            const auto &jmesh(Json::check
                              (jlod["mesh"], Json::objectValue
                               , "manifest.windows.lods.mesh"));

            Json::get(path, jmesh, "path");
            lod.mesh.path = lod.path / path;
            {
                std::string tmp;
                Json::get(tmp, jmesh, "format");
                lod.mesh.format = boost::lexical_cast<Mesh::Format>(tmp);
            }

            for (const auto &jtexture
                     : Json::check(jlod["atlas"], Json::arrayValue
                                   , "manifest.windows.lods.atlas"))
            {
                Json::check(jtexture, Json::objectValue
                            , "manifest.windows.lods.atlas[[");
                Json::get(path, jtexture, "path");
                lod.atlas.emplace_back(lod.path / path);
                auto &texture(lod.atlas.back());

                Json::get(texture.size.width, jtexture, "size", 0);
                Json::get(texture.size.height, jtexture, "size", 1);

                {
                    std::string tmp;
                    Json::get(tmp, jtexture, "format");
                    texture.format = boost::lexical_cast<Texture::Format>(tmp);
                }
            }
        }
    }

    return mf;
}

} // namespace detail

Manifest loadManifest(std::istream &in, const fs::path &path
                      , bool useLocalPaths)
{
    LOG(info1) << "Loading manifest from " << path  << ".";

    // load json
    Json::Value manifest;
    Json::Reader reader;
    if (!reader.parse(in, manifest)) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to parse manifest " << path << ": "
            << reader.getFormattedErrorMessages() << ".";
    }

    const auto basePath(useLocalPaths
                        ? ""
                        : path.parent_path());

    try {
        int version(0);
        Json::get(version, manifest, "version");

        switch (version) {
        case 1:
            return detail::parse1(manifest, basePath);
        }

        LOGTHROW(err1, std::runtime_error)
            << "Invalid VFE manifest format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Invalid VFE manifest format (" << e.what()
            << "); Unable to work with this manifest (file: " << path << ").";
    }
    throw;
}

Manifest loadManifest(const roarchive::IStream::pointer &in
                      , bool useLocalPaths)
{
    return loadManifest(in->get(), in->path(), useLocalPaths);
}

Manifest loadManifest(const fs::path &path, bool useLocalPaths)
{
    LOG(info1) << "Loading manifest from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
        f.peek();
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to load manifest file " << path << ".";
    }
    auto mf(loadManifest(f, path, useLocalPaths));
    f.close();
    return mf;
}

void saveManifest(std::ostream &os, const fs::path &path
                  , const Manifest &manifest)
{
    Json::Value mf(Json::objectValue);

    mf["version"] = 1;
    if (manifest.srs) {
        mf["srs"] = boost::lexical_cast<std::string>(*manifest.srs);
    }

    // following code relies on the fact that path of each entity is directly
    // below the path od parent's entity

    auto &jwindows(mf["windows"] = Json::arrayValue);

    for (const auto &window : manifest.windows) {
        auto &jwindow(jwindows.append(Json::objectValue));
        jwindow["path"] = window.path.filename().string();

        auto &jlods(jwindow["lods"] = Json::arrayValue);

        for (const auto &lod : window.lods) {
            auto &jlod(jlods.append(Json::objectValue));
            jlod["path"] = lod.path.filename().string();

            auto &jmesh(jlod["mesh"] = Json::objectValue);
            jmesh["path"] = lod.mesh.path.filename().string();
            jmesh["format"]
                = boost::lexical_cast<std::string>(lod.mesh.format);

            auto &atlas(jlod["atlas"] = Json::arrayValue);
            for (const auto &texture : lod.atlas) {
                auto &jtexture(atlas.append(Json::objectValue));
                jtexture["path"] = texture.path.filename().string();

                auto &size(jtexture["size"] = Json::arrayValue);
                size.append(texture.size.width);
                size.append(texture.size.height);

                jtexture["format"]
                    = boost::lexical_cast<std::string>(texture.format);
            }
        }
    }

    os.precision(15);
    Json::StyledStreamWriter().write(os, mf);

    (void) path;
}

void saveManifest(const fs::path &path, const Manifest &manifest)
{
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    saveManifest(f, path, manifest);
    f.close();
}

} // namespace

Archive::Archive(const fs::path &root)
    : archive_(root, constants::ManifestName)
    , manifest_(loadManifest(archive_.istream(constants::ManifestName)
                             , archive_.directio()))
{}

ArchiveWriter::ArchiveWriter(const fs::path &root
                                             , bool overwrite)
    : root_(root), changed_(false)
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (!overwrite) {
            LOGTHROW(err2, std::runtime_error)
                << "VEF archive at " << root_  << " already exists.";
        }
    }
}

ArchiveWriter::~ArchiveWriter()
{
    if (changed_ && !std::uncaught_exception()) {
        LOG(warn4)
            << "Unflushed VEF archive at " << root_
            << "; all changes made will not be reflected in the storage.";
    }
}

fs::path Mesh::mtlPath() const
{
    return path.parent_path() / constants::MtlFileName;
}

void ArchiveWriter::flush()
{
    if (!changed_) { return; }

    saveManifest(root_ / constants::ManifestName, manifest_);

    // write MTL files
    for (const auto &window : manifest_.windows) {
        for (const auto &lod : window.lods) {
            auto path(lod.mesh.mtlPath());
            LOG(info1) << "Writing " << path;
            std::ofstream f(path.string());

            int index(0);
            for (const auto &texture : lod.atlas) {
                f << "newmtl " << index
                  << "\nmap_Kd " << texture.path.filename().string()
                  << "\n";

                ++index;
            }

            f.close();
        }
    }

    changed_ = false;
}

Id ArchiveWriter::addWindow(const OptionalString &path)
{
    changed_ = true;
    auto index(manifest_.windows.size());
    if (path) {
        // TODO: convert / to _
        manifest_.windows.emplace_back(root_ / *path);
    } else {
        manifest_.windows.emplace_back
            (root_ / boost::lexical_cast<std::string>(index));
    }
    create_directories(manifest_.windows.back().path);
    return index;
}

Id ArchiveWriter::addLod(Id windowId, const OptionalString &path
                                 , Mesh::Format meshFormat)
{
    if (windowId >= manifest_.windows.size()) {
        LOGTHROW(err1, std::logic_error)
            << "Cannot add LOD to window: invalid window index " << windowId
            << ".";
    }

    changed_ = true;

    auto &window(manifest_.windows[windowId]);

    auto index(window.lods.size());
    if (path) {
        // TODO: convert / to _
        window.lods.emplace_back(window.path / *path);
    } else {
        window.lods.emplace_back
            (window.path / boost::lexical_cast<std::string>(index));
    }

    auto &windowLod(window.lods.back());
    create_directories(windowLod.path);

    windowLod.mesh.format = meshFormat;
    windowLod.mesh.path = (windowLod.path /
                           str(boost::format(constants::MeshNameFormat)
                               % asExtension(meshFormat)));

    return index;
}

Mesh& ArchiveWriter::mesh(Id windowId, Id lod)
{
    if (windowId >= manifest_.windows.size()) {
        LOGTHROW(err1, std::logic_error)
            << "Cannot get mesh from window: invalid window index " << windowId
            << ".";
    }

    changed_ = true;

    auto &window(manifest_.windows[windowId]);

    if (lod >= window.lods.size()) {
        LOGTHROW(err1, std::logic_error)
            << "Cannot get mesh from window lod: invalid lod " << lod
            << ".";
    }

    return window.lods[lod].mesh;
}

Texture ArchiveWriter::addTexture(Id windowId, Id lod, const Texture &t
                                          , Texture::Format format)
{
    if (windowId >= manifest_.windows.size()) {
        LOGTHROW(err1, std::logic_error)
            << "Cannot add texture to window: invalid window index "
            << windowId << ".";
    }

    auto &window(manifest_.windows[windowId]);

    if (lod >= window.lods.size()) {
        LOGTHROW(err1, std::logic_error)
            << "Cannot add texture to window lod: invalid lod " << lod
            << ".";
    }

    changed_ = true;

    auto &windowLod(window.lods[lod]);
    auto &atlas(windowLod.atlas);
    auto index(atlas.size());
    atlas.push_back(t);

    // set update path and return
    auto &tt(atlas.back());
    tt.format = format;
    tt.path = (windowLod.path /
               str(boost::format(constants::TextureNameFormat)
                   % index % asExtension(format)));
    return tt;
}

void ArchiveWriter::setSrs(const geo::SrsDefinition &srs)
{
    manifest_.srs = srs;
    changed_ = true;
}

} // namespace vef

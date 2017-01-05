#include <fstream>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"

#include "./vef.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vef {

namespace {

namespace constants {
std::string ManifestName("manifest.json");
std::string MeshName("mesh.obj");
// std::string TextureNameFormat("texture-%d.jp2");
std::string TextureNameFormat("texture-%d.jpg");
}

Manifest loadManifest(const fs::path &path)
{
    Manifest mf;
    return mf;

    (void) path;
}

void saveManifest(std::ostream &os, const fs::path &path
                  , const Manifest &manifest)
{
    (void) path;
    (void) manifest;

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

            auto &atlas(jlod["atlas"] = Json::arrayValue);
            for (const auto &texture : lod.atlas) {
                auto &jtexture(atlas.append(Json::objectValue));
                jtexture["path"] = texture.path.filename().string();

                auto &size(jtexture["size"] = Json::arrayValue);
                size.append(texture.size.width);
                size.append(texture.size.height);
            }
        }
    }

    os.precision(15);
    Json::StyledStreamWriter().write(os, mf);
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

VadstenaArchive::VadstenaArchive(const boost::filesystem::path &root)
    : root_(root), changed_(false)
    , manifest_(loadManifest(root / constants::ManifestName))
{
}

VadstenaArchive::VadstenaArchive(const fs::path &root, bool overwrite)
    : root_(root), changed_(false)
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (!overwrite) {
            LOGTHROW(err2, std::runtime_error)
                << "Vadstena export archive at " << root_
                << " already exists.";
        }
    }
}

VadstenaArchive::~VadstenaArchive()
{
    if (changed_ && !std::uncaught_exception()) {
        LOG(warn4)
            << "Unflushed vadstena archive at " << root_
            << "; all changes made will not be reflected in the storage.";
    }
}

boost::filesystem::path Mesh::mtlPath() const
{
    auto p(path);
    p.replace_extension(".mtl");
    return p;
}

void VadstenaArchive::flush()
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

Id VadstenaArchive::addWindow(const OptionalString &path)
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

Id VadstenaArchive::addLod(Id windowId, const OptionalString &path)
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
    windowLod.mesh.path = windowLod.path / constants::MeshName;

    return index;
}

Mesh& VadstenaArchive::mesh(Id windowId, Id lod)
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

Texture VadstenaArchive::addTexture(Id windowId, Id lod, const Texture &t)
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
    tt.path = (windowLod.path /
               str(boost::format(constants::TextureNameFormat) % index));
    return tt;
}

void VadstenaArchive::setSrs(const geo::SrsDefinition &srs)
{
    manifest_.srs = srs;
    changed_ = true;
}

} } // namespace vadstena::vef

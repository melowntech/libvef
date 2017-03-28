#include <map>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "./reader.hpp"

namespace fs = boost::filesystem;

namespace vef {

namespace {

namespace constants {
std::string ManifestName("manifest.json");
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

} // namespace

Archive::Archive(const fs::path &root)
    : archive_(root, constants::ManifestName)
    , manifest_(loadManifest(archive_.istream(constants::ManifestName), true))
{}

} // namespace vef
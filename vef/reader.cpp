/**
 * Copyright (c) 2017 Melown Technologies SE
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
#include <map>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"
#include "math/transform.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "reader.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;

namespace vef {

namespace {

namespace constants {
std::string ManifestName("manifest.json");
}

namespace detail {

void loadTrafo(boost::optional<math::Matrix4> &trafo, const Json::Value &obj)
{
    if (!obj.isMember("trafo")) { return; }

    const auto &jTrafo
        (Json::check(obj["trafo"], Json::arrayValue, "trafo"));

    trafo = math::identity4();

    for (int ii(0), j(0); j < 3; ++j) {
        for (int i(0); i < 4; ++i, ++ii) {
            (*trafo)(j, i) = jTrafo[ii].asDouble();
        }
    }
}

Manifest parse1(const Json::Value &value, const fs::path &basePath)
{
    Manifest mf;

    if (value.isMember("srs")) {
        mf.srs = geo::SrsDefinition::fromString(value["srs"].asString());
    }

    loadTrafo(mf.trafo, value);

    std::string path;
    for (const auto &jwindow : Json::check(value["windows"], Json::arrayValue
                                           , "manifest.windows"))
    {
        Json::get(path, jwindow, "path");

        mf.windows.emplace_back(basePath / path);
        auto &window(mf.windows.back());

        loadTrafo(window.trafo, jwindow);

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
    auto manifest(Json::read(in, path, "manifest"));

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

Archive::Archive(roarchive::RoArchive &archive)
    : archive_(archive.applyHint(constants::ManifestName))
    , manifest_(loadManifest(archive_.istream(constants::ManifestName), true))
{}

roarchive::IStream::pointer Archive::meshIStream(const Mesh &mesh) const
{
    switch (mesh.format) {
    case Mesh::Format::obj:
        // raw data
        return archive_.istream(mesh.path);

    case Mesh::Format::gzippedObj:
        // we have to ungzip data
        return archive_.istream
            (mesh.path, [](bio::filtering_istream &fis) {
                // use raw zlib decompressor, tell zlib to autodetect gzip
                // header
                bio::zlib_params p;
                p.window_bits |= 16;
                fis.push(bio::zlib_decompressor(p));
            });
    }
    LOGTHROW(err2, std::runtime_error)
        << "Invalid value of Mesh::Format enum: <"
        << static_cast<int>(mesh.format) << ">.";
    throw;
}

} // namespace vef

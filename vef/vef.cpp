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
#include <map>
#include <fstream>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vef.hpp"

namespace fs = boost::filesystem;

namespace vef {

namespace {

namespace constants {
std::string ManifestName(MainFile);
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

void saveTrafo(Json::Value &obj, const math::Matrix4 &trafo)
{
    auto &jTrafo(obj["trafo"] = Json::arrayValue);
    for (int j(0); j < 3; ++j) {
        for (int i(0); i < 4; ++i) {
            jTrafo.append(trafo(j, i));
        }
    }
}

void saveTrafo(Json::Value &obj, const boost::optional<math::Matrix4> &trafo)
{
    if (!trafo) { return; }
    saveTrafo(obj, *trafo);
}

void saveManifest(std::ostream &os, const fs::path &path
                  , const Manifest &manifest, const fs::path &root)
{
    Json::Value mf(Json::objectValue);

    mf["version"] = 1;
    if (manifest.srs) {
        mf["srs"] = boost::lexical_cast<std::string>(*manifest.srs);
    }

    saveTrafo(mf, manifest.trafo);

    const auto localPath([](const fs::path &path, const fs::path &root)
                         -> std::string
    {
        const auto str(utility::lexically_relative(path, root).string());
        if (str == ".") { return {}; }
        return str;
    });

    auto &jwindows(mf["windows"] = Json::arrayValue);

    for (const auto &window : manifest.windows) {
        auto &jwindow(jwindows.append(Json::objectValue));
        const auto windowPath(fs::absolute(window.path, root));
        jwindow["path"] = localPath(windowPath, root);
        saveTrafo(jwindow, window.trafo);

        auto &jlods(jwindow["lods"] = Json::arrayValue);

        for (const auto &lod : window.lods) {
            auto &jlod(jlods.append(Json::objectValue));
            const auto lodPath(fs::absolute(lod.path, windowPath));
            jlod["path"] = localPath(lod.path, windowPath);

            auto &jmesh(jlod["mesh"] = Json::objectValue);
            const auto meshPath(fs::absolute(lod.mesh.path, lodPath));
            jmesh["path"] = localPath(meshPath, lodPath);
            jmesh["format"]
                = boost::lexical_cast<std::string>(lod.mesh.format);

            auto &atlas(jlod["atlas"] = Json::arrayValue);
            for (const auto &texture : lod.atlas) {
                auto &jtexture(atlas.append(Json::objectValue));
                const auto texturePath(fs::absolute(texture.path, lodPath));
                jtexture["path"] = localPath(texture.path, lodPath);

                auto &size(jtexture["size"] = Json::arrayValue);
                size.append(texture.size.width);
                size.append(texture.size.height);

                jtexture["format"]
                    = boost::lexical_cast<std::string>(texture.format);
            }
        }
    }

    {
        Json::StreamWriterBuilder wb;
        os.precision(15);
        std::unique_ptr<Json::StreamWriter> writer(wb.newStreamWriter());
        writer->write(mf, &os);
    }

    (void) path;
}

void saveManifest(const fs::path &path, const Manifest &manifest
                  , const fs::path &root)
{
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    saveManifest(f, path, manifest, root);
    f.close();
}

} // namespace

ArchiveWriter::ArchiveWriter(const fs::path &root, bool overwrite)
    : root_(fs::absolute(root)), changed_(false)
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

    saveManifest(root_ / constants::ManifestName, manifest_, root_);

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

Id ArchiveWriter::addWindow(const OptionalString &path
                            , const OptionalMatrix &trafo)
{
    changed_ = true;
    auto index(manifest_.windows.size());
    if (path) {
        manifest_.windows.emplace_back(fs::absolute(*path, root_));
    } else {
        manifest_.windows.emplace_back
            (root_ / boost::lexical_cast<std::string>(index));
    }
    auto &window(manifest_.windows.back());
    window.trafo = trafo;
    create_directories(window.path);
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

void ArchiveWriter::setTrafo(const OptionalMatrix &trafo)
{
    manifest_.trafo = trafo;
    changed_ = true;
}

OptionalMatrix windowMatrix(const Manifest &manifest
                             , const LoddedWindow &window)
{
    if (!manifest.trafo) { return window.trafo; }
    if (!window.trafo) { return manifest.trafo; }

    return math::Matrix4(prod(*manifest.trafo, *window.trafo));
}

} // namespace vef

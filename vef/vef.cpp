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

namespace vadstena { namespace vef {

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

class FileIStream : public IStream {
public:
    FileIStream(const fs::path &path)
        : path_(path)
        , stream_(path.string())
    {}

    virtual boost::filesystem::path path() const { return path_; }
    virtual std::istream& get() { return stream_; }
    virtual void close() { stream_.close(); }

private:
    const fs::path path_;
    utility::ifstreambuf stream_;
};

class TarIStream : public IStream {
public:
    typedef utility::io::SubStreamDevice::Filedes Filedes;

    TarIStream(const fs::path &path, const Filedes &fd)
        : path_(path)
        , buffer_(path, fd), stream_(&buffer_)
    {
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
        buf_.reset(new char[1 << 16]);
        buffer_.pubsetbuf(buf_.get(), 1 << 16);
    }

    virtual std::istream& get() { return stream_; }
    virtual fs::path path() const { return path_; }
    virtual void close() {}

private:
    fs::path path_;

    std::unique_ptr<char[]> buf_;
    boost::iostreams::stream_buffer<utility::io::SubStreamDevice> buffer_;
    std::istream stream_;
};

boost::filesystem::path
findPrefix(const fs::path &path
           , const utility::tar::Reader::File::list &files)
{
    for (const auto &file : files) {
        if (file.path.filename() == constants::ManifestName) {
            return file.path.parent_path();
        }
    }

    LOGTHROW(err2, std::runtime_error)
        << "No manifest found in the archive at " << path << ".";
    throw;
}

class TarIndex {
public:
    typedef utility::io::SubStreamDevice::Filedes Filedes;

    TarIndex(utility::tar::Reader &reader)
        : path_(reader.path())
    {
        const auto files(reader.files());
        const auto prefix(findPrefix(path_, files));
        const auto fd(reader.filedes());

        for (const auto &file : files) {
            if (!utility::isPathPrefix(file.path, prefix)) { continue; }

            const auto path(utility::cutPathPrefix(file.path, prefix));
            index_.insert(map::value_type
                              (path.string()
                               , { fd, file.start, file.end() }));
        }
    }

    const Filedes& file(const std::string &path) const {
        auto findex(index_.find(path));
        if (findex == index_.end()) {
            LOGTHROW(err2, std::runtime_error)
                << "File \"" << path << "\" not found in the archive at "
                << path_ << ".";
        }
        return findex->second;
    }

private:
    const fs::path path_;
    typedef std::map<std::string, Filedes> map;
    map index_;
};

} // namespace

struct VadstenaArchive::Detail {
    typedef std::shared_ptr<Detail> pointer;

    Detail(const fs::path &root)
        : root(root), reader(root), index(reader)
    {}

    IStream::pointer istream(const fs::path &path) const;

    static pointer build(const fs::path &root);

    static bool localPath(const Detail::pointer &detail) {
        return detail.operator bool();
    }

    static bool directAccess(const Detail::pointer &detail) {
        return !detail.operator bool();
    }

    static IStream::pointer manifestStream(const fs::path &root
                                           , const Detail::pointer &detail);
    const fs::path root;

    utility::tar::Reader reader;
    TarIndex index;
};

IStream::pointer VadstenaArchive::Detail::istream(const fs::path &path) const
{
    return std::make_shared<TarIStream>(path, index.file(path.string()));
}

IStream::pointer
VadstenaArchive::Detail::manifestStream(const fs::path &root
                                        , const Detail::pointer &detail)
{
    if (detail) { return detail->istream(constants::ManifestName); }

    const auto path(root / constants::ManifestName);
    try {
        return std::make_shared<FileIStream>(path);
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to open file " << path << ".";
    }
    return {};
}

VadstenaArchive::Detail::pointer
VadstenaArchive::Detail::build(const fs::path &root)
{
    if (utility::Magic().mime(root) != "application/x-tar") {
        // not a tar, expect directory
        return {};
    }

    return std::make_shared<Detail>(root);
}

VadstenaArchive::VadstenaArchive(const fs::path &root)
    : root_(root)
    , detail_(Detail::build(root))
    , manifest_(loadManifest(*Detail::manifestStream(root, detail_)
                             , root / constants::ManifestName
                             , Detail::localPath(detail_)))
{}

IStream::pointer VadstenaArchive::istream(const fs::path &path) const
{
    if (detail_) { return detail_->istream(path); }

    // raw filesystem access
    try {
        return std::make_shared<FileIStream>(path);
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to open file " << path << ".";
    }
    return {};
}

bool VadstenaArchive::directAccess() const
{
    return Detail::directAccess(detail_);
}

VadstenaArchiveWriter::VadstenaArchiveWriter(const fs::path &root
                                             , bool overwrite)
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

VadstenaArchiveWriter::~VadstenaArchiveWriter()
{
    if (changed_ && !std::uncaught_exception()) {
        LOG(warn4)
            << "Unflushed vadstena archive at " << root_
            << "; all changes made will not be reflected in the storage.";
    }
}

fs::path Mesh::mtlPath() const
{
    return path.parent_path() / constants::MtlFileName;
}

void VadstenaArchiveWriter::flush()
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

Id VadstenaArchiveWriter::addWindow(const OptionalString &path)
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

Id VadstenaArchiveWriter::addLod(Id windowId, const OptionalString &path
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

Mesh& VadstenaArchiveWriter::mesh(Id windowId, Id lod)
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

Texture VadstenaArchiveWriter::addTexture(Id windowId, Id lod, const Texture &t
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

void VadstenaArchiveWriter::setSrs(const geo::SrsDefinition &srs)
{
    manifest_.srs = srs;
    changed_ = true;
}

} } // namespace vadstena::vef

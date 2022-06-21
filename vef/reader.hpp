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
#ifndef vef_reader_hpp_included_
#define vef_reader_hpp_included_

#include <vector>

#include <boost/optional.hpp>

#include "roarchive/roarchive.hpp"

#include "vef.hpp"

namespace vef {

/** VEF archive reader
 */
class Archive {
public:
    using list = std::vector<Archive>;

    Archive(const boost::filesystem::path &root, bool useLocalPaths = true);

    Archive(roarchive::RoArchive &archive, bool useLocalPaths = true);

    const Manifest& manifest() const { return manifest_; }

    const roarchive::RoArchive archive() const { return archive_; }

    const boost::filesystem::path path() const { return archive_.path(); }

    /** Get istream for mesh
     */
    roarchive::IStream::pointer meshIStream(const Mesh &mesh) const;

private:
    roarchive::RoArchive archive_;

    /** Loaded manifest.
     */
    Manifest manifest_;
};

} // namespace vef

#endif // vef_reader_hpp_included_

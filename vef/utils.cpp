/**
 * Copyright (c) 2022 Melown Technologies SE
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

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "vts-libs/vts/opencv/atlas.hpp"

#include "utils.hpp"

namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;

namespace vef {

namespace {

cv::Mat loadTexture(const Archive &archive, const fs::path &path)
{
    const auto &a(archive.archive());
    if (a.directio()) {
        // optimized access
        auto tex(cv::imread(a.path(path).string()));
        if (!tex.data) {
            LOGTHROW(err2, std::runtime_error)
                << "Unable to load texture from " << path << ".";
        }
        return tex;
    }

    auto is(a.istream(path));
    auto tex(cv::imdecode(is->read(), cv::IMREAD_COLOR));

    if (!tex.data) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to load texture from " << is->path() << ".";
    }

    return tex;
}

} // namespace

Atlas loadAtlas(const Archive &archive, const Window &window)
{

    vts::opencv::Atlas atlas;
    for (const auto &texture : window.atlas) {
        LOG(info2) << "Loading window texture from: " << texture.path;
        atlas.add(loadTexture(archive, texture.path));
    }
    return atlas;
}

} // namespace vef

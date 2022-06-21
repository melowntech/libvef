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

#ifndef vef_25d_po_hpp_included_
#define vef_25d_po_hpp_included_

/** 2.5D support -- common program options handling.
 */

#include <boost/optional.hpp>
#include <boost/program_options.hpp>

namespace vef {

struct Config25d {
    std::size_t extraLods = 0;
    int extraLodsSourceLod = -1;
    boost::optional<int> tweak_generateExtraLodsFrom;

    void configuration(boost::program_options::options_description &od);

    void configure(const boost::program_options::variables_map &vars);
};

inline void
Config25d::configuration(boost::program_options::options_description &od)
{
    namespace po = boost::program_options;

    od.add_options()
        ("addExtraLods"
         , po::value(&extraLods)->default_value(extraLods)
         , "Add extra LODs from 2.5D data.")

        ("extraLodsSourceLod"
         , po::value(&extraLodsSourceLod)
         ->default_value(extraLodsSourceLod)
         , "Source LOD for generting extra LODs from 2.5D data. Negative "
         "number indexes LODs from corsest to finest.")

        ("tweak_generateExtraLodsFrom"
         , po::value<int>()
         , "Generate extra meshes from given LOD. Only for debug purposes.")
        ;
}

inline void
Config25d::configure(const boost::program_options::variables_map &vars)
{
    if (vars.count("tweak_generateExtraLodsFrom")) {
        tweak_generateExtraLodsFrom
            = vars["tweak_generateExtraLodsFrom"].as<int>();
   }
}

} // namespace vef

#endif // vef_25d_po_included_

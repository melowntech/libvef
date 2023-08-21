/**
 * Copyright (c) 2021 Melown Technologies SE
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

#include "utility/raise.hpp"

#include "tiling-po.hpp"

namespace po = boost::program_options;

namespace vef {

namespace worlds {

std::map<std::string, World> known = {
    { "webmerc"
      , {
         geo::SrsDefinition(3857)
         , math::Extents3(-20037508.342789, -20037508.342789, -20037508.342789
                          , 20037508.342789, 20037508.342789, 20037508.342789)
        }
    }
};

std::vector<std::string> knownNames()
{
    std::vector<std::string> names;
    for (const auto &item : known) {
        names.push_back(item.first);
    }
    return names;
}

} // namespace worlds

void worldConfiguration(po::options_description &od)
{
    od.add_options()
        ("world", po::value<std::string>()
         , (utility::concat
            ("Enforces world definitions (SRS/extents) by known name,"
             " one of: ("
             , utility::join(worlds::knownNames(), ",")
             , "). NB: Conflicts with world.srs and world.extents."
             ).c_str()))
        ("world.srs", po::value<geo::SrsDefinition>()
         , "Enforced world SRS."
         " NB: must be used together with `world.extents`,"
         " conflicts with `world'.")
        ("world.extents", po::value<math::Extents2>()
         , "Enforced world extents."
         " NB: Division in Z-axis is placed symetrically around zero."
         " NB: must be used together with `world.srs',"
         " conflicts with `world'.")
        ;
}

void configureWorld(std::optional<vef::World> &world
                    , const po::variables_map &vars)
{
    bool w(vars.count("world"));
    bool s(vars.count("world.srs"));
    bool e(vars.count("world.extents"));

    if (w) {
        if (s || e) {
            throw po::error
                ("`world' conflicts with `world.srs' and `world.extents'");
        }

        const auto name(vars["world"].as<std::string>());

        auto fknown(worlds::known.find(name));
        if (fknown == worlds::known.end()) {
            utility::raise<po::error>("Unknown world \"%s\".", name);
        }
        world = fknown->second;
    } else {
        // sanity check
        if (s != e) {
            throw po::error
                ("Both `world.srs' and `world.extents' must be "
                 "specified together.");
        }

        if (s) {
            // both exit
            world.emplace(vars["world.srs"].as<geo::SrsDefinition>()
                          , vars["world.extents"].as<math::Extents2>());
        }
    }
}

std::ostream& World::dump(std::ostream &os, const std::string &prefix) const
{
    return os
        << prefix << "world.srs = " << srs << "\n"
        << prefix << "world.extents = " << extents << "\n"
        ;
}

} // namespace vef

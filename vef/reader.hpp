#ifndef vef_reader_hpp_included_
#define vef_reader_hpp_included_

#include "roarchive/roarchive.hpp"

#include "./vef.hpp"

namespace vef {

/** VEF archive reader
 */
class Archive {
public:
    Archive(const boost::filesystem::path &root);

    const Manifest& manifest() const { return manifest_; }

    const roarchive::RoArchive archive() const { return archive_; }

private:
    roarchive::RoArchive archive_;

    /** Loaded manifest.
     */
    Manifest manifest_;
};

} // namespace vef

#endif // vef_reader_hpp_included_

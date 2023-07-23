
#include <filesystem>
#include <string>

namespace r2k_core
{

[[nodiscard]] bool file_exists_and_correct_extension(
  const std::filesystem::path & path, const std::string & extension);

}  // namespace r2k_core

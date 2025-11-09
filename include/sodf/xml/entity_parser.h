#ifndef SODF_XML_ENTITY_PARSER_H_
#define SODF_XML_ENTITY_PARSER_H_

#include <set>
#include <unordered_map>
#include <memory>
#include <tinyxml2.h>

#include <sodf/database/database.h>

namespace sodf {
namespace xml {

/**
 * @brief Main class for loading and parsing XML scene/entity/component files.
 */
class EntityParser
{
public:
  EntityParser();
  ~EntityParser();

  // Loads entities from a file (resolves includes relative to file location)
  bool loadEntitiesFromFile(const std::string& filename, database::Database& db);

  // Loads entities from text (optionally resolve includes relative to base_dir)
  bool loadEntitiesFromText(const std::string& text, database::Database& db, const std::string& base_dir = "");

private:
  bool loadEntities(tinyxml2::XMLDocument* doc, const std::string& base_dir, database::Database& db);

  /// Pointer to the loaded XML document (lifetime is managed).
  std::unique_ptr<tinyxml2::XMLDocument> doc;
};

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_PARSER_H_

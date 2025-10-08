#pragma once
#include <string_view>
#include <tinyxml2.h>
#include <sodf/xml/scene_model.h>

namespace sodf {
namespace xml {

void ensure_owned(SceneComponent& sc);

// Returns true if tag is one of: Add / Update / Upsert / Remove
bool is_patch_tag(std::string_view tag);

// Apply a single <Overlay> element (its children may be direct components or patch blocks)
void apply_overlay(SceneObject& obj, const tinyxml2::XMLElement* overlay_xml);

// Apply one patch block element: <Add> / <Update> / <Upsert> / <Remove>
void apply_patch_block(SceneObject& obj, const tinyxml2::XMLElement* block, std::string_view op_name);

// Upsert a direct component (<Link>, <FSM>, <Origin>, etc.) by id+type
void upsert_direct_component(SceneObject& obj, const tinyxml2::XMLElement* child);

}  // namespace xml
}  // namespace sodf

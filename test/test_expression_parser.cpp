#include <sodf/xml/expression_parser.h>

#include <gtest/gtest.h>

using namespace sodf::xml;

namespace {
inline sodf::xml::XMLParseContext makeContext(tinyxml2::XMLDocument& doc, const char* filename = "test.xml")
{
  XMLParseContext ctx;
  ctx.doc = &doc;
  ctx.filename = filename;

  auto* root = doc.RootElement();

  if (!root)
  {
    ctx.object_root = nullptr;
    return ctx;
  }

  // If root is Object, use it
  if (std::string(root->Name()) == "Object")
  {
    ctx.object_root = root;
    return ctx;
  }

  // If child Object exists, use it
  if (auto* object = root->FirstChildElement("Object"))
  {
    ctx.object_root = object;
    return ctx;
  }

  // Otherwise fall back to document root
  ctx.object_root = root;
  return ctx;
}
}  // namespace

TEST(ExpressionRefs, RelativeParentAndMath)
{
  const char* xml = R"(
  <Root>
    <Object id="bio-rad:t100-thermal-cycler">
      <Origin><Transform><Orientation pitch="0.5"/></Transform></Origin>
      <Test id="yolo">
        <Position x="${../../Origin/Transform/Orientation.pitch} + 6" y="0" z="0"/>
      </Test>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* test = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Test");
  double x = evalNumberAttributeRequired(test->FirstChildElement("Position"), "x", ctx);  // uses internal context
  EXPECT_NEAR(x, 6.5, 1e-12);
}

TEST(ExpressionRefs, IndexingAndFields)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Block>
        <Orientation roll="0.1" pitch="0.2" yaw="0.3"/>
        <Orientation roll="1.1" pitch="1.2" yaw="1.3"/>
        <Position x="10" y="20" z="30"/>
        <Use a="${../Orientation[1].pitch} + ${../Position.x}"/>
      </Block>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Block")->FirstChildElement("Use");
  double a = evalNumberAttributeRequired(use, "a", ctx);
  EXPECT_NEAR(a, 1.2 + 10.0, 1e-12);
}

TEST(ExpressionRefs, IdWithSlash)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Thing id="link/base">
        <Collision><Shape><Dimensions height="0.3"/></Shape></Collision>
      </Thing>
      <Use h="${../Thing[@id=link/base]/Collision/Shape/Dimensions.height} / 2"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  double h = evalNumberAttributeRequired(use, "h", ctx);
  EXPECT_NEAR(h, 0.15, 1e-12);
}

TEST(ExpressionRefs, DocRootLookup)
{
  const char* xml = R"(
  <Root>
    <Config><Scale value="42"/></Config>
    <Object id="obj"><Use k="${//Config/Scale.value}"/></Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  double k = evalNumberAttributeRequired(use, "k", ctx);
  EXPECT_EQ(k, 42.0);
}

TEST(ExpressionRefs, NestedReference)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Params>
        <Value id="base" a="5"/>
        <!-- Reference to another reference -->
        <Value id="alias" a="${../Value[@id=base].a}"/>
        <!-- Final usage -->
        <Use b="${../Value[@id=alias].a} + 2"/>
      </Params>
    </Object>
  </Root>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);

  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Params")->FirstChildElement("Use");

  double b = evalNumberAttributeRequired(use, "b", ctx);
  EXPECT_NEAR(b, 7.0, 1e-12);
}

TEST(ExpressionRefs, RecursiveReferenceMultiHop)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Params>
        <Value id="base"  a="3"/>
        <!-- alias1 depends on base -->
        <Value id="alias1" a="${../Value[@id=base].a} * 2"/>        <!-- 3 * 2 = 6 -->
        <!-- alias2 depends on alias1 -->
        <Value id="alias2" a="${../Value[@id=alias1].a} + 1"/>      <!-- 6 + 1 = 7 -->
        <!-- use depends on alias2 and alias1 -->
        <Use c="${../Value[@id=alias2].a} + ${../Value[@id=alias1].a}"/>       <!-- 7 + 6 = 13 -->
      </Params>
    </Object>
  </Root>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Params")->FirstChildElement("Use");

  double c = evalNumberAttributeRequired(use, "c", ctx);
  EXPECT_NEAR(c, 13.0, 1e-12);
}

TEST(ExpressionRefs, RecursiveReferenceCycleDetect)
{
  // A <-> B cycle: must throw with a clear error (infinite expansion avoided)
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Params>
        <Value id="A" a="${../Value[@id=B].a} + 1"/>
        <Value id="B" a="${../Value[@id=A].a} + 1"/>
        <Use k="${../Value[@id=A].a}"/>
      </Params>
    </Object>
  </Root>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Params")->FirstChildElement("Use");

  EXPECT_THROW({ (void)evalNumberAttributeRequired(use, "k", ctx); }, std::runtime_error);
}

TEST(ExpressionRefs, CurrentElementAttr)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Params>
        <Use val="7" out="${.val} * 2"/>
      </Params>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Params")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "out", ctx), 14.0, 1e-12);
}

TEST(ExpressionRefs, ChildRootScope)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Use out="${./Param.x} + ${Param.x}">
        <Param x="3"/>
      </Use>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "out", ctx), 6.0, 1e-12);
}

TEST(ExpressionRefs, MidPathIdWithinSubtree1)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Group>
        <Node id="T"><Param p="9"/></Node>
        <Use z="${../Node[@id=T]/Param.p}"/>
      </Group>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Group")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "z", ctx), 9.0, 1e-12);
}

TEST(ExpressionRefs, MidPathIdWithinSubtree2)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Thing id="link/base"><Dims h="0.3"/></Thing>
      <Use h10="${../Thing[@id=link/base]/Dims.h} * 10"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "h10", ctx), 3.0, 1e-12);
}

TEST(ExpressionRefs, ZeroBasedIndexing)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Block>
        <Position x="10"/>
        <Position x="20"/>
        <Position x="30"/>
        <Use first="${../Position[0].x}" third="${../Position[2].x}"/>
      </Block>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Block")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "first", ctx), 10.0, 1e-12);
  EXPECT_NEAR(evalNumberAttributeRequired(use, "third", ctx), 30.0, 1e-12);
}

TEST(ExpressionRefs, IndexOutOfRangeThrows)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Block>
        <Orientation pitch="0.2"/><Orientation pitch="1.2"/>
        <Use bad="${../Orientation[5].pitch}"/>
      </Block>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Block")->FirstChildElement("Use");
  EXPECT_THROW((void)evalNumberAttributeRequired(use, "bad", ctx), std::runtime_error);
}

TEST(ExpressionRefs, UnclosedRefThrows)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Origin><Transform><Orientation pitch="0.5"/></Transform></Origin>
      <Use bad="${/Origin/Transform/Orientation.pitch + 1"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_THROW((void)evalNumberAttributeRequired(use, "bad", ctx), std::runtime_error);
}

TEST(ExpressionRefs, MissingFinalDotAttrThrows)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Origin><Transform><Orientation pitch="0.5"/></Transform></Origin>
      <Use bad="${/Origin/Transform/Orientation}"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_THROW((void)evalNumberAttributeRequired(use, "bad", ctx), std::runtime_error);
}

TEST(ExpressionRefs, UnknownAttributeThrows)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Origin><Transform><Orientation pitch="0.5"/></Transform></Origin>
      <Use bad="${/Origin/Transform/Orientation.rollo}"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_THROW((void)evalNumberAttributeRequired(use, "bad", ctx), std::runtime_error);
}

TEST(ExpressionRefs, ParentTraversalBeyondRootThrows)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Use bad="${../../../Nope.attr}"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_THROW((void)evalNumberAttributeRequired(use, "bad", ctx), std::runtime_error);
}

TEST(ExpressionRefs, ParenthesesPreservePrecedence)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Refs><Ref val="2 + 3"/></Refs>
      <!-- Should compute (2 + 3) * 2 = 10, not 2 + 3 * 2 = 8 -->
      <Use out="${/Refs/Ref.val} * 2"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "out", ctx), 10.0, 1e-12);
}

TEST(ExpressionRefs, WhitespaceToleratedInRef)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Origin><Transform><Orientation pitch="0.5"/></Transform></Origin>
      <Use x="${   /Origin / Transform /  Orientation .  pitch    } + 0.5"/>
    </Object>
  </Root>)";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "x", ctx), 1.0, 1e-12);
}

TEST(ExpressionRefs, MissingKeyInSelectorThrows)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Params><Value id="my.id" a="11"/></Params>
      <Use bad="${../Params/Value[=my.id].a}"/>
    </Object>
  </Root>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_THROW((void)evalNumberAttributeRequired(use, "bad", ctx), std::runtime_error);
}

TEST(ExpressionRefs, QuotedKeyWithDotIdWorks)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Params><Value id="my.id" a="11"/></Params>
      <!-- Correct usage with explicit key and quoted id -->
      <Use good="${../Params/Value[@id=my.id].a} + 1"/>
    </Object>
  </Root>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "good", ctx), 12.0, 1e-12);
}

TEST(ExpressionRefs, CombineMultipleRefs)
{
  const char* xml = R"(
  <Root>
    <Object id="obj">
      <Block>
        <Position x="10" y="20" z="30"/>
        <Use sum="${../Position.x} + ${../Position.z}"/>
      </Block>
    </Object>
  </Root>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
  auto ctx = makeContext(doc);
  auto* use = doc.RootElement()->FirstChildElement("Object")->FirstChildElement("Block")->FirstChildElement("Use");
  EXPECT_NEAR(evalNumberAttributeRequired(use, "sum", ctx), 40.0, 1e-12);  // 10 + 30
}

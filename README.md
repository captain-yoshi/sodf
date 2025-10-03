# SODF - Semantic Object Description Format
Like URDF/SRDF but for objects in a robot workspace.


## Expression Engine

SODF expressions allow flexible, context-aware references and inline math:
- **Math support**: expressions are evaluated using the muParser library, with built-in constants (pi, inf) and functions (sin, cos, tan, etc.).
- **Variable substitution**: values are referenced inside `${...}` and expanded before evaluation.

``` xml
<!-- Full example covering: relative, parent, object root (/), doc root (//),
     #ID.attr sugar, quoted #'id', mid-path #, 0-based [i], .attr on current. -->
<Root>
  <Config><Scale value="42"/></Config>

  <Object id="obj">
    <Params>
      <Const id="X" a="5"/>
      <Thing id="link/base">
        <Orientation pitch="0.5"/>
        <Orientation pitch="1.2"/>
        <Dims h="0.3" w="0.1"/>
      </Thing>
    </Params>

    <Block>
      <Position x="10" y="20" z="30"/>
      <Node id="T"><Param p="9"/></Node>

      <Expand
        a ="${../../Params/Thing/Orientation[1].pitch}"   <!-- parent hop + [1] (0-based) -->
        b ="${.val}"                                      <!-- current element attr -->
        c ="${/Params/Thing/Dims.h}"                      <!-- object-root anchor -->
        d ="${//Config/Scale.value}"                      <!-- doc-root anchor -->
        e ="${#X.a} + 2"                                  <!-- #ID.attr sugar + math -->
        f ="${#'link/base'/Dims.h} / 2"                   <!-- quoted id with slash, then descend -->
        g ="${../#T/Param.p}"                             <!-- mid-path # within subtree -->
        eqn="pi / 2 + inf"                                <!-- use inline math expressions -->
        sum="${../Position.x} + ${../Position.z}"         <!-- combine refs -->
        val="7"
      />
    </Block>
  </Object>
</Root>

<!-- After Evaluation -->
<Expand
  a ="1.2" 
  b ="7" 
  c ="0.3
  d ="42"
  e ="7"
  f ="0.15"
  g ="9"
  eqn="1.57079632679 + inf"
  sum="40"
  val="7"
/>
```

## URI Resolver

SODF supports a custom `sodf://` scheme to locate mesh and resource files.
This lets you reference objects without hardcoding absolute/relative paths.

Example in XML:

```xml
<Resource uri="sodf://eppendorf/twin-tec-pcr-plate-96/collision/coarse.stl"/>
```

### Configure SODF_URI_PATH
The environment variable `SODF_URI_PATH` defines where sodf:// URIs are searched.
You can specify multiple roots, separated by `:` on Linux/macOS or `;` on Windows.
The resolver checks each root in order until a match is found.

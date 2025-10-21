# SODF - Semantic Object Description Format
Like URDF/SRDF but for objects in a robot workspace.


## Expression Engine

SODF expressions allow flexible, context-aware references and inline math:
- **Math support**: expressions are evaluated using the muParser library, with built-in constants (pi, inf) and functions (sin, cos, tan, etc.).
- **Variable substitution**: values are referenced inside `${...}` and expanded before evaluation.

``` xml
<Root>
  <Config><Scale value="42"/></Config>

  <Object id="obj">
    <Params>
      <Const id="X" a="5"/>
      <Thing id="link/base">
        <Orientation pitch="0.5"/>
        <Dims h="0.3" w="0.1"/>
      </Thing>
    </Params>

    <Block>
      <Position x="10" y="20" z="30"/>
      <Node id="T"><Param p="9"/></Node>

      <Expand
        a ="${//Config/Scale.value}"                 <!-- doc-root scope        -->
        b ="${/Params/Thing/Dims.h}"                 <!-- object-root scope     -->
        h ="${/Params/Thing[0]/Dims.h}"              <!-- tag index             -->
        c ="${/Params/Thing[@id=link/base]/Dims.h}"  <!-- attr. filter          -->
        d ="${Child.id}"                             <!-- child-root scope 1    -->
        e ="${./Child.id}"                           <!-- child-root scope 2    -->
        f ="${.val}"                                 <!-- current element attr  -->
        g ="${../Node[@id=T]/Param.p}"               <!-- parent hop            -->
        eq1="cos(pi/2) + sind(45) + tau"             <!-- inline math 1         -->
        eq2="inf + nan"                              <!-- inline math 2         -->
        sum="${../Position.x} + ${../Position.z}"    <!-- combine expressions   -->
        val="7">

          <Child id="yoshi"/>
      </Expand>
    </Block>
  </Object>
</Root>

<!-- After Evaluation -->
<Expand
  a="42"
  b="0.3"
  h="0.3"
  c="0.3"
  d="yoshi"
  e="yoshi"
  f="7"
  g="9"
  eq1="6.990292088366134" 
  eq2="nan"
  sum="40"
  val="7"
/>
```

## URI Resolver

SODF supports a custom `sodf://` scheme to locate mesh and resource files.
This lets you reference objects without hardcoding absolute/relative paths.

Example in XML:

```xml
<External uri="sodf://eppendorf/twin-tec-pcr-plate-96/collision/coarse.stl"/>
```

### Configure SODF_URI_PATH
The environment variable `SODF_URI_PATH` defines where sodf:// URIs are searched.
You can specify multiple roots, separated by `:` on Linux/macOS or `;` on Windows.
The resolver checks each root in order until a match is found.

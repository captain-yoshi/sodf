# SODF - Semantic Object Description Format
Like URDF/SRDF but for objects in a robot workspace.

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

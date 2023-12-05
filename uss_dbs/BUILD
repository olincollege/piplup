load("@drake//tools/skylark:py.bzl", "py_binary")

package(
    default_visibility = ["//visibility:public"],
)


py_library(
    name = "uss_dbs",
    srcs = [
        "__init__.py",
        "uss_dbs_interface.py",
    ],
    data = [],
    visibility = ["//visibility:public"],
    deps = [
        "@drake//bindings/pydrake",
    ],
)


py_binary(
    name = "uss_dbs_interface_demo",
    srcs = ["uss_dbs_interface_demo.py"],
    imports = [".."],
    deps = [":uss_dbs"],
)
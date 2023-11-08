py_library(
    name = "station",
    srcs = [
        "__init__.py",
        "scenario.py",
        "station.py",
    ],
    data = ["//models"],
    imports = [".."],
    visibility = ["//visibility:public"],
    deps = [
        "//common",
        "//kinova_gen3",
        "//robotiq_2f_85",
        "//robotiq_epick",
        "//perception",
        "@drake//bindings/pydrake",
    ],
)

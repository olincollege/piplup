py_library(
    name = "station",
    srcs = [
        "__init__.py",
        "station.py",
        "scenario.py",
    ],
    data = ["//models"],
    imports = [".."],
    visibility = ["//visibility:public"],
    deps = ["//kinova_gen3","//robotiq_2f_85","//common","@drake//bindings/pydrake",],
)


py_binary(
    name = "teleop_demo",
    srcs = ["teleop_demo.py"],
    imports = [".."],
    deps = [":station"],
)

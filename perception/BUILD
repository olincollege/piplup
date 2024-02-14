filegroup(
    name = "perception_configs",
    srcs = glob([
        "*.yaml"
    ]),
    visibility = ["//visibility:public"],
)

py_library(
    name = "perception",
    srcs = [
        "__init__.py",
        "point_cloud_generator.py",
        "image_segmenter.py"
    ],
    imports = [".."],
    visibility = ["//visibility:public"],
    data = [":perception_configs"],
)
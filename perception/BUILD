py_library(
    name = "perception",
    srcs = [
        "__init__.py",
        "point_cloud_generator.py",
        "image_segmenter.py",
        "all_cameras_segmenter.py"
    ],
    imports = [".."],
    visibility = ["//visibility:public"],
    deps = [],
)

filegroup(
    name = "perception_config",
    srcs = glob([
        "**/*.yaml"
    ]),
    visibility = ["//visibility:public"],
)
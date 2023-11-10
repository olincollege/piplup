cc_binary(
    name = "test",
    srcs = ["test.cc"],
    deps = [
        "//third_party:opencv",
        "//third_party:realsense",
        "@drake//common:name_value",
        "@drake//common:scoped_singleton",
        "@drake//systems/analysis:simulator",
        "@drake//systems/framework:leaf_system",
        "@drake//systems/sensors:camera_config",
        "@drake//systems/sensors:image",
    ],
)

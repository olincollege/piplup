licenses(["notice"])  # BSD license

exports_files(["LICENSE"])

cc_library(
    name = "kortex",
    srcs = glob(
        [
            "lib/release/libKortexApiCpp*.a",
        ],
    ),
    hdrs = glob([
        "include/**/*.h*",
    ]),
    includes = [
        "include/",
        "include/client/",
        "include/client_stubs/",
        "include/common/",
        "include/google/",
        "include/messages/",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
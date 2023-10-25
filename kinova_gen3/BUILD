load("@rules_cc//cc:defs.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "kinova_gen3",
    visibility = ["//visibility:public"],
    deps = [
        ":gen3_command_receiver",
        ":gen3_command_sender",
        ":gen3_constants",
        ":gen3_status_receiver",
        ":gen3_status_sender",
        "@drake//:drake_shared_library",
    ],
)

cc_library(
    name = "gen3_constants",
    hdrs = ["gen3_constants.h"],
)

cc_library(
    name = "gen3_command_receiver",
    srcs = ["gen3_command_receiver.cc"],
    hdrs = ["gen3_command_receiver.h"],
    deps = [
        ":gen3_constants",
        "@drake//:drake_shared_library",
    ],
)

cc_library(
    name = "gen3_command_sender",
    srcs = ["gen3_command_sender.cc"],
    hdrs = ["gen3_command_sender.h"],
    deps = [
        ":gen3_constants",
        "@drake//:drake_shared_library",
    ],
)

cc_library(
    name = "gen3_status_receiver",
    srcs = ["gen3_status_receiver.cc"],
    hdrs = ["gen3_status_receiver.h"],
    deps = [
        ":gen3_constants",
        "@drake//:drake_shared_library",
    ],
)

cc_library(
    name = "gen3_status_sender",
    srcs = ["gen3_status_sender.cc"],
    hdrs = ["gen3_status_sender.h"],
    deps = [
        ":gen3_constants",
        "@drake//:drake_shared_library",
    ],
)

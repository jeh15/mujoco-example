workspace(name = "mujoco_test")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
    name = "mujoco",
    branch = "main",
    remote = "https://github.com/jeh15/mujoco_bazel.git",
)

load("@mujoco//:deps.bzl", "mujoco_deps")

mujoco_deps()

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

GLFW_VERSION = "3.3.5"

http_archive(
    name = "glfw",
    build_file = "@//third_party/glfw:glfw.BUILD",
    sha256 = "a89bb6074bc12bc12fcd322dcf848af81b679ccdc695f70b29ca8a9aa066684b",
    strip_prefix = "glfw-{}".format(GLFW_VERSION),
    urls = ["https://github.com/glfw/glfw/archive/{}.zip".format(GLFW_VERSION)],
)
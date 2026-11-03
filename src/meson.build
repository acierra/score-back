add_project_arguments('-DCRPC_SERVER_API_EXPORT', language: 'cpp')

deps = [json_dep, thread_dep, glog_dep, yaml_dep, asio_dep, base64_dep]

cloud_point_rpc_sources = files(
    'rpc_coder.cpp',
    'rpc_dto.cpp',
    'rpc_server.cpp',
    'server_api.cpp',
    'service.cpp',
)

libcloud_point_rpc = shared_library(
    'cloud_point_rpc',
    cloud_point_rpc_sources,
    include_directories: inc,
    dependencies: deps,
    install: true,
    install_rpath: '$ORIGIN',
)

cloud_point_rpc_dep = declare_dependency(
    include_directories: inc,
    link_with: libcloud_point_rpc,
    dependencies: deps,
)

# Test lib
libcloud_point_rpc_test = shared_library(
    'test_cloud_point',
    'test_api.cpp',
    dependencies: cloud_point_rpc_dep,
    install: true,
    install_rpath: '$ORIGIN',
)

cloud_point_rpc_test_dep = declare_dependency(
    include_directories: inc,
    link_with: libcloud_point_rpc_test,
    dependencies: [cloud_point_rpc_dep],
)

subdir('cloud_point')

# CLI lib — links cloud_point_compute when OpenCV is available so that
# options 4 and 5 (compute-cloud) are compiled in.
cli_deps = [cloud_point_rpc_dep]
# Default viewer template for option 6 when viewer.template_dir is unset:
# the source-tree copy (right for host builds and the bind-mounted Docker
# dev container alike).
cli_cpp_args = ['-DCPRPC_VIEWER_TEMPLATE_DIR="'
                + meson.project_source_root() / 'scripts/cloud_viewer/site'
                + '"']
if opencv_dep.found()
    cli_deps += [cloud_point_compute_dep]
    cli_cpp_args += ['-DHAVE_CLOUD_POINT_COMPUTE']
endif

libcloud_point_rpc_cli = shared_library(
    'libcloud_point_rpc_cli',
    'cli.cpp',
    include_directories: inc,
    dependencies: cli_deps,
    cpp_args: cli_cpp_args,
    install: true,
)

cloud_point_rpc_cli_dep = declare_dependency(
    include_directories: inc,
    link_with: libcloud_point_rpc_cli,
    dependencies: cli_deps,
)

# Client/CLI tool (legacy stdin/stdout)
executable(
    'cloud_point_rpc_cli',
    [
        'main.cpp',
    ],
    dependencies: cloud_point_rpc_cli_dep,
    install: true,
)

# Server executable (TCP)
executable(
    'cloud_point_rpc_server',
    'server_main.cpp',
    dependencies: cloud_point_rpc_dep,
    link_args: '-pthread',
    install: true,
)

# Minimal client example
executable(
    'minimal_client',
    'minimal_client.cpp',
    dependencies: cloud_point_rpc_dep,
    install: true,
)

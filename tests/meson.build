test_sources = files(
  'test_rpc.cpp',
  'test_integration.cpp',
  'test_tcp.cpp',
  'test_cli.cpp',
  'test_config.cpp',
  'test_c_api.cpp',
  'test_base64.cpp',
  'test_serialize_image.cpp'
)
test_deps = [cloud_point_rpc_dep, cloud_point_rpc_cli_dep,
             cloud_point_rpc_test_dep, json_dep, gtest_dep,
             gtest_main_dep, gmock_dep]

if opencv_dep.found()
  message('found cloud_point_compute dependency')
  test_sources += files(
    'test_image.cpp',
    'test_stereo_matcher.cpp',
    'test_stereo_rectifier.cpp',
    'test_stereo_preprocessor.cpp',
    'test_point_cloud_builder.cpp',
    'test_point_cloud_evaluator.cpp',
    'test_reconstruction_diagnostics.cpp',
    'test_cloud_point_client.cpp',
    'test_ply_export.cpp',
    'test_viewer_export.cpp',
    'test_scared_dataset.cpp',
    'test_scared_ground_truth_loader.cpp'
  )
  test_deps += [cloud_point_compute_dep]
else
  message('cpc_dep was not found')
endif

# Lets test_config.cpp load the checked-in config.yaml.example and
# config.scared.yml regardless of the test working directory.
test_cpp_args = ['-DCPRPC_SOURCE_ROOT="' + meson.project_source_root() + '"']

test_exe = executable('unit_tests',
  test_sources,
  dependencies : test_deps,
  cpp_args : test_cpp_args)

# The suite includes RPC round-trips and an optional full-size SCARED frame.
# Allow it to finish on a two-core development VM (default 30 s is too short).
test('unit_tests', test_exe, timeout: 120)

# Python unit tests for scripts/scared_overview.py and scripts/cloud_viewer/
# (canned benchmark JSON, no binary needed; XLSX cases skip without openpyxl).
python3 = find_program('python3', required : false)
if python3.found()
  test('python_overview_outputs', python3,
       args : ['-m', 'unittest', 'discover', '-s',
               meson.project_source_root() / 'tests' / 'python'],
       workdir : meson.project_source_root())
endif

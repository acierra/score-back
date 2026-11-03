opencv_dep = dependency('opencv4',
                        fallback: ['libopencv', 'libopencv4'],
                        required: false)

if not opencv_dep.found()
    message('\'opencv\' was not found. Try install libopencv-dev or similar package on your system')
    message('cloud_point_compute library removed from compilation')
    subdir_done()
endif

cxx = meson.get_compiler('cpp')
opencv_cuda_available = cxx.has_header('opencv2/cudastereo.hpp', dependencies: opencv_dep)

cloud_point_sources = files(
    'image.cpp',
    'cpu_stereo_matcher.cpp',
    'gpu_stereo_matcher.cpp',
    'stereo_matcher_factory.cpp',
    'stereo_rectifier.cpp',
    'point_cloud_builder.cpp',
    'point_cloud_evaluator.cpp',
    'reconstruction_diagnostics.cpp',
    'cloud_point_client.cpp',
    'scared_dataset_loader.cpp',
    'scared_ground_truth_loader.cpp',
)

# base64_dep: viewer frame export (encode_viewer_frame) in cloud_point_client.
cpc_deps = [ cloud_point_rpc_dep, opencv_dep, base64_dep ]

cpp_args = []
if opencv_cuda_available
    cpp_args += '-DHAVE_OPENCV_CUDA'
endif

# Optional ximgproc (opencv_contrib) for the WLS disparity post-filter.
opencv_ximgproc_lib = disabler()
if cxx.has_header('opencv2/ximgproc/disparity_filter.hpp', dependencies: opencv_dep)
    opencv_ximgproc_lib = cxx.find_library('opencv_ximgproc', required: false)
endif
if opencv_ximgproc_lib.found()
    cpc_deps += opencv_ximgproc_lib
    cpp_args += '-DCLOUD_POINT_HAVE_OPENCV_XIMGPROC'
    message('opencv_ximgproc found: WLS disparity filter enabled')
else
    message('opencv_ximgproc not found: WLS disparity filter disabled')
endif

cloud_point_compute_lib = shared_library('cloud_point_compute',
    sources: cloud_point_sources,
    include_directories: inc,
    dependencies: cpc_deps,
    cpp_args: cpp_args,
)

cloud_point_compute_dep = declare_dependency(
    include_directories: inc,
    link_with: cloud_point_compute_lib,
    dependencies: cpc_deps
)

executable(
    'scared_dataset_server',
    'scared_dataset_server.cpp',
    dependencies: [cloud_point_compute_dep],
    install: true,
)

executable(
    'scared_dataset_benchmark',
    'scared_dataset_benchmark.cpp',
    dependencies: [cloud_point_compute_dep],
    install: true,
)

project('cloud_point_rpc', 'cpp',
  version : '0.2.0',
  default_options : ['warning_level=3', 'cpp_std=c++23'])

# Dependencies
json_dep = dependency('nlohmann_json', fallback : ['nlohmann_json', 'nlohmann_json_dep'])
thread_dep = dependency('threads')
asio_dep = dependency('asio', fallback : ['asio', 'asio_dep'])
base64_dep = dependency('base64', fallback: ['aklomp-base64', 'base64'])
# GLog via CMake fallback
cmake = import('cmake')
glog_opt = cmake.subproject_options()
glog_opt.add_cmake_defines({
  'WITH_GFLAGS': 'OFF',
  'WITH_GTEST': 'OFF',
  'CMAKE_POLICY_VERSION_MINIMUM': '3.5'
})

libtype = get_option('default_library')
if libtype == 'static'
  message('Will share static state with glog')
  glog_opt.add_cmake_defines({
    'BUILD_SHARED_LIBS': 'OFF',
  })
  add_project_arguments('-DBASE64_STATIC_DEFINE', '-DYAML_CPP_STATIC_DEFINE', language: 'cpp')
endif

glog_proj = cmake.subproject('glog', options: glog_opt)
glog_dep = glog_proj.dependency('glog')

yaml_dep = dependency('yaml-cpp', fallback : ['yaml-cpp', 'yaml_cpp_dep'])

# GTest & GMock via explicit subproject
gtest_proj = subproject('gtest')
gtest_dep = gtest_proj.get_variable('gtest_dep')
gtest_main_dep = gtest_proj.get_variable('gtest_main_dep')
gmock_dep = gtest_proj.get_variable('gmock_dep')

inc = include_directories(['include', 'rpc/include'])

subdir('src')
subdir('tests')

if host_machine.system() == 'windows' and libtype == 'shared'
  message('We are on Windows, so to prevent our ass from pain')
  devenv = environment()
  message('By the way, I hate windows :O')
  prefixed = meson.global_build_root() / 'subprojects'
  message('Prefixed dir is: ', prefixed)
  devenv.append('PATH', prefixed / 'glog')
  devenv.append('PATH', prefixed / 'googletest-1.17.0' / 'googletest')
  devenv.append('PATH', prefixed / 'googletest-1.17.0' / 'googlemock')
  devenv.append('PATH', prefixed / 'yaml-cpp-0.8.0')

  meson.add_devenv(devenv)

endif

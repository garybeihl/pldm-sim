project('pldm-sim', 'cpp',
    default_options: {
        'cpp_std': 'c++23',
        'warning_level': '2',
    },
)

libpldm_dir = get_option('libpldm_dir')

libpldm_inc = include_directories(
    libpldm_dir / 'include',
    libpldm_dir / 'build-host' / 'include',
)

cpp = meson.get_compiler('cpp')
libpldm = cpp.find_library('pldm',
    dirs: [libpldm_dir / 'build-host' / 'src'],
)

nlohmann_json_dep = dependency('nlohmann_json', include_type: 'system')

executable('pldm_sim',
    'pldm_sim.cpp',
    include_directories: libpldm_inc,
    dependencies: [libpldm, nlohmann_json_dep],
    install: false,
)

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

# By default pldm-sim is standalone: it carries its own MCTP device-side
# transport and has no external dependencies beyond libpldm. Optionally,
# -Duse_mctp_sim=true swaps that bundled transport for a shared mctp-sim
# transport library, consumed as a meson subproject.
extra_deps = []
if get_option('use_mctp_sim')
    add_project_arguments('-DUSE_MCTP_SIM', language: 'cpp')
    extra_deps += subproject('mctp-sim').get_variable('mctpsim_dep')
endif

executable('pldm_sim',
    'pldm_sim.cpp',
    include_directories: libpldm_inc,
    dependencies: [libpldm, nlohmann_json_dep] + extra_deps,
    install: false,
)
